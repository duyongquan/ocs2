//
// Created by rgrandia on 09.11.20.
//

#include "ocs2_sqp/MultipleShootingSolver.h"
#include <ocs2_core/control/FeedforwardController.h>
#include <iostream>
#include <chrono>
#include <Eigen/QR>

namespace ocs2
{

  MultipleShootingSolver::MultipleShootingSolver(MultipleShootingSolverSettings settings,
                                                 const SystemDynamicsBase *systemDynamicsPtr,
                                                 const CostFunctionBase *costFunctionPtr,
                                                 const ConstraintBase *constraintPtr)
      : Solver_BASE(),
        systemDynamicsPtr_(systemDynamicsPtr->clone()),
        costFunctionPtr_(costFunctionPtr->clone()),
        constraintPtr_(constraintPtr->clone()),
        settings_(std::move(settings))
  {
    std::cout << "creating multiple shooting solver\n";
  }

  void MultipleShootingSolver::reset()
  {
    // Solver_BASE::reset();
    // there is no Solve_BASE::reset() function. One can see GaussNewtonDDP.h. The reset function there only clears some variables of the solver itself.
    // additional reset
    std::cout << "resetting\n";
  }

  void MultipleShootingSolver::runImpl(scalar_t initTime,
                                       const vector_t &initState,
                                       scalar_t finalTime,
                                       const scalar_array_t &partitioningTimes)
  {
    // ignore partitioningTimes

    // Initialize cost
    costFunctionPtr_->setCostDesiredTrajectoriesPtr(&this->getCostDesiredTrajectories());

    // Solve the problem.
    scalar_t delta_t_ = (finalTime - initTime) / settings_.N;
    matrix_t x(settings_.nx, settings_.N + 1);
    matrix_t u(settings_.nu, settings_.N);
    if (!settings_.initPrimalSol)
    {
      x = matrix_t::Random(settings_.nx, settings_.N + 1);
      u = matrix_t::Random(settings_.nu, settings_.N);
      std::cout << "using random init\n";
    }
    else
    {
      for (int i = 0; i < settings_.N; i++)
      {
        x.col(i) = primalSolution_.stateTrajectory_[i];
        u.col(i) = primalSolution_.inputTrajectory_[i];
      }
      std::cout << "using past primal sol init\n";
    }
    settings_.initPrimalSol = true;

    scalar_t sqpTimeAll = 0.0;
    for (int i = 0; i < settings_.sqpIteration; i++)
    {
      std::cout << "\n---------------sqp iteration " << i << "----------\n";
      auto startSqpTime = std::chrono::steady_clock::now();
      matrix_t delta_x, delta_u;
      std::tie(delta_x, delta_u) = runSingleIter(*systemDynamicsPtr_, *costFunctionPtr_, *constraintPtr_, delta_t_, initTime, x, u, initState);
      x += delta_x;
      u += delta_u;
      for (int j = 0; j < settings_.N; j++)
      {
        std::cout << "x dot u is: " << x.col(j).dot(u.col(j)) << std::endl;
      }
      auto endSqpTime = std::chrono::steady_clock::now();
      auto sqpIntervalTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endSqpTime - startSqpTime);
      scalar_t sqpTime = std::chrono::duration<scalar_t, std::milli>(sqpIntervalTime).count();
      sqpTimeAll += sqpTime;
      std::cout << "SQP time one iter: " << sqpTime << "[ms]." << std::endl;
    }

    std::cout << "SQP time total: " << sqpTimeAll << "[ms]." << std::endl;

    // Fill PrimalSolution. time, state , input
    scalar_array_t timeTrajectory;
    vector_array_t stateTrajectory;
    vector_array_t inputTrajectory;
    timeTrajectory.resize(settings_.N);
    stateTrajectory.resize(settings_.N);
    inputTrajectory.resize(settings_.N);
    for (int i = 0; i < settings_.N; i++)
    {
      timeTrajectory[i] = initTime + delta_t_ * i;
      stateTrajectory[i] = x.col(i);
      inputTrajectory[i] = u.col(i);
    }
    primalSolution_.timeTrajectory_ = timeTrajectory;
    primalSolution_.stateTrajectory_ = stateTrajectory;
    primalSolution_.inputTrajectory_ = inputTrajectory;
    primalSolution_.modeSchedule_ = this->getModeSchedule();
    primalSolution_.controllerPtr_.reset(new FeedforwardController(primalSolution_.timeTrajectory_, primalSolution_.inputTrajectory_));
  }

  std::tuple<matrix_t, matrix_t> MultipleShootingSolver::runSingleIter(SystemDynamicsBase &systemDynamicsObj,
                                                                       CostFunctionBase &costFunctionObj,
                                                                       ConstraintBase &constraintObj,
                                                                       scalar_t delta_t_,
                                                                       scalar_t initTime,
                                                                       const matrix_t &x,
                                                                       const matrix_t &u,
                                                                       const vector_t &initState)
  {
    // Matrix x of shape (nx, N+1), nx = n = n_state;
    // Matrix u of shape (nu, N), nu = m = n_input;
    // Matrix pi of shape (nx, N), this is not used temporarily
    // N is the horizon length
    // Vector x_init of shape (nx, 1)

    auto startAllocTime = std::chrono::steady_clock::now();
    int N = u.cols();
    int n_state = x.rows();
    int n_input = u.rows();

    int n_constraint = 1; // should be get from the constraintObj, p = n_constraint

    // number of input
    int nnuData[N + 1];
    // number of states
    int nnxData[N + 1];
    // number of input box constraints
    int nnbuData[N + 1];
    // number of states box constraints
    int nnbxData[N + 1];
    // number of general constraints
    int nngData[N + 1];
    // number of softed constraints on state box constraints
    int nnsbxData[N + 1];
    // number of softed constraints on input box constraints
    int nnsbuData[N + 1];
    // number of softed constraints on general constraints
    int nnsgData[N + 1];

    int *iidxbx[N + 1] = {}; // box constraints for states
    scalar_t *llbx[N + 1] = {};
    scalar_t *uubx[N + 1] = {};

    int *iidxbu[N + 1] = {}; // box constraints for inputs
    scalar_t *llbu[N + 1] = {};
    scalar_t *uubu[N + 1] = {};

    scalar_t *uu_guess[N + 1] = {};
    scalar_t *xx_guess[N + 1] = {};
    scalar_t *ssl_guess[N + 1] = {};
    scalar_t *ssu_guess[N + 1] = {};

    scalar_t *CC[N] = {};
    scalar_t *DD[N] = {};
    scalar_t *llg[N] = {};
    scalar_t *uug[N] = {};

    scalar_t *ZZl[N + 1] = {};
    scalar_t *ZZu[N + 1] = {};
    scalar_t *zzl[N + 1] = {};
    scalar_t *zzu[N + 1] = {};
    int *iidxs[N + 1] = {};
    scalar_t *llls[N + 1] = {};
    scalar_t *llus[N + 1] = {};

    for (int i = 0; i < N + 1; i++)
    {
      if (settings_.constrained)
      {
        nnuData[i] = n_input - n_constraint; // m - p
      }
      else
      {
        nnuData[i] = n_input; // m
      }
      nnxData[i] = n_state;
      nnbuData[i] = 0;
      nnbxData[i] = 0;
      nngData[i] = 0;
      nnsbxData[i] = 0;
      nnsbuData[i] = 0;
      nnsgData[i] = 0;
    }
    nnuData[N] = 0; // no input in the final state x_N
    nnxData[0] = 0; // to ignore the bounding condition for x0

    int *nu = nnuData;
    int *nx = nnxData;
    int *nbu = nnbuData;
    int *nbx = nnbxData;
    int *ng = nngData;
    int *nsbx = nnsbxData;
    int *nsbu = nnsbuData;
    int *nsg = nnsgData;

    scalar_t *AA[N];
    scalar_t *BB[N];
    scalar_t *bb[N];
    scalar_t *QQ[N + 1];
    scalar_t *RR[N];
    scalar_t *SS[N];
    scalar_t *qq[N + 1];
    scalar_t *rr[N];

    matrix_array_t A_data;
    matrix_array_t B_data;
    matrix_array_t b_data;
    matrix_array_t Q_data;
    matrix_array_t R_data;
    matrix_array_t S_data;
    matrix_array_t q_data;
    matrix_array_t r_data;
    A_data.resize(N);
    B_data.resize(N);
    b_data.resize(N);
    Q_data.resize(N + 1);
    R_data.resize(N);
    S_data.resize(N);
    q_data.resize(N + 1);
    r_data.resize(N);

    // below are useful only when there are constraints
    matrix_array_t C_data;
    vector_array_t e_data;
    matrix_array_t Q1_data;
    matrix_array_t Q2_data;
    matrix_array_t R1_data;
    C_data.resize(N);
    e_data.resize(N);
    Q1_data.resize(N);
    Q2_data.resize(N);
    R1_data.resize(N);

    scalar_t operTime = initTime;

    for (int i = 0; i < N; i++)
    {
      ocs2::VectorFunctionLinearApproximation systemDynamicApprox = systemDynamicsObj.linearApproximation(operTime, x.col(i), u.col(i));
      // dx_{k+1} = A_{k} * dx_{k} + B_{k} * du_{k} + b_{k}
      // A_{k} = Id + dt * dfdx
      // B_{k} = dt * dfdu
      // b_{k} = x_{n} + dt * f(x_{n},u_{n}) - x_{n+1}
      // currently only one-step Euler integration, subject to modification to RK4
      A_data[i] = delta_t_ * systemDynamicApprox.dfdx + matrix_t::Identity(n_state, n_state);
      B_data[i] = delta_t_ * systemDynamicApprox.dfdu;
      b_data[i] = x.col(i) + delta_t_ * systemDynamicApprox.f - x.col(i + 1);
      if (i == 0)
      {
        b_data[i] += A_data[i] * (initState - x.col(0)); // to ignore the bounding condition for x0
      }

      ocs2::ScalarFunctionQuadraticApproximation costFunctionApprox = costFunctionObj.costQuadraticApproximation(operTime, x.col(i), u.col(i));
      Q_data[i] = costFunctionApprox.dfdxx;
      R_data[i] = costFunctionApprox.dfduu;
      S_data[i] = costFunctionApprox.dfdux;
      q_data[i] = costFunctionApprox.dfdx;
      r_data[i] = costFunctionApprox.dfdu;

      if (settings_.constrained)
      {
        ocs2::VectorFunctionLinearApproximation constraintApprox = constraintObj.stateInputEqualityConstraintLinearApproximation(operTime, x.col(i), u.col(i));
        // C_{k} * dx_{k} + D_{k} * du_{k} + e_{k} = 0
        // C_{k} = dfdx
        // D_{k} = dfdu
        // e_{k} = f

        // handle equality constraints using QR decomposition
        matrix_t C = constraintApprox.dfdx; // p x n
        matrix_t D = constraintApprox.dfdu; // p x m
        vector_t e = constraintApprox.f;    // p x 1
        if (i == 0)
        {
          e += C * (initState - x.col(0)); // due to x0 is not part of optimization variables
        }
        matrix_t D_transpose = D.transpose(); // m x p
        Eigen::HouseholderQR<matrix_t> qr(D_transpose);
        matrix_t Q = qr.householderQ();                            // m x m
        matrix_t R = qr.matrixQR().triangularView<Eigen::Upper>(); // m x p
        // D_transpose = Q * R
        matrix_t Q1 = Q.leftCols(n_constraint);            // m x p
        matrix_t Q2 = Q.rightCols(n_input - n_constraint); // m x (m-p)
        // Q = [Q1, Q2]
        matrix_t R1 = R.topRows(n_constraint); // p x p
        // R = [R1;
        //      0]

        // store the matrices Ck, Q1k, Q2k, R1k and vector ek for later remapping \tilde{\delta uk} -> \delta uk
        C_data[i] = C;
        e_data[i] = e;
        Q1_data[i] = Q1;
        Q2_data[i] = Q2;
        R1_data[i] = R1;

        // some intermediate variables used multiple times
        matrix_t Q1_R1invT = Q1 * (R1.transpose()).inverse();
        matrix_t Q1_R1invT_C = Q1_R1invT * C;

        // see the doc/deduction.pdf for more details
        A_data[i] = A_data[i] - B_data[i] * Q1_R1invT_C;
        b_data[i] = b_data[i] - B_data[i] * Q1_R1invT * e;
        B_data[i] = B_data[i] * Q2;

        q_data[i] = q_data[i] - Q1_R1invT_C.transpose() * r_data[i] - S_data[i] * Q1_R1invT * e - Q1_R1invT_C.transpose() * R_data[i] * Q1_R1invT * e;
        r_data[i] = Q2.transpose() * r_data[i] - Q2.transpose() * R_data[i] * Q1_R1invT * e;
        Q_data[i] = Q_data[i] - 2 * Q1_R1invT_C.transpose() * S_data[i] + Q1_R1invT_C.transpose() * R_data[i] * Q1_R1invT_C;
        S_data[i] = Q2.transpose() * S_data[i] - Q2.transpose() * R_data[i] * Q1_R1invT_C;
        R_data[i] = Q2.transpose() * R_data[i] * Q2;
      }

      AA[i] = A_data[i].data();
      BB[i] = B_data[i].data();
      bb[i] = b_data[i].data();
      QQ[i] = Q_data[i].data();
      RR[i] = R_data[i].data();
      SS[i] = S_data[i].data();
      qq[i] = q_data[i].data();
      rr[i] = r_data[i].data();

      operTime += delta_t_;
    }

    // we should have used the finalCostQuadraticApproximation defined by Q_final matrix, just like the following:
    // ocs2::ScalarFunctionQuadraticApproximation finalCostFunctionApprox = costFunctionObj.finalCostQuadraticApproximation(operTime, x.col(N));
    // but in this case, it is zero, which leads to unpenalized ending states
    // so I temporarily used costQuadraticApproximation with a random linearization point of input u
    ocs2::ScalarFunctionQuadraticApproximation finalCostFunctionApprox = costFunctionObj.costQuadraticApproximation(operTime, x.col(N), u.col(N - 1));
    Q_data[N] = 10 * finalCostFunctionApprox.dfdxx; // manually add larger penalty s.t. the final state converges to the ref state
    q_data[N] = 10 * finalCostFunctionApprox.dfdx;  // 10 is a customized number, subject to adjustment
    QQ[N] = Q_data[N].data();
    qq[N] = q_data[N].data();

    scalar_t **hA = AA;
    scalar_t **hB = BB;
    scalar_t **hb = bb;
    scalar_t **hQ = QQ;
    scalar_t **hR = RR;
    scalar_t **hS = SS;
    scalar_t **hq = qq;
    scalar_t **hr = rr;
    int **hidxbx = iidxbx;
    scalar_t **hlbx = llbx;
    scalar_t **hubx = uubx;
    int **hidxbu = iidxbu;
    scalar_t **hlbu = llbu;
    scalar_t **hubu = uubu;
    scalar_t **hC = CC;
    scalar_t **hD = DD;
    scalar_t **hlg = llg;
    scalar_t **hug = uug;
    scalar_t **hZl = ZZl;
    scalar_t **hZu = ZZu;
    scalar_t **hzl = zzl;
    scalar_t **hzu = zzu;
    int **hidxs = iidxs;
    scalar_t **hlls = llls;
    scalar_t **hlus = llus;

    scalar_t **hu_guess = uu_guess;
    scalar_t **hx_guess = xx_guess;
    scalar_t **hsl_guess = ssl_guess;
    scalar_t **hsu_guess = ssu_guess;

    // the hpipm parameters below are subject to changes, may be integratred into settings_
    int iter_max = 30;
    scalar_t alpha_min = 1e-8;
    scalar_t mu0 = 1e4;
    scalar_t tol_stat = 1e-5;
    scalar_t tol_eq = 1e-5;
    scalar_t tol_ineq = 1e-5;
    scalar_t tol_comp = 1e-5;
    scalar_t reg_prim = 1e-12;
    int warm_start = 0;
    int pred_corr = 1;
    int ric_alg = 0;

    int hpipm_status;

    int dim_size = d_ocp_qp_dim_memsize(N);
    void *dim_mem = malloc(dim_size);

    struct d_ocp_qp_dim dim;
    d_ocp_qp_dim_create(N, &dim, dim_mem);

    d_ocp_qp_dim_set_all(nx, nu, nbx, nbu, ng, nsbx, nsbu, nsg, &dim);

    int qp_size = d_ocp_qp_memsize(&dim);
    void *qp_mem = malloc(qp_size);

    struct d_ocp_qp qp;
    d_ocp_qp_create(&dim, &qp, qp_mem);

    d_ocp_qp_set_all(hA, hB, hb, hQ, hS, hR, hq, hr, hidxbx, hlbx, hubx, hidxbu, hlbu, hubu, hC, hD, hlg, hug, hZl, hZu, hzl, hzu, hidxs, hlls, hlus, &qp);

    int qp_sol_size = d_ocp_qp_sol_memsize(&dim);
    void *qp_sol_mem = malloc(qp_sol_size);

    struct d_ocp_qp_sol qp_sol;
    d_ocp_qp_sol_create(&dim, &qp_sol, qp_sol_mem);

    int ipm_arg_size = d_ocp_qp_ipm_arg_memsize(&dim);
    void *ipm_arg_mem = malloc(ipm_arg_size);

    struct d_ocp_qp_ipm_arg arg;
    d_ocp_qp_ipm_arg_create(&dim, &arg, ipm_arg_mem);

    ::hpipm_mode mode = ::hpipm_mode::SPEED;

    d_ocp_qp_ipm_arg_set_default(mode, &arg);

    d_ocp_qp_ipm_arg_set_mu0(&mu0, &arg);
    d_ocp_qp_ipm_arg_set_iter_max(&iter_max, &arg);
    d_ocp_qp_ipm_arg_set_alpha_min(&alpha_min, &arg);
    d_ocp_qp_ipm_arg_set_mu0(&mu0, &arg);
    d_ocp_qp_ipm_arg_set_tol_stat(&tol_stat, &arg);
    d_ocp_qp_ipm_arg_set_tol_eq(&tol_eq, &arg);
    d_ocp_qp_ipm_arg_set_tol_ineq(&tol_ineq, &arg);
    d_ocp_qp_ipm_arg_set_tol_comp(&tol_comp, &arg);
    d_ocp_qp_ipm_arg_set_reg_prim(&reg_prim, &arg);
    d_ocp_qp_ipm_arg_set_warm_start(&warm_start, &arg);
    d_ocp_qp_ipm_arg_set_pred_corr(&pred_corr, &arg);
    d_ocp_qp_ipm_arg_set_ric_alg(&ric_alg, &arg);

    int ipm_size = d_ocp_qp_ipm_ws_memsize(&dim, &arg);
    void *ipm_mem = malloc(ipm_size);

    struct d_ocp_qp_ipm_ws workspace;
    d_ocp_qp_ipm_ws_create(&dim, &arg, &workspace, ipm_mem);

    auto endAllocTime = std::chrono::steady_clock::now();
    auto allocIntervalTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endAllocTime - startAllocTime);
    scalar_t allocTime = std::chrono::duration<scalar_t, std::milli>(allocIntervalTime).count();
    std::cout << "Alloc time usage: " << allocTime << "[ms]." << std::endl;

    auto startSolveTime = std::chrono::steady_clock::now();

    hpipm_timer timer;
    hpipm_tic(&timer);

    d_ocp_qp_ipm_solve(&qp, &qp_sol, &arg, &workspace);
    d_ocp_qp_ipm_get_status(&workspace, &hpipm_status);

    scalar_t time_ipm = hpipm_toc(&timer);

    auto endSolveTime = std::chrono::steady_clock::now();
    auto solveIntervalTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endSolveTime - startSolveTime);
    scalar_t solveTime = std::chrono::duration<scalar_t, std::milli>(solveIntervalTime).count();
    std::cout << "Solve time usage: " << solveTime << "[ms]." << std::endl;

    if (settings_.printSolverStatus)
    {
      printf("\nHPIPM returned with flag %i.\n", hpipm_status);
      if (hpipm_status == 0)
      {
        printf("\n -> QP solved!\n");
      }
      else if (hpipm_status == 1)
      {
        printf("\n -> Solver failed! Maximum number of iterations reached\n");
      }
      else if (hpipm_status == 2)
      {
        printf("\n -> Solver failed! Minimum step length reached\n");
      }
      else if (hpipm_status == 3)
      {
        printf("\n -> Solver failed! NaN in computations\n");
      }
      else
      {
        printf("\n -> Solver failed! Unknown return flag\n");
      }
      printf("\nSolution time: %e [s]\n", time_ipm);
      printf("\n\n");
    }

    if (settings_.printSolverStatistics)
    {
      int iter;
      d_ocp_qp_ipm_get_iter(&workspace, &iter);
      scalar_t res_stat;
      d_ocp_qp_ipm_get_max_res_stat(&workspace, &res_stat);
      scalar_t res_eq;
      d_ocp_qp_ipm_get_max_res_eq(&workspace, &res_eq);
      scalar_t res_ineq;
      d_ocp_qp_ipm_get_max_res_ineq(&workspace, &res_ineq);
      scalar_t res_comp;
      d_ocp_qp_ipm_get_max_res_comp(&workspace, &res_comp);
      scalar_t *stat;
      d_ocp_qp_ipm_get_stat(&workspace, &stat);
      int stat_m;
      d_ocp_qp_ipm_get_stat_m(&workspace, &stat_m);

      printf("\nipm return = %d\n", hpipm_status);
      printf("\nipm residuals max: res_g = %e, res_b = %e, res_d = %e, res_m = %e\n", res_stat, res_eq, res_ineq, res_comp);

      printf("\nipm iter = %d\n", iter);
      printf("\nalpha_aff\tmu_aff\t\tsigma\t\talpha_prim\talpha_dual\tmu\t\tres_stat\tres_eq\t\tres_ineq\tres_comp\tlq fact\t\titref pred\titref corr\tlin res stat\tlin res eq\tlin res ineq\tlin res comp\n");
      d_print_exp_tran_mat(stat_m, iter + 1, stat, stat_m);

      printf("\nocp ipm time = %e [s]\n\n", time_ipm);
    }

    // retrieve deltaX, deltaUTilde
    auto startFillTime = std::chrono::steady_clock::now();
    matrix_t deltaX(n_state, N + 1);
    deltaX.col(0) = initState - x.col(0); // because the first variable delta x0 is not optimized. Rather it's a constant.
    matrix_t deltaU(n_input, N);
    matrix_t deltaUTilde(n_input - n_constraint, N); // now this is \tilde{\delta u}, only useful when constrained
    scalar_t uTemp[n_input];
    scalar_t xTemp[n_state];

    for (int ii = 0; ii < N; ii++)
    {
      d_ocp_qp_sol_get_x(ii + 1, &qp_sol, xTemp);
      for (int j = 0; j < n_state; j++)
      {
        deltaX(j, ii + 1) = xTemp[j];
      }
      d_ocp_qp_sol_get_u(ii, &qp_sol, uTemp);
      if (settings_.constrained)
      {
        for (int j = 0; j < n_input - n_constraint; j++) // <-- pay attention to the number of states for tilded deltaU, should be m - p
        {
          deltaUTilde(j, ii) = uTemp[j];
        }
      }
      else
      {
        for (int j = 0; j < n_input; j++) // <-- pay attention to the number of states for tilded deltaU, should be m - p
        {
          deltaU(j, ii) = uTemp[j];
        }
      }
    }
    auto endFillTime = std::chrono::steady_clock::now();
    auto fillIntervalTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endFillTime - startFillTime);
    scalar_t fillTime = std::chrono::duration<scalar_t, std::milli>(fillIntervalTime).count();
    std::cout << "Retrieve time usage: " << fillTime << "[ms]." << std::endl;

    // remap the tilde delta u to real delta u
    if (settings_.constrained)
    {
      for (int i = 0; i < N; i++)
      {
        deltaU.col(i) = Q2_data[i] * deltaUTilde.col(i) - Q1_data[i] * (R1_data[i].transpose()).inverse() * (C_data[i] * deltaX.col(i) + e_data[i]);
      }
    }

    // free the hpipm memories
    free(dim_mem);
    free(qp_mem);
    free(qp_sol_mem);
    free(ipm_arg_mem);
    free(ipm_mem);

    return std::make_tuple(deltaX, deltaU);
  }

} // namespace ocs2
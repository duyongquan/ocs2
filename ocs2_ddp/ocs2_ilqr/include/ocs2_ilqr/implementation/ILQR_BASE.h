/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

namespace ocs2
{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::ILQR_BASE(
		const controlled_system_base_t* systemDynamicsPtr,
		const derivatives_base_t* systemDerivativesPtr,
		const constraint_base_t* systemConstraintsPtr,
		const cost_function_base_t* costFunctionPtr,
		const operating_trajectories_base_t* operatingTrajectoriesPtr,
		const ILQR_Settings& settings /*= ILQR_Settings()*/,
		const LOGIC_RULES_T* logicRulesPtr /*= nullptr*/,
		const cost_function_base_t* heuristicsFunctionPtr /* = nullptr*/)

		: BASE(systemDynamicsPtr,
				systemDerivativesPtr,
				systemConstraintsPtr,
				costFunctionPtr,
				operatingTrajectoriesPtr,
				settings.ddpSettings_,
				settings.rolloutSettings_,
				logicRulesPtr,
				heuristicsFunctionPtr,
				"ILQR")
		, settings_(settings)
{}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::approximateOptimalControlProblem()  {

	for (size_t i=0; i<BASE::numPartitions_; i++) {
		size_t N = BASE::nominalTimeTrajectoriesStock_[i].size();
		// resizing the new variables containers
		// Discrete-time coefficients
		AmDtimeTrajectoryStock_[i].resize(N);
		BmDtimeTrajectoryStock_[i].resize(N);

		qDtimeTrajectoryStock_[i].resize(N);
		QvDtimeTrajectoryStock_[i].resize(N);
		QmDtimeTrajectoryStock_[i].resize(N);
		RvDtimeTrajectoryStock_[i].resize(N);
		RmDtimeTrajectoryStock_[i].resize(N);
		PmDtimeTrajectoryStock_[i].resize(N);
		RmInverseDtimeTrajectoryStock_[i].resize(N);
	}  // end of i loop

	// base method should be called after the resizes
	BASE::approximateOptimalControlProblem();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::approximateLQWorker(
		size_t workerIndex,
		const size_t& partitionIndex,
		const size_t& timeIndex)  {

	// unconstrained LQ problem
	approximateUnconstrainedLQWorker(workerIndex, partitionIndex, timeIndex);

	// discretize LQ problem
	discreteLQWorker(workerIndex, partitionIndex, timeIndex);

	const scalar_t stateConstraintPenalty = settings_.ddpSettings_.stateConstraintPenaltyCoeff_ *
				pow(settings_.ddpSettings_.stateConstraintPenaltyBase_, BASE::iteration_);

//	// modify the unconstrained LQ coefficients to constrained ones
//	approximateConstrainedLQWorker(workerIndex, partitionIndex, timeIndex, stateConstraintPenalty);

	// calculate an LQ approximate of the event times process.
	BASE::approximateEventsLQWorker(workerIndex, partitionIndex, timeIndex, stateConstraintPenalty);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::approximateUnconstrainedLQWorker(
		size_t workerIndex,
		const size_t& i,
		const size_t& k) {

	BASE::approximateUnconstrainedLQWorker(workerIndex, i, k);

	// making sure that constrained Qm is PSD
	if (settings_.ddpSettings_.useMakePSD_==true)
		BASE::makePSD(BASE::QmTrajectoryStock_[i][k]);

	// TODO: add support for the constrained ILQR
	if (BASE::nc1TrajectoriesStock_[i][k]!=0 || BASE::nc2TrajectoriesStock_[i][k]!=0 || BASE::ncIneqTrajectoriesStock_[i][k]!=0)
		throw std::runtime_error("We currently only support unconstrained ILQR.");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::discreteLQWorker(
			size_t workerIndex,
			const size_t& i,
			const size_t& k) {

	// time step
	scalar_t dt = 0.0;
	if (k+1 < BASE::nominalTimeTrajectoriesStock_[i].size()) {
		 dt = BASE::nominalTimeTrajectoriesStock_[i][k+1] - BASE::nominalTimeTrajectoriesStock_[i][k];
	}

	/*
	 * linearize system dynamics
	 */
	AmDtimeTrajectoryStock_[i][k] = BASE::AmTrajectoryStock_[i][k] * dt + state_matrix_t::Identity();
	BmDtimeTrajectoryStock_[i][k] = BASE::BmTrajectoryStock_[i][k] * dt;

	/*
	 * quadratic approximation to the cost function
	 */
	qDtimeTrajectoryStock_[i][k]  = BASE::qTrajectoryStock_[i][k] * dt;
	QvDtimeTrajectoryStock_[i][k] = BASE::QvTrajectoryStock_[i][k] * dt;
	QmDtimeTrajectoryStock_[i][k] = BASE::QmTrajectoryStock_[i][k] * dt;
	RvDtimeTrajectoryStock_[i][k] = BASE::RvTrajectoryStock_[i][k] * dt;
	RmDtimeTrajectoryStock_[i][k] = BASE::RmTrajectoryStock_[i][k] * dt;
	PmDtimeTrajectoryStock_[i][k] = BASE::PmTrajectoryStock_[i][k] * dt;
	RmInverseDtimeTrajectoryStock_[i][k] = BASE::RmInverseTrajectoryStock_[i][k] / dt;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateController() {

	for (size_t i=0; i<BASE::numPartitions_; i++)  {

		if (i<BASE::initActivePartition_ || i>BASE::finalActivePartition_) {
			BASE::nominalControllersStock_[i].clear();
			continue;
		}

		const size_t N = BASE::SsTimeTrajectoryStock_[i].size();

		BASE::nominalControllersStock_[i].timeStamp_ = BASE::SsTimeTrajectoryStock_[i];
		BASE::nominalControllersStock_[i].gainArray_.resize(N);
		BASE::nominalControllersStock_[i].biasArray_.resize(N);
		BASE::nominalControllersStock_[i].deltaBiasArray_.resize(N);

		// if the partition is not active
		if (N==0)  continue;

		// current partition update
		BASE::constraintStepSize_ = BASE::initialControllerDesignStock_[i] ? 0.0 : settings_.ddpSettings_.constraintStepSize_;

		/*
		 * perform the calculatePartitionController for partition i
		 */
		calculatePartitionController(i);

	}  // end of i loop

	// correcting for the last controller element of partitions
	for (size_t i=BASE::initActivePartition_; i<BASE::finalActivePartition_; i++) {
		BASE::nominalControllersStock_[i].gainArray_.back()       = BASE::nominalControllersStock_[i+1].gainArray_.front();
		BASE::nominalControllersStock_[i].biasArray_.back()       = BASE::nominalControllersStock_[i+1].biasArray_.front();
		BASE::nominalControllersStock_[i].deltaBiasArray_.back()  = BASE::nominalControllersStock_[i+1].deltaBiasArray_.front();
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateControllerWorker (
		size_t workerIndex,
		const size_t& partitionIndex,
		const size_t& timeIndex)  {

	const size_t& i = partitionIndex;
	const size_t& k = timeIndex;

	const state_vector_t& nominalState = BASE::nominalStateTrajectoriesStock_[i][k];
	const input_vector_t& nominalInput = BASE::nominalInputTrajectoriesStock_[i][k];

	input_state_matrix_t Lm  = HmInverseTrajectoryStock_[i][k] * GmTrajectoryStock_[i][k];
	input_vector_t       Lv  = HmInverseTrajectoryStock_[i][k] * GvTrajectoryStock_[i][k];
	input_vector_t       Lve = input_vector_t::Zero();

	BASE::nominalControllersStock_[i].gainArray_[k]   = -Lm;
	BASE::nominalControllersStock_[i].biasArray_[k] = nominalInput - BASE::nominalControllersStock_[i].gainArray_[k]*nominalState;
	BASE::nominalControllersStock_[i].deltaBiasArray_[k] = -Lv;

	// checking the numerical stability of the controller parameters
	if (settings_.ddpSettings_.checkNumericalStability_==true){
		try {
			if (!BASE::nominalControllersStock_[i].gainArray_[k].allFinite())
				throw std::runtime_error("Feedback gains are unstable.");
			if (!BASE::nominalControllersStock_[i].deltaBiasArray_[k].allFinite())
				throw std::runtime_error("feedForwardControl is unstable.");
		}
		catch(const std::exception& error)  {
			std::cerr << "what(): " << error.what() << " at time " << BASE::nominalControllersStock_[i].timeStamp_[k] << " [sec]." << std::endl;
		}
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::solveRiccatiEquationsWorker(
		size_t workerIndex,
		const size_t& partitionIndex,
		const state_matrix_t& SmFinal,
		const state_vector_t& SvFinal,
		const eigen_scalar_t& sFinal)  {

	const size_t N  = BASE::nominalTimeTrajectoriesStock_[partitionIndex].size();
	const size_t NE = BASE::nominalEventsPastTheEndIndecesStock_[partitionIndex].size();

	const scalar_t scalingStart  = BASE::partitioningTimes_[partitionIndex];
	const scalar_t scalingFinal  = BASE::partitioningTimes_[partitionIndex+1];
	const scalar_t scalingFactor = scalingStart - scalingFinal;  // this is negative

	// normalized time
	BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex].resize(N);
	for (size_t k=0; k<N; k++) {
		BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex][N-1-k] =
				(BASE::nominalTimeTrajectoriesStock_[partitionIndex][k]-scalingFinal) / scalingFactor;
	}

	// normalized event past the index
	BASE::SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex].resize(NE);
	for (size_t k=0; k<NE; k++) {
		BASE::SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex][NE-1-k] =
				N - BASE::nominalEventsPastTheEndIndecesStock_[partitionIndex][k];
	}

	// output containers resizing
	BASE::SsTimeTrajectoryStock_[partitionIndex] = BASE::nominalTimeTrajectoriesStock_[partitionIndex];
	BASE::sTrajectoryStock_[partitionIndex].resize(N);
	BASE::SvTrajectoryStock_[partitionIndex].resize(N);
	BASE::SmTrajectoryStock_[partitionIndex].resize(N);

	HmTrajectoryStock_[partitionIndex].resize(N);
	HmInverseTrajectoryStock_[partitionIndex].resize(N);
	GmTrajectoryStock_[partitionIndex].resize(N);
	GvTrajectoryStock_[partitionIndex].resize(N);

	// terminate if the partition is not active
	if (N==0) return;

	// switching times
	size_array_t SsSwitchingTimesIndices;
	SsSwitchingTimesIndices.reserve(NE+2);
	SsSwitchingTimesIndices.push_back( 0 );
	for (size_t k=0; k<NE; k++) {
		const size_t& index = BASE::nominalEventsPastTheEndIndecesStock_[partitionIndex][k];
		SsSwitchingTimesIndices.push_back( index );
	}
	SsSwitchingTimesIndices.push_back( N );

	// final temporal values
	state_matrix_t SmFinalTemp = SmFinal;
	state_vector_t SvFinalTemp = SvFinal;
	eigen_scalar_t sFinalTemp  = sFinal;

	/*
	 * solving the Riccati equations
	 */
	int beginTimeItr;
	int endTimeItr;

	for (int i=NE; i>=0; i--) {

		beginTimeItr = SsSwitchingTimesIndices[i];   // similar to std::begin()
		endTimeItr   = SsSwitchingTimesIndices[i+1]; // similar to std::end()

		/*
		 * solution at final time of an interval (uses the continuous-time formulation)
		 */
		const size_t finalIndex = endTimeItr-1;
		// note that these are the continuous time coefficients
		const state_input_matrix_t& Bmc = BASE::BmTrajectoryStock_[partitionIndex][finalIndex];
		const input_vector_t&       Rvc = BASE::RvTrajectoryStock_[partitionIndex][finalIndex];
		const input_matrix_t&       Rmc = BASE::RmTrajectoryStock_[partitionIndex][finalIndex];
		const input_state_matrix_t& Pmc = BASE::PmTrajectoryStock_[partitionIndex][finalIndex];
		const input_matrix_t&       RmcInverse = BASE::RmInverseTrajectoryStock_[partitionIndex][finalIndex];

		BASE::sTrajectoryStock_[partitionIndex][finalIndex]  = sFinalTemp;
		BASE::SvTrajectoryStock_[partitionIndex][finalIndex] = SvFinalTemp;
		BASE::SmTrajectoryStock_[partitionIndex][finalIndex] = SmFinalTemp;

		HmTrajectoryStock_[partitionIndex][finalIndex] = Rmc;
		HmInverseTrajectoryStock_[partitionIndex][finalIndex] = RmcInverse;
		GmTrajectoryStock_[partitionIndex][finalIndex] = Pmc + Bmc.transpose() * SmFinalTemp;
		GvTrajectoryStock_[partitionIndex][finalIndex] = Rvc + Bmc.transpose() * SvFinalTemp;


		// solve Riccati equations if interval length is not zero
		if (beginTimeItr < endTimeItr-1) {
			for (int k=endTimeItr-2; k>=beginTimeItr; k--) {

				const state_matrix_t&       Am = AmDtimeTrajectoryStock_[partitionIndex][k];
				const state_input_matrix_t& Bm = BmDtimeTrajectoryStock_[partitionIndex][k];
				const eigen_scalar_t&       q  = qDtimeTrajectoryStock_[partitionIndex][k];
				const state_vector_t&       Qv = QvDtimeTrajectoryStock_[partitionIndex][k];
				const state_matrix_t&       Qm = QmDtimeTrajectoryStock_[partitionIndex][k];
				const input_vector_t&       Rv = RvDtimeTrajectoryStock_[partitionIndex][k];
				const input_matrix_t&       Rm = RmDtimeTrajectoryStock_[partitionIndex][k];
				const input_state_matrix_t& Pm = PmDtimeTrajectoryStock_[partitionIndex][k];

				input_matrix_t&       Hm = HmTrajectoryStock_[partitionIndex][k];
				input_matrix_t&       HmInverse = HmInverseTrajectoryStock_[partitionIndex][k];
				input_vector_t&       Gv = GvTrajectoryStock_[partitionIndex][k];
				input_state_matrix_t& Gm = GmTrajectoryStock_[partitionIndex][k];

				Hm = Rm + Bm.transpose() * BASE::SmTrajectoryStock_[partitionIndex][k+1] * Bm;
				if (settings_.ddpSettings_.useMakePSD_==true)
					BASE::makePSD(Hm);
				else
					Hm += settings_.ddpSettings_.addedRiccatiDiagonal_ * input_matrix_t::Identity();
				HmInverse = Hm.ldlt().solve(input_matrix_t::Identity());
				Gm = Pm + Bm.transpose() * BASE::SmTrajectoryStock_[partitionIndex][k+1] * Am;
				Gv = Rv + Bm.transpose() * BASE::SvTrajectoryStock_[partitionIndex][k+1];

				BASE::sTrajectoryStock_[partitionIndex][k] = q + BASE::sTrajectoryStock_[partitionIndex][k+1] -0.5*Gv.transpose()*HmInverse*Gv;
				BASE::SvTrajectoryStock_[partitionIndex][k] = Qv + Am.transpose()*BASE::SvTrajectoryStock_[partitionIndex][k+1] - Gm.transpose()*HmInverse*Gv;
				BASE::SmTrajectoryStock_[partitionIndex][k] = Qm + Am.transpose()*BASE::SmTrajectoryStock_[partitionIndex][k+1]*Am - Gm.transpose()*HmInverse*Gm;
			}
		}

		if (i > 0) {
			sFinalTemp  = BASE::sTrajectoryStock_[partitionIndex][beginTimeItr]  + BASE::qFinalStock_[partitionIndex][i-1];
			SvFinalTemp = BASE::SvTrajectoryStock_[partitionIndex][beginTimeItr] + BASE::QvFinalStock_[partitionIndex][i-1];
			SmFinalTemp = BASE::SmTrajectoryStock_[partitionIndex][beginTimeItr] + BASE::QmFinalStock_[partitionIndex][i-1];
		}

	}  // end of i loop


	// testing the numerical stability of the Riccati equations
	if (settings_.ddpSettings_.checkNumericalStability_)
		for (int k=N-1; k>=0; k--) {
			try {
				if (!BASE::SmTrajectoryStock_[partitionIndex][k].allFinite())  throw std::runtime_error("Sm is unstable.");
				if (BASE::SmTrajectoryStock_[partitionIndex][k].eigenvalues().real().minCoeff() < -Eigen::NumTraits<scalar_t>::epsilon())
					throw std::runtime_error("Sm matrix is not positive semi-definite. It's smallest eigenvalue is " +
							std::to_string(BASE::SmTrajectoryStock_[partitionIndex][k].eigenvalues().real().minCoeff()) + ".");
				if (!BASE::SvTrajectoryStock_[partitionIndex][k].allFinite())  throw std::runtime_error("Sv is unstable.");
				if (!BASE::sTrajectoryStock_[partitionIndex][k].allFinite())   throw std::runtime_error("s is unstable");
			}
			catch(const std::exception& error)
			{
				std::cerr << "what(): " << error.what() << " at time " << BASE::SsTimeTrajectoryStock_[partitionIndex][k] << " [sec]." << std::endl;
				for (int kp=k; kp<k+10; kp++)  {
					if (kp >= N) continue;
					std::cerr << "Sm[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\n"<< BASE::SmTrajectoryStock_[partitionIndex][kp].norm() << std::endl;
					std::cerr << "Sv[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\t"<< BASE::SvTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
					std::cerr << "s["  << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\t"<< BASE::sTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
				}
				exit(0);
			}
		}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
ILQR_Settings& ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::settings() {

	return settings_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setupOptimizer(const size_t& numPartitions) {

	BASE::setupOptimizer(numPartitions);

	HmTrajectoryStock_.resize(numPartitions);
	HmInverseTrajectoryStock_.resize(numPartitions);
	GmTrajectoryStock_.resize(numPartitions);
	GvTrajectoryStock_.resize(numPartitions);

	// Discrete-time coefficients
	AmDtimeTrajectoryStock_.resize(numPartitions);
	BmDtimeTrajectoryStock_.resize(numPartitions);

	qDtimeTrajectoryStock_.resize(numPartitions);
	QvDtimeTrajectoryStock_.resize(numPartitions);
	QmDtimeTrajectoryStock_.resize(numPartitions);
	RvDtimeTrajectoryStock_.resize(numPartitions);
	RmDtimeTrajectoryStock_.resize(numPartitions);
	PmDtimeTrajectoryStock_.resize(numPartitions);
	RmInverseDtimeTrajectoryStock_.resize(numPartitions);
}

}  // ocs2 namespace

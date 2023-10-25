cmake_minimum_required(VERSION 3.8)
project(ocs2_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ocs2_thirdparty REQUIRED)

find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# pthread and OpenMp
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)
if (Threads_FOUND) # Rename for catkin
  set(Threads_INCLUDE_DIRS ${THREADS_PTHREADS_INCLUDE_DIR})
  set(Threads_LIBRARIES ${CMAKE_THREAD_LIBS_INIT})
endif (Threads_FOUND)
find_package(OpenMP REQUIRED)

# Load ocs2 compile flags
include(cmake/ocs2_cxx_flags.cmake)
message(STATUS "OCS2_CXX_FLAGS: " ${OCS2_CXX_FLAGS})

include_directories(
  include
  test/include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  ocs2_thirdparty
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/Types.cpp
  src/augmented_lagrangian/AugmentedLagrangian.cpp
  src/augmented_lagrangian/StateAugmentedLagrangian.cpp
  src/augmented_lagrangian/StateInputAugmentedLagrangian.cpp
  src/augmented_lagrangian/StateAugmentedLagrangianCollection.cpp
  src/augmented_lagrangian/StateInputAugmentedLagrangianCollection.cpp
  src/automatic_differentation/CppAdInterface.cpp
  src/automatic_differentation/CppAdSparsity.cpp
  src/automatic_differentation/FiniteDifferenceMethods.cpp
  src/constraint/StateConstraintCppAd.cpp
  src/constraint/StateInputConstraintCppAd.cpp
  src/constraint/StateConstraintCollection.cpp
  src/constraint/StateInputConstraintCollection.cpp
  src/constraint/LinearStateConstraint.cpp
  src/constraint/LinearStateInputConstraint.cpp
  src/control/FeedforwardController.cpp
  src/control/LinearController.cpp
  src/control/StateBasedLinearController.cpp
  src/cost/QuadraticStateCost.cpp
  src/cost/QuadraticStateInputCost.cpp
  src/cost/StateCostCollection.cpp
  src/cost/StateCostCppAd.cpp
  src/cost/StateInputCostCollection.cpp
  src/cost/StateInputCostCppAd.cpp
  src/cost/StateInputGaussNewtonCostAd.cpp
  src/dynamics/ControlledSystemBase.cpp
  src/dynamics/LinearSystemDynamics.cpp
  src/dynamics/SystemDynamicsBase.cpp
  src/dynamics/SystemDynamicsBaseAD.cpp
  src/dynamics/SystemDynamicsLinearizer.cpp
  src/dynamics/TransferFunctionBase.cpp
  src/integration/SensitivityIntegrator.cpp
  src/integration/SensitivityIntegratorImpl.cpp
  src/integration/Integrator.cpp
  src/integration/IntegratorBase.cpp
  src/integration/RungeKuttaDormandPrince5.cpp
  src/integration/OdeBase.cpp
  src/integration/Observer.cpp
  src/integration/StateTriggeredEventHandler.cpp
  src/integration/SystemEventHandler.cpp
  src/reference/ModeSchedule.cpp
  src/reference/TargetTrajectories.cpp
  src/loopshaping/LoopshapingDefinition.cpp
  src/loopshaping/LoopshapingPropertyTree.cpp
  src/loopshaping/LoopshapingFilter.cpp
  src/loopshaping/LoopshapingPreComputation.cpp
  src/loopshaping/cost/LoopshapingCost.cpp
  src/loopshaping/cost/LoopshapingStateCost.cpp
  src/loopshaping/cost/LoopshapingStateInputCost.cpp
  src/loopshaping/cost/LoopshapingCostEliminatePattern.cpp
  src/loopshaping/cost/LoopshapingCostOutputPattern.cpp
  src/loopshaping/soft_constraint/LoopshapingSoftConstraint.cpp
  src/loopshaping/soft_constraint/LoopshapingStateInputSoftConstraint.cpp
  src/loopshaping/soft_constraint/LoopshapingSoftConstraintEliminatePattern.cpp
  src/loopshaping/soft_constraint/LoopshapingSoftConstraintOutputPattern.cpp
  src/loopshaping/augmented_lagrangian/LoopshapingAugmentedLagrangian.cpp
  src/loopshaping/augmented_lagrangian/LoopshapingStateAugmentedLagrangian.cpp
  src/loopshaping/augmented_lagrangian/LoopshapingStateInputAugmentedLagrangian.cpp
  src/loopshaping/augmented_lagrangian/LoopshapingAugmentedLagrangianEliminatePattern.cpp
  src/loopshaping/augmented_lagrangian/LoopshapingAugmentedLagrangianOutputPattern.cpp
  src/loopshaping/constraint/LoopshapingConstraint.cpp
  src/loopshaping/constraint/LoopshapingStateConstraint.cpp
  src/loopshaping/constraint/LoopshapingStateInputConstraint.cpp
  src/loopshaping/constraint/LoopshapingConstraintEliminatePattern.cpp
  src/loopshaping/constraint/LoopshapingConstraintOutputPattern.cpp
  src/loopshaping/dynamics/LoopshapingDynamics.cpp
  src/loopshaping/dynamics/LoopshapingDynamicsEliminatePattern.cpp
  src/loopshaping/dynamics/LoopshapingDynamicsOutputPattern.cpp
  src/loopshaping/dynamics/LoopshapingFilterDynamics.cpp
  src/loopshaping/initialization/LoopshapingInitializer.cpp
  src/model_data/ModelData.cpp
  src/model_data/Metrics.cpp
  src/model_data/Multiplier.cpp
  src/misc/LinearAlgebra.cpp
  src/misc/Log.cpp
  src/soft_constraint/StateSoftConstraint.cpp
  src/soft_constraint/StateInputSoftConstraint.cpp
  src/soft_constraint/StateInputSoftBoxConstraint.cpp
  src/penalties/MultidimensionalPenalty.cpp
  src/penalties/Penalties.cpp
  src/penalties/penalties/RelaxedBarrierPenalty.cpp
  src/penalties/penalties/SquaredHingePenalty.cpp
  src/thread_support/ThreadPool.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()

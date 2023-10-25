cmake_minimum_required(VERSION 3.8)
project(ocs2_oc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_thirdparty REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
  ocs2_core
  ocs2_thirdparty
)

add_library(${PROJECT_NAME}
  src/approximate_model/ChangeOfInputVariables.cpp
  src/approximate_model/LinearQuadraticApproximator.cpp
  src/multiple_shooting/Helpers.cpp
  src/multiple_shooting/Initialization.cpp
  src/multiple_shooting/LagrangianEvaluation.cpp
  src/multiple_shooting/MetricsComputation.cpp
  src/multiple_shooting/PerformanceIndexComputation.cpp
  src/multiple_shooting/ProjectionMultiplierCoefficients.cpp
  src/multiple_shooting/Transcription.cpp
  src/oc_data/LoopshapingPrimalSolution.cpp
  src/oc_data/PerformanceIndex.cpp
  src/oc_data/TimeDiscretization.cpp
  src/oc_problem/OptimalControlProblem.cpp
  src/oc_problem/LoopshapingOptimalControlProblem.cpp
  src/oc_problem/OptimalControlProblemHelperFunction.cpp
  src/oc_problem/OcpSize.cpp
  src/oc_problem/OcpToKkt.cpp
  src/oc_solver/SolverBase.cpp
  src/precondition/Ruzi.cpp
  src/rollout/PerformanceIndicesRollout.cpp
  src/rollout/RolloutBase.cpp
  src/rollout/RootFinder.cpp
  src/rollout/InitializerRollout.cpp
  src/rollout/StateTriggeredRollout.cpp
  src/rollout/TimeTriggeredRollout.cpp
  src/rollout/RolloutSettings.cpp
  src/synchronized_module/ReferenceManager.cpp
  src/synchronized_module/LoopshapingReferenceManager.cpp
  src/synchronized_module/LoopshapingSynchronizedModule.cpp
  src/synchronized_module/SolverObserver.cpp
  src/search_strategy/FilterLinesearch.cpp
  src/trajectory_adjustment/TrajectorySpreading.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
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

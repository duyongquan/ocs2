cmake_minimum_required(VERSION 3.8)
project(ocs2_ddp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_qp_solver REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_mpc REQUIRED)
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
  ocs2_qp_solver
  ocs2_oc
  ocs2_mpc
  ocs2_thirdparty
)

add_library(${PROJECT_NAME}
  src/riccati_equations/ContinuousTimeRiccatiEquations.cpp
  src/riccati_equations/DiscreteTimeRiccatiEquations.cpp
  src/riccati_equations/RiccatiModification.cpp
  src/search_strategy/LevenbergMarquardtStrategy.cpp
  src/search_strategy/LineSearchStrategy.cpp
  src/search_strategy/StrategySettings.cpp
  src/ContinuousTimeLqr.cpp
  src/GaussNewtonDDP.cpp
  src/HessianCorrection.cpp
  src/ILQR.cpp
  src/SLQ.cpp
  src/DDP_Settings.cpp
  src/DDP_HelperFunctions.cpp
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

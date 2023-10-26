cmake_minimum_required(VERSION 3.8)
project(ocs2_double_integrator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem)

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
  ocs2_ddp
  ocs2_mpc
  ocs2_robotic_tools
)

# Resolve for the package path at compile time.
configure_file (
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY
)

# cartpole interface library
add_library(${PROJECT_NAME}
  src/DoubleIntegratorInterface.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  # add_subdirectory(test)
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY auto_generated DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()

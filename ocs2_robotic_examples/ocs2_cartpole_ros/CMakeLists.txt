cmake_minimum_required(VERSION 3.8)
project(ocs2_cartpole_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED) 
find_package(ament_index_cpp REQUIRED) 
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_robotic_tools REQUIRED) 
find_package(ocs2_cartpole REQUIRED)
find_package(ocs2_robotic_assets REQUIRED) 

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
  ament_index_cpp
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_ros_interfaces
  ocs2_robotic_tools
  ocs2_cartpole
  ocs2_robotic_assets
)

# Mpc node
add_executable(cartpole_mpc src/CartpoleMpcNode.cpp)
ament_target_dependencies(cartpole_mpc ${dependencies})
target_link_libraries(cartpole_mpc ${Boost_LIBRARIES})

# Dummy node
add_executable(cartpole_dummy_test
  src/DummyCartpoleNode.cpp
  src/CartpoleDummyVisualization.cpp
)
ament_target_dependencies(cartpole_dummy_test ${dependencies})
target_link_libraries(cartpole_dummy_test ${Boost_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS cartpole_mpc cartpole_dummy_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install(TARGETS cartpole_mpc cartpole_dummy_test
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()

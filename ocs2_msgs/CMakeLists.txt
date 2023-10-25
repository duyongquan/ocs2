cmake_minimum_required(VERSION 3.5)
project(ocs2_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MpcState.msg"
  "msg/MpcInput.msg"
  "msg/ModeSchedule.msg"
  "msg/MpcObservation.msg"
  "msg/MpcPerformanceIndices.msg"
  "msg/MpcTargetTrajectories.msg"
  "msg/ControllerData.msg"
  "msg/MpcFlattenedController.msg"
  "msg/LagrangianMetrics.msg"
  "msg/Multiplier.msg"
  "msg/Constraint.msg"
  "srv/Reset.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

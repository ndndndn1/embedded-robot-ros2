cmake_minimum_required(VERSION 3.16)
project(embedded_robot_interfaces VERSION 1.0.0 LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GraspCandidate.msg"
  "msg/GraspCandidateArray.msg"
  "msg/CalibrationObservation.msg"
  "msg/PerceptionFrameInfo.msg"
  "action/CalibrateExtrinsics.action"
  "action/ValidateGrasp.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

cmake_minimum_required(VERSION 3.8)
project(venom_overtake_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/TrackedObstacle.msg"
  "msg/TrackedObstacleArray.msg"
  "msg/LeadVehicle.msg"
  "msg/OvertakeDecision.msg"
  "msg/LaneCorridor.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

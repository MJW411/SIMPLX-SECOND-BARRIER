cmake_minimum_required(VERSION 3.10)
project(omx_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Person.msg"
  "msg/People.msg"
  "msg/Permit.msg"
  "msg/RobotState.msg"
  "msg/SafetyState.msg"
  "msg/IoState.msg"
  "msg/HelmetState.msg"
  "msg/Event.msg"
  "msg/ClipStatus.msg"
  "srv/Command.srv"
)
ament_package()

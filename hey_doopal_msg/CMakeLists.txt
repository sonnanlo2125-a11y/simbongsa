cmake_minimum_required(VERSION 3.8)
project(hey_doopal_msg)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetPoint.msg"

  "srv/VoiceKeyword.srv"
  "srv/ScanRequest.srv"
  "srv/GripBoundingBox.srv"
  "srv/GetDbData.srv"

  "action/FindTargetOrder.action"
  "action/FindOrder.action"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

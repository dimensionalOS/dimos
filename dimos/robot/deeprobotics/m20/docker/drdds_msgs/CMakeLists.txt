cmake_minimum_required(VERSION 3.8)
project(drdds)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Timestamp.msg"
  "msg/MetaType.msg"
  "msg/NavCmdValue.msg"
  "msg/NavCmd.msg"
  "msg/MotionStateValue.msg"
  "msg/GaitValue.msg"
  "msg/MotionInfoValue.msg"
  "msg/MotionInfo.msg"
  "msg/MotionStatusValue.msg"
  "msg/MotionStatus.msg"
)

ament_package()

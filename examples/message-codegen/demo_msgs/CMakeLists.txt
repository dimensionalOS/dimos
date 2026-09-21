cmake_minimum_required(VERSION 3.20)
project(demo_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Reading.msg"
  "msg/Telemetry.msg"
  "msg/ImageEnvelope.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()

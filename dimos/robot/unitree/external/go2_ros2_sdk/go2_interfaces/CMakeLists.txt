cmake_minimum_required(VERSION 3.8)
project(go2_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Go2Cmd.msg"
  "msg/Go2State.msg"
  "msg/Go2Move.msg"
  "msg/Go2RpyCmd.msg"
  "msg/IMU.msg"

  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)


ament_package()

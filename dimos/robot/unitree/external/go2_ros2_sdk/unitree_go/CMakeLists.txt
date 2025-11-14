cmake_minimum_required(VERSION 3.8)
project(unitree_go)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.


rosidl_generate_interfaces(${PROJECT_NAME}
"msg/AudioData.msg"
"msg/BmsCmd.msg"
"msg/BmsState.msg"
"msg/Error.msg"
"msg/Go2FrontVideoData.msg"
"msg/HeightMap.msg"
"msg/IMUState.msg"
"msg/InterfaceConfig.msg"
"msg/LidarState.msg"
"msg/LowCmd.msg"
"msg/LowState.msg"
"msg/MotorCmd.msg"
"msg/MotorCmds.msg"
"msg/MotorState.msg"
"msg/MotorStates.msg"
"msg/PathPoint.msg"
"msg/Req.msg"
"msg/Res.msg"
"msg/SportModeCmd.msg"
"msg/SportModeState.msg"
"msg/TimeSpec.msg"
"msg/UwbState.msg"
"msg/UwbSwitch.msg"
"msg/VoxelMapCompressed.msg"
"msg/VoxelHeightMapState.msg"
"msg/WebRtcReq.msg"
"msg/WirelessController.msg"

)


ament_package()

cmake_minimum_required(VERSION 3.8)
project(drdds_bridge)

set(CMAKE_CXX_STANDARD 17)

# ---- Target 1: drdds_recv (drdds side, FastDDS 2.14) ----
# Built separately because it links against host's drdds libs (FastDDS 2.14)
# which conflict with ROS2 Humble's FastDDS 2.6.
#
# This target is built with a standalone cmake invocation, NOT through colcon,
# because colcon would inject ROS2's FastDDS into the link path.
#
# Build command (standalone on NOS host):
#   mkdir -p build_drdds && cd build_drdds
#   cmake .. -DBUILD_DRDDS_RECV=ON -DBUILD_ROS2_PUB=OFF
#   make -j2

option(BUILD_DRDDS_RECV "Build the drdds receiver" OFF)
option(BUILD_ROS2_PUB "Build the ROS2 publisher" OFF)

if(BUILD_DRDDS_RECV)
    find_package(drdds REQUIRED)

    add_executable(drdds_recv src/drdds_recv.cpp)
    target_include_directories(drdds_recv PRIVATE
        include
        ${drdds_INCLUDE_DIRS}
    )
    target_link_libraries(drdds_recv ${drdds_LIBRARIES} -lrt)

    install(TARGETS drdds_recv DESTINATION lib/${PROJECT_NAME})
endif()

# ---- Target 2: ros2_pub (ROS2 side, FastDDS 2.6) ----
# Built through colcon as a standard ROS2 package.

if(BUILD_ROS2_PUB)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(sensor_msgs REQUIRED)

    add_executable(ros2_pub src/ros2_pub.cpp)
    target_include_directories(ros2_pub PRIVATE include)
    ament_target_dependencies(ros2_pub rclcpp sensor_msgs)
    target_link_libraries(ros2_pub -lrt)

    install(TARGETS ros2_pub DESTINATION lib/${PROJECT_NAME})

    # Install launch file
    install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

    ament_package()
endif()

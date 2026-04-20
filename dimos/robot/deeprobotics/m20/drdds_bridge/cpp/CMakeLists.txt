cmake_minimum_required(VERSION 3.14)
project(drdds_lidar_bridge CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/result" CACHE PATH "" FORCE)
endif()

# Shared helpers (dimos_native_module.hpp) are vendored into ./include/ so the
# nix build doesn't need to reach the rest of the dimos tree. Keep the vendor
# copy in sync with dimos/navigation/smart_nav/common/dimos_native_module.hpp
# whenever that canonical header changes.

# Fetch dimos-lcm for C++ message headers
include(FetchContent)
FetchContent_Declare(dimos_lcm
  GIT_REPOSITORY https://github.com/dimensionalOS/dimos-lcm.git
  GIT_TAG main
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(dimos_lcm)

# Find LCM
find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)

add_executable(drdds_lidar_bridge main.cpp)

# Headers: shm_transport.h and dimos_native_module.hpp in local include/
target_include_directories(drdds_lidar_bridge PRIVATE
  ${dimos_lcm_SOURCE_DIR}/generated/cpp_lcm_msgs
  ${LCM_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(drdds_lidar_bridge PRIVATE
  ${LCM_LIBRARIES}
  rt      # POSIX shared memory (shm_open, shm_unlink)
  pthread # semaphores, threads
)

target_link_directories(drdds_lidar_bridge PRIVATE
  ${LCM_LIBRARY_DIRS}
)

# nav_cmd_pub: raw FastDDS publisher for /NAV_CMD (no ROS, no drdds wrapper)
# Subscribes to cmd_vel on LCM, publishes to rt/NAV_CMD via FastDDS.
# Skipped in the nix build because FastDDS + libdrdds live at /usr/local on
# NOS and aren't reachable from the nix sandbox. Built via the local-cmake
# path (build/) instead — see deploy.sh.
option(BUILD_NAV_CMD_PUB "Build the nav_cmd_pub FastDDS bridge (requires /usr/local/{include,lib})" ON)
if(BUILD_NAV_CMD_PUB)
add_executable(nav_cmd_pub nav_cmd_pub.cpp)

target_include_directories(nav_cmd_pub PRIVATE
  ${dimos_lcm_SOURCE_DIR}/generated/cpp_lcm_msgs
  ${LCM_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  /usr/local/include          # drdds IDL headers
  /usr/local/include/dridl    # drdds message types
)

# Find all IDL msg subdirectories for includes
file(GLOB DRIDL_MSG_DIRS "/usr/local/include/dridl/*/msg")
target_include_directories(nav_cmd_pub PRIVATE ${DRIDL_MSG_DIRS})

target_link_libraries(nav_cmd_pub PRIVATE
  ${LCM_LIBRARIES}
  fastrtps    # FastDDS 2.14
  fastcdr     # CDR serialization
  drdds       # NavCmd message implementations
  pthread
)

target_link_directories(nav_cmd_pub PRIVATE
  ${LCM_LIBRARY_DIRS}
  /usr/local/lib
)
endif()  # BUILD_NAV_CMD_PUB

# drdds_recv: FastDDS SHM subscriber for rsdriver/yesense → POSIX SHM. Same
# BUILD_NAV_CMD_PUB reasoning — depends on host /usr/local FastDDS which isn't
# in the nix sandbox. Built via the cpp/build/ path, installed to
# /opt/drdds_bridge/lib/drdds_bridge/ by deploy.sh.
if(BUILD_NAV_CMD_PUB)  # share the host-libs gate
add_executable(drdds_recv drdds_recv.cpp)

target_include_directories(drdds_recv PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  /usr/local/include
  /usr/local/include/dridl
)
file(GLOB DRDDS_RECV_MSG_DIRS "/usr/local/include/dridl/*/msg")
target_include_directories(drdds_recv PRIVATE ${DRDDS_RECV_MSG_DIRS})

target_link_libraries(drdds_recv PRIVATE
  fastrtps
  fastcdr
  drdds
  rt
  pthread
)

target_link_directories(drdds_recv PRIVATE
  /usr/local/lib
)
endif()  # BUILD_NAV_CMD_PUB (drdds_recv)

# airy_imu_bridge: taps the RoboSense Airy integrated IMU directly from its
# UDP multicast stream (rsdriver doesn't republish it to drdds despite the
# config key). Parses the 51B rs_driver packet format and publishes LCM
# sensor_msgs::Imu in base_link frame.
add_executable(airy_imu_bridge airy_imu_bridge.cpp)

target_include_directories(airy_imu_bridge PRIVATE
  ${dimos_lcm_SOURCE_DIR}/generated/cpp_lcm_msgs
  ${LCM_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(airy_imu_bridge PRIVATE
  ${LCM_LIBRARIES}
  pthread
)

target_link_directories(airy_imu_bridge PRIVATE
  ${LCM_LIBRARY_DIRS}
)

set(INSTALL_TARGETS drdds_lidar_bridge airy_imu_bridge)
if(BUILD_NAV_CMD_PUB)
  list(APPEND INSTALL_TARGETS nav_cmd_pub drdds_recv)
endif()
install(TARGETS ${INSTALL_TARGETS} DESTINATION bin)

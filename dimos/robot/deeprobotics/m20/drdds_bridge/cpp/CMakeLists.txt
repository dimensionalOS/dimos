cmake_minimum_required(VERSION 3.14)
project(drdds_lidar_bridge CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/result" CACHE PATH "" FORCE)
endif()

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
# Subscribes to cmd_vel on LCM, publishes to rt/NAV_CMD via FastDDS
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

install(TARGETS drdds_lidar_bridge nav_cmd_pub DESTINATION bin)

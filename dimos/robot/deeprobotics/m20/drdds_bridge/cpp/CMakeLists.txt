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

# SHM transport header (shared with drdds_recv)
set(DRDDS_BRIDGE_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../../../docker/drdds_bridge/include)

# NativeModule common headers
set(NATIVE_COMMON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../hardware/sensors/lidar/common)

add_executable(drdds_lidar_bridge main.cpp)

target_include_directories(drdds_lidar_bridge PRIVATE
  ${dimos_lcm_SOURCE_DIR}/generated/cpp_lcm_msgs
  ${LCM_INCLUDE_DIRS}
  ${DRDDS_BRIDGE_INCLUDE}
  ${NATIVE_COMMON_DIR}
)

target_link_libraries(drdds_lidar_bridge PRIVATE
  ${LCM_LIBRARIES}
  rt      # POSIX shared memory (shm_open, shm_unlink)
  pthread # semaphores, threads
)

target_link_directories(drdds_lidar_bridge PRIVATE
  ${LCM_LIBRARY_DIRS}
)

install(TARGETS drdds_lidar_bridge DESTINATION bin)

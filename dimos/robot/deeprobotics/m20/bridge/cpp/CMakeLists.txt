# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.14)
project(m20_ros_bridge CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(drdds REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED IMPORTED_TARGET lcm)

if(DEFINED DIMOS_LCM_DIR)
  set(dimos_lcm_SOURCE_DIR ${DIMOS_LCM_DIR})
else()
  include(FetchContent)
  FetchContent_Declare(dimos_lcm
    GIT_REPOSITORY https://github.com/dimensionalOS/dimos-lcm.git
    GIT_TAG 0a1c24710ce2f7a569e1673617102cc5254a75e2
  )
  # dimos-lcm is a generated-header repository without a CMakeLists.txt, so it
  # can be populated but not added as a CMake subproject.
  FetchContent_GetProperties(dimos_lcm)
  if(NOT dimos_lcm_POPULATED)
    FetchContent_Populate(dimos_lcm)
  endif()
endif()

if(NOT DEFINED DIMOS_NATIVE_CPP_DIR)
  set(DIMOS_NATIVE_CPP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../native/cpp)
endif()
add_subdirectory(${DIMOS_NATIVE_CPP_DIR} ${CMAKE_BINARY_DIR}/dimos_native)

add_executable(m20_ros_bridge main.cpp)
target_include_directories(m20_ros_bridge PRIVATE
  ${dimos_lcm_SOURCE_DIR}/generated/cpp_lcm_msgs
)
ament_target_dependencies(m20_ros_bridge
  drdds
  rclcpp
)
target_link_libraries(m20_ros_bridge
  dimos_native
  PkgConfig::LCM
)
target_compile_options(m20_ros_bridge PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS m20_ros_bridge DESTINATION bin)

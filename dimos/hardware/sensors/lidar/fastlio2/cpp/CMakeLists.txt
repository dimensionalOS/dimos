cmake_minimum_required(VERSION 3.14)
project(fastlio2_native CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/result" CACHE PATH "" FORCE)
endif()

# OpenMP for parallel processing
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# MP defines (same logic as FAST-LIO)
message("CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  if(N GREATER 4)
    add_definitions(-DMP_EN -DMP_PROC_NUM=3)
  elseif(N GREATER 3)
    add_definitions(-DMP_EN -DMP_PROC_NUM=2)
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

# Fetch dependencies
include(FetchContent)

# FAST-LIO-NON-ROS (pass -DFASTLIO_DIR=<path> or auto-fetched from GitHub)
if(NOT FASTLIO_DIR)
  message(STATUS "FASTLIO_DIR not set, fetching FAST-LIO-NON-ROS from GitHub...")
  FetchContent_Declare(fast_lio
    GIT_REPOSITORY https://github.com/leshy/FAST-LIO-NON-ROS.git
    GIT_TAG dimos-integration
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(fast_lio)
  set(FASTLIO_DIR ${fast_lio_SOURCE_DIR})
endif()

# dimos-lcm C++ message headers
FetchContent_Declare(dimos_lcm
  GIT_REPOSITORY https://github.com/dimensionalOS/dimos-lcm.git
  GIT_TAG main
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(dimos_lcm)

# LCM
find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)

# Eigen3
find_package(Eigen3 REQUIRED)

# PCL (only components we need — avoid full PCL which drags in VTK via io)
find_package(PCL 1.8 REQUIRED COMPONENTS common filters)

# yaml-cpp (FAST-LIO config parsing — standard YAML format)
find_package(yaml-cpp REQUIRED)

# Livox SDK2 (from nix or /usr/local fallback)
find_library(LIVOX_SDK livox_lidar_sdk_shared)
if(NOT LIVOX_SDK)
  message(FATAL_ERROR "Livox SDK2 not found. Available via nix flake in lidar/livox/")
endif()
get_filename_component(LIVOX_SDK_LIB_DIR ${LIVOX_SDK} DIRECTORY)
get_filename_component(LIVOX_SDK_PREFIX ${LIVOX_SDK_LIB_DIR} DIRECTORY)
set(LIVOX_SDK_INCLUDE_DIR ${LIVOX_SDK_PREFIX}/include)

add_executable(fastlio2_native
  main.cpp
  ${FASTLIO_DIR}/src/preprocess.cpp
  ${FASTLIO_DIR}/include/ikd-Tree/ikd_Tree.cpp
)

# Shared Livox common headers (livox_sdk_config.hpp etc.)
if(NOT LIVOX_COMMON_DIR)
  set(LIVOX_COMMON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../common)
endif()

target_include_directories(fastlio2_native PRIVATE
  ${FASTLIO_DIR}/include
  ${FASTLIO_DIR}/src
  ${dimos_lcm_SOURCE_DIR}/generated/cpp_lcm_msgs
  ${LCM_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${LIVOX_COMMON_DIR}
  ${LIVOX_SDK_INCLUDE_DIR}
)

target_link_libraries(fastlio2_native PRIVATE
  ${LCM_LIBRARIES}
  ${LIVOX_SDK}
  ${PCL_LIBRARIES}
  yaml-cpp::yaml-cpp
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(fastlio2_native PRIVATE OpenMP::OpenMP_CXX)
endif()

target_link_directories(fastlio2_native PRIVATE
  ${LCM_LIBRARY_DIRS}
)

install(TARGETS fastlio2_native DESTINATION bin)

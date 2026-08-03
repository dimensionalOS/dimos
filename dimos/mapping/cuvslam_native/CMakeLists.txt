# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.14)
project(dimos_cuvslam_native CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# Generated LCM message headers (sensor_msgs::Image, nav_msgs::Odometry, ...).
# Under nix, -DFETCHCONTENT_SOURCE_DIR_DIMOS_LCM points this at the flake input,
# so the sandboxed build needs no network.
FetchContent_Declare(dimos_lcm
  GIT_REPOSITORY https://github.com/dimensionalOS/dimos-lcm.git
  GIT_TAG main
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(dimos_lcm)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)

# Header-only dimos native C++ SDK. Overridable so nix can point at the flake
# input instead of the in-repo relative path.
if(NOT DEFINED DIMOS_NATIVE_CPP_DIR)
  set(DIMOS_NATIVE_CPP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../native/cpp)
endif()
add_subdirectory(${DIMOS_NATIVE_CPP_DIR} ${CMAKE_BINARY_DIR}/dimos_native)

# NVIDIA's prebuilt cuVSLAM C++ SDK (headers + libcuvslam.so). The nix flake
# packages the official release archive and passes its store path here. There is
# deliberately no fallback to a hand-written header: guessing at NVIDIA's ABI
# would compile and then fail at runtime in ways that are very hard to diagnose.
if(NOT DEFINED CUVSLAM_SDK_DIR)
  message(FATAL_ERROR
    "CUVSLAM_SDK_DIR is not set. Build via `nix build` (which packages NVIDIA's "
    "official C++ SDK), or pass -DCUVSLAM_SDK_DIR=/path/to/extracted/sdk.")
endif()
find_path(CUVSLAM_INCLUDE_DIR cuvslam/cuvslam2.h HINTS ${CUVSLAM_SDK_DIR}/include REQUIRED)
find_library(CUVSLAM_LIBRARY cuvslam HINTS ${CUVSLAM_SDK_DIR}/lib REQUIRED)
message(STATUS "cuVSLAM headers: ${CUVSLAM_INCLUDE_DIR}")
message(STATUS "cuVSLAM library: ${CUVSLAM_LIBRARY}")

add_executable(cuvslam_odometry src/cuvslam_odometry.cpp)
target_include_directories(cuvslam_odometry PRIVATE
  ${dimos_lcm_SOURCE_DIR}/generated/cpp_lcm_msgs
  ${LCM_INCLUDE_DIRS}
  ${CUVSLAM_INCLUDE_DIR}
)
target_link_libraries(cuvslam_odometry PRIVATE
  dimos_native
  ${LCM_LIBRARIES}
  ${CUVSLAM_LIBRARY}
)
target_link_directories(cuvslam_odometry PRIVATE ${LCM_LIBRARY_DIRS})
install(TARGETS cuvslam_odometry RUNTIME DESTINATION bin)

# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.14)
project(m20_pointlio CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP QUIET)
find_package(PCL 1.8 REQUIRED COMPONENTS common filters)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
pkg_check_modules(LCM REQUIRED IMPORTED_TARGET lcm)

include(FetchContent)

if(DEFINED DIMOS_LCM_DIR)
  set(dimos_lcm_SOURCE_DIR ${DIMOS_LCM_DIR})
else()
  FetchContent_Declare(dimos_lcm
    GIT_REPOSITORY https://github.com/dimensionalOS/dimos-lcm.git
    GIT_TAG 0a1c24710ce2f7a569e1673617102cc5254a75e2
  )
  FetchContent_GetProperties(dimos_lcm)
  if(NOT dimos_lcm_POPULATED)
    FetchContent_Populate(dimos_lcm)
  endif()
endif()

if(NOT DEFINED POINTLIO_DIR)
  FetchContent_Declare(pointlio
    GIT_REPOSITORY https://github.com/dimensionalOS/dimos-module-pointlio.git
    GIT_TAG 82ef3a327347e2866e981bd95c8bece8b72903cf
  )
  FetchContent_GetProperties(pointlio)
  if(NOT pointlio_POPULATED)
    FetchContent_Populate(pointlio)
  endif()
  set(POINTLIO_DIR ${pointlio_SOURCE_DIR})
endif()

set(POINTLIO_SOURCE_DIR ${POINTLIO_DIR})
file(READ "${POINTLIO_SOURCE_DIR}/src/laserMapping.hpp" pointlio_mapping_source)
file(READ "${POINTLIO_SOURCE_DIR}/include/ivox/ivox3d.h" pointlio_ivox_source)
string(FIND "${pointlio_mapping_source}" "crossmat_list.reserve" pointlio_has_reserve)
string(FIND "${pointlio_mapping_source}" "crossmat_list.resize" pointlio_has_resize)
string(FIND "${pointlio_ivox_source}" "#include <glog/logging.h>" pointlio_has_glog)

if(NOT pointlio_has_reserve EQUAL -1 AND NOT pointlio_has_glog EQUAL -1)
  set(POINTLIO_PATCHED_DIR "${CMAKE_BINARY_DIR}/pointlio-patched")
  file(REMOVE_RECURSE "${POINTLIO_PATCHED_DIR}")
  file(MAKE_DIRECTORY "${POINTLIO_PATCHED_DIR}")
  file(COPY "${POINTLIO_SOURCE_DIR}/" DESTINATION "${POINTLIO_PATCHED_DIR}")
  find_package(Git REQUIRED)
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" apply "${CMAKE_CURRENT_SOURCE_DIR}/pointlio-gos.patch"
    WORKING_DIRECTORY "${POINTLIO_PATCHED_DIR}"
    RESULT_VARIABLE pointlio_patch_result
    ERROR_VARIABLE pointlio_patch_error
  )
  if(NOT pointlio_patch_result EQUAL 0)
    message(FATAL_ERROR "Could not patch the pinned Point-LIO source: ${pointlio_patch_error}")
  endif()
  set(POINTLIO_DIR "${POINTLIO_PATCHED_DIR}")
elseif(NOT pointlio_has_resize EQUAL -1 AND pointlio_has_glog EQUAL -1)
  message(STATUS "Using an already-patched Point-LIO source: ${POINTLIO_SOURCE_DIR}")
else()
  message(FATAL_ERROR "Point-LIO source is neither the pinned original nor the expected patched tree")
endif()

if(NOT DEFINED DIMOS_NATIVE_CPP_DIR)
  set(DIMOS_NATIVE_CPP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../native/cpp)
endif()
add_subdirectory(${DIMOS_NATIVE_CPP_DIR} ${CMAKE_BINARY_DIR}/dimos_native)

add_executable(m20_pointlio
  main.cpp
  ${POINTLIO_DIR}/src/preprocess.cpp
  ${POINTLIO_DIR}/src/Estimator.cpp
  ${POINTLIO_DIR}/src/parameters.cpp
)
target_include_directories(m20_pointlio PRIVATE
  ${POINTLIO_DIR}/include
  ${POINTLIO_DIR}/include/IKFoM/IKFoM_toolkit
  ${POINTLIO_DIR}/src
  ${dimos_lcm_SOURCE_DIR}/generated/cpp_lcm_msgs
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../../../hardware/sensors/lidar/common
)
target_compile_definitions(m20_pointlio PRIVATE
  MP_PROC_NUM=1
  ROOT_DIR="/tmp/m20_pointlio_"
)
target_compile_options(m20_pointlio PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(m20_pointlio
  dimos_native
  Eigen3::Eigen
  PkgConfig::LCM
  ${PCL_LIBRARIES}
)
ament_target_dependencies(m20_pointlio
  rclcpp
  sensor_msgs
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(m20_pointlio OpenMP::OpenMP_CXX)
endif()

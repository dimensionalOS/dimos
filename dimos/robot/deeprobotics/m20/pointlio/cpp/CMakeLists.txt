# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.14)
project(m20_pointlio CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)
find_package(OpenMP QUIET)
find_package(PCL 1.8 REQUIRED COMPONENTS common filters)
find_package(PkgConfig REQUIRED)
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
  ${CMAKE_CURRENT_SOURCE_DIR}/compat
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
target_link_libraries(m20_pointlio PRIVATE
  dimos_native
  Eigen3::Eigen
  PkgConfig::LCM
  ${PCL_LIBRARIES}
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(m20_pointlio PRIVATE OpenMP::OpenMP_CXX)
endif()

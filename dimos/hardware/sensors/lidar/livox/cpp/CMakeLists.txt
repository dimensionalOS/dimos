cmake_minimum_required(VERSION 3.14)
project(livox_mid360_native CXX)

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

# Livox SDK2 (from nix or /usr/local fallback)
find_library(LIVOX_SDK livox_lidar_sdk_shared)
if(NOT LIVOX_SDK)
  message(FATAL_ERROR "Livox SDK2 not found. Available via nix flake in lidar/livox/")
endif()
get_filename_component(LIVOX_SDK_LIB_DIR ${LIVOX_SDK} DIRECTORY)
get_filename_component(LIVOX_SDK_PREFIX ${LIVOX_SDK_LIB_DIR} DIRECTORY)
set(LIVOX_SDK_INCLUDE_DIR ${LIVOX_SDK_PREFIX}/include)

add_executable(mid360_native main.cpp)

# Shared Livox common headers (livox_sdk_config.hpp etc.)
if(NOT LIVOX_COMMON_DIR)
  set(LIVOX_COMMON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../common)
endif()

target_include_directories(mid360_native PRIVATE
  ${dimos_lcm_SOURCE_DIR}/generated/cpp_lcm_msgs
  ${LCM_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${LIVOX_COMMON_DIR}
  ${LIVOX_SDK_INCLUDE_DIR}
)

target_link_libraries(mid360_native PRIVATE
  ${LCM_LIBRARIES}
  ${LIVOX_SDK}
)

target_link_directories(mid360_native PRIVATE
  ${LCM_LIBRARY_DIRS}
)

install(TARGETS mid360_native DESTINATION bin)

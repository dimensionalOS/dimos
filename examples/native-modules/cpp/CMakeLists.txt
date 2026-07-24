# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.14)
project(dimos_native_ping_pong CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# Generated LCM message headers (geometry_msgs::Twist, ...). Under nix,
# -DFETCHCONTENT_SOURCE_DIR_DIMOS_LCM points this at the flake input, so the
# sandboxed build needs no network.
FetchContent_Declare(dimos_lcm
  GIT_REPOSITORY https://github.com/dimensionalOS/dimos-lcm.git
  GIT_TAG main
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(dimos_lcm)

# JSON parser used by the SDK's stdin config. Prefer a system/nix package
# (nix provides it via buildInputs), else fetch it for a plain build.
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
  FetchContent_Declare(nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
  )
  FetchContent_MakeAvailable(nlohmann_json)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)
find_package(Threads REQUIRED)

# The header-only dimos native C++ SDK. Overridable so nix can point at the
# flake input instead of the in-repo relative path.
if(NOT DEFINED DIMOS_NATIVE_CPP_DIR)
  set(DIMOS_NATIVE_CPP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../native/cpp)
endif()
add_subdirectory(${DIMOS_NATIVE_CPP_DIR} ${CMAKE_BINARY_DIR}/dimos_native)

foreach(mod ping pong)
  add_executable(${mod} src/${mod}.cpp)
  target_include_directories(${mod} PRIVATE
    ${dimos_lcm_SOURCE_DIR}/generated/cpp_lcm_msgs
    ${LCM_INCLUDE_DIRS}
  )
  target_link_libraries(${mod} PRIVATE
    dimos_native
    nlohmann_json::nlohmann_json
    Threads::Threads
    ${LCM_LIBRARIES}
  )
  target_link_directories(${mod} PRIVATE ${LCM_LIBRARY_DIRS})
  install(TARGETS ${mod} RUNTIME DESTINATION bin)
endforeach()

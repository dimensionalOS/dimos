# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.14)
project(dimos_native_ping_pong CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# The header-only dimos native C++ SDK. Overridable so nix can point at the
# flake input instead of the in-repo relative path.
if(NOT DEFINED DIMOS_NATIVE_CPP_DIR)
  set(DIMOS_NATIVE_CPP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../native/cpp)
endif()
add_subdirectory(${DIMOS_NATIVE_CPP_DIR} ${CMAKE_BINARY_DIR}/dimos_native)

foreach(mod ping pong cdr_relay)
  add_executable(${mod} src/${mod}.cpp)
  target_link_libraries(${mod} PRIVATE dimos_native)
  install(TARGETS ${mod} RUNTIME DESTINATION bin)
endforeach()

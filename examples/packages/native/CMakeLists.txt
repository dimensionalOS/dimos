# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.15)
project(dimos_external_native LANGUAGES CXX)
if(NOT UNIX)
  message(FATAL_ERROR "This NativeModule example requires POSIX signals")
endif()
add_executable(package_probe native/probe.cpp)
target_compile_features(package_probe PRIVATE cxx_std_17)
install(TARGETS package_probe RUNTIME DESTINATION dimos_external_native/bin)

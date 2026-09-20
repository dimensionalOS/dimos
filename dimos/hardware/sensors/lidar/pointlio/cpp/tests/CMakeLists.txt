# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0
#
# Standalone on purpose: the module itself needs PCL, Eigen, glog and the Livox
# SDK to configure, and the publish-stamp rules need none of them. Run with
#
#   cmake -S dimos/hardware/sensors/lidar/pointlio/cpp/tests -B build/pointlio-stamp
#   cmake --build build/pointlio-stamp
#   ctest --test-dir build/pointlio-stamp --output-on-failure
cmake_minimum_required(VERSION 3.14)
project(pointlio_publish_stamp_tests CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()
add_executable(test_publish_stamp test_publish_stamp.cpp)
target_include_directories(test_publish_stamp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_compile_options(test_publish_stamp PRIVATE -Wall -Wextra -Werror)
add_test(NAME publish_stamp COMMAND test_publish_stamp)

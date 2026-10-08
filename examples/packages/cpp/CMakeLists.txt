# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.15)
project(dimos_package_cpp LANGUAGES CXX)
include(FetchContent)
# Fixed SDK and generated schemas; ordinary wheels never fetch these at runtime.
FetchContent_Declare(dimos_sdk
  GIT_REPOSITORY https://github.com/dimensionalOS/dimos.git
  GIT_TAG dbb630f8e298218e9862e03dc24d6ba47080d20c
  # This SDK-only checkout needs source files, never monorepo LFS assets.
  GIT_CONFIG filter.lfs.process= filter.lfs.smudge= filter.lfs.required=false
  SOURCE_SUBDIR native/cpp
)
FetchContent_Declare(dimos_messages
  GIT_REPOSITORY https://github.com/dimensionalOS/dimos-lcm.git
  GIT_TAG 04d78e8622500244123ba9cefa4c51b4cb454549
)
FetchContent_MakeAvailable(dimos_sdk dimos_messages)
add_executable(package_pong native/main.cpp)
target_include_directories(package_pong PRIVATE "${dimos_messages_SOURCE_DIR}/generated/cpp_lcm_msgs")
target_link_libraries(package_pong PRIVATE dimos_native)
install(TARGETS package_pong RUNTIME DESTINATION dimos_package_cpp/bin COMPONENT python)

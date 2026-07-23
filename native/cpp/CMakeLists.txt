# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.14)
project(dimos_native CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# PFR (standalone Boost.PFR): compile-time struct reflection for config
# parsing. Header-only; nix builds inject it via FETCHCONTENT_SOURCE_DIR_PFR.
include(FetchContent)
FetchContent_Declare(pfr
  GIT_REPOSITORY https://github.com/apolukhin/pfr_non_boost.git
  GIT_TAG 2.3.2
)
FetchContent_MakeAvailable(pfr)

# Header-only SDK. Consumers link this INTERFACE target for the include path.
# cxx_std_20 propagates to consumers: the SDK target is the single source of
# truth for the minimum C++ standard.
add_library(dimos_native INTERFACE)
target_include_directories(dimos_native INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(dimos_native INTERFACE cxx_std_20)
target_link_libraries(dimos_native INTERFACE pfr)

option(DIMOS_NATIVE_BUILD_TESTS "Build the dimos native SDK tests" OFF)
if(DIMOS_NATIVE_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

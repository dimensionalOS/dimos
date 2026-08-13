# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.14)
project(dimos_native CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# PFR (standalone Boost.PFR): compile-time struct reflection for config
# parsing. Header-only. nix builds inject it via FETCHCONTENT_SOURCE_DIR_PFR.
include(FetchContent)
FetchContent_Declare(pfr
  GIT_REPOSITORY https://github.com/apolukhin/pfr_non_boost.git
  GIT_TAG b1a9798677adc191e21dd7904fb42cbe2f2889a7  # 2.3.2
)
FetchContent_MakeAvailable(pfr)

# JSON parser. config.hpp includes it as a public SDK header, so it is a usage
# requirement of this target rather than something each consumer repeats. nix
# provides it via buildInputs.
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
  FetchContent_Declare(nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG 9cca280a4d0ccf0c08f47a99aa71d1b0e52f8d03  # v3.11.3
  )
  FetchContent_MakeAvailable(nlohmann_json)
endif()

# tf.hpp exposes Eigen types in its interface. Header-only, like the SDK itself.
find_package(Eigen3 3.4 QUIET)
if(NOT Eigen3_FOUND)
  FetchContent_Declare(eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.1/eigen-3.4.1.tar.gz
    URL_HASH SHA256=b93c667d1b69265cdb4d9f30ec21f8facbbe8b307cf34c0b9942834c6d4fdbe2
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
  set(EIGEN_BUILD_DOC OFF)
  set(EIGEN_BUILD_TESTING OFF)
  set(EIGEN_BUILD_PKGCONFIG OFF)
  FetchContent_MakeAvailable(eigen)
endif()

# module.hpp runs the dispatch loop and publish workers on std::thread.
find_package(Threads REQUIRED)

# Header-only SDK. Consumers link this INTERFACE target and inherit the include
# path, the C++ standard, and the SDK's own dependencies. A consumer still sets
# CMAKE_CXX_STANDARD for the targets it builds that do not link this one.
add_library(dimos_native INTERFACE)
target_include_directories(dimos_native INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(dimos_native INTERFACE cxx_std_20)
target_link_libraries(dimos_native INTERFACE
  pfr
  nlohmann_json::nlohmann_json
  Eigen3::Eigen
  Threads::Threads
)

option(DIMOS_NATIVE_BUILD_TESTS "Build the dimos native SDK tests" OFF)
if(DIMOS_NATIVE_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

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
  Threads::Threads
)

# Zenoh transport. zenoh-cpp is header-only over zenoh-c, which is a compiled
# rust library, so unlike the rest of the SDK it cannot be vendored cheaply.
# AUTO builds it in when the packages are installed; ON demands them.
set(DIMOS_NATIVE_ZENOH AUTO CACHE STRING "Build zenoh transport support: AUTO, ON or OFF")
set_property(CACHE DIMOS_NATIVE_ZENOH PROPERTY STRINGS AUTO ON OFF)
if(NOT DIMOS_NATIVE_ZENOH STREQUAL "OFF")
  # zenohcxxConfig.cmake only defines its backend target when zenohc::lib
  # already exists, and hard-errors when no backend does, so zenoh-c is looked
  # up first and zenoh-cpp only when it was found.
  if(DIMOS_NATIVE_ZENOH STREQUAL "ON")
    find_package(zenohc REQUIRED)
    find_package(zenohcxx REQUIRED)
  else()
    find_package(zenohc QUIET)
    if(zenohc_FOUND)
      find_package(zenohcxx QUIET)
    endif()
  endif()
endif()
if(zenohcxx_FOUND)
  # The transport needs reliability QoS and the link list it waits on, which
  # zenoh-c only emits under its unstable API. That is off in an upstream source
  # build and on in every published binary, so it is detected, not assumed.
  include(CheckCXXSourceCompiles)
  get_target_property(zenohc_includes zenohc::lib INTERFACE_INCLUDE_DIRECTORIES)
  set(CMAKE_REQUIRED_INCLUDES ${zenohc_includes})
  check_cxx_source_compiles("
    #include <zenoh_configure.h>
    #if !defined(Z_FEATURE_UNSTABLE_API)
    #error unstable api off
    #endif
    int main() { return 0; }
  " DIMOS_NATIVE_ZENOH_UNSTABLE_API)
  unset(CMAKE_REQUIRED_INCLUDES)
endif()
if(zenohcxx_FOUND AND DIMOS_NATIVE_ZENOH_UNSTABLE_API)
  message(STATUS "dimos_native: zenoh-cpp found, zenoh transport enabled")
  target_compile_definitions(dimos_native INTERFACE DIMOS_NATIVE_ZENOH)
  target_link_libraries(dimos_native INTERFACE zenohcxx::zenohc)
elseif(zenohcxx_FOUND)
  set(zenoh_hint "zenoh-c is built without its unstable API, which the zenoh transport needs, \
rebuild it with -DZENOHC_BUILD_WITH_UNSTABLE_API=ON")
  if(DIMOS_NATIVE_ZENOH STREQUAL "ON")
    message(FATAL_ERROR "dimos_native: ${zenoh_hint}")
  endif()
  message(STATUS "dimos_native: ${zenoh_hint}, DIMOS_TRANSPORT=zenoh will error at startup")
else()
  message(STATUS "dimos_native: zenoh-cpp not found, DIMOS_TRANSPORT=zenoh will error at startup")
endif()

option(DIMOS_NATIVE_BUILD_TESTS "Build the dimos native SDK tests" OFF)
if(DIMOS_NATIVE_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

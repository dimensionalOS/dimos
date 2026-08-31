# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

include(FetchContent)
FetchContent_Declare(
  doctest
  GIT_REPOSITORY https://github.com/doctest/doctest.git
  GIT_TAG 1da23a3e8119ec5cce4f9388e91b065e20bf06f5  # v2.4.12
)
FetchContent_MakeAvailable(doctest)

add_executable(dimos_native_tests
  test_main.cpp
  test_transport.cpp
  test_log.cpp
  test_transport_selection.cpp
  test_config.cpp
  test_module.cpp
  test_lcm_codec.cpp
)
target_link_libraries(dimos_native_tests PRIVATE
  dimos_native
  doctest::doctest
)
target_compile_options(dimos_native_tests PRIVATE -Wall -Wextra -Werror)

# LcmTransport needs liblcm, which isn't always present (system pkg-config often
# lacks it, it comes in via nix). Compile-check the adapter only when LCM is
# discoverable, so the core suite still builds anywhere.
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(LCM IMPORTED_TARGET lcm)
endif()
if(LCM_FOUND)
  message(STATUS "dimos_native: liblcm found, including LcmTransport compile check")
  target_sources(dimos_native_tests PRIVATE test_lcm_transport.cpp)
  target_link_libraries(dimos_native_tests PRIVATE PkgConfig::LCM)
else()
  message(STATUS "dimos_native: liblcm not found, skipping LcmTransport compile check")
endif()

# Same for zenoh-cpp. dimos_native already carries its include dirs, its define
# and its link when the SDK found it.
if(zenohcxx_FOUND AND DIMOS_NATIVE_ZENOH_UNSTABLE_API)
  message(STATUS "dimos_native: zenoh-cpp found, including ZenohTransport tests")
  target_sources(dimos_native_tests PRIVATE test_zenoh_transport.cpp)
else()
  message(STATUS "dimos_native: zenoh-cpp not found, skipping ZenohTransport tests")
endif()

include(${doctest_SOURCE_DIR}/scripts/cmake/doctest.cmake)
doctest_discover_tests(dimos_native_tests)

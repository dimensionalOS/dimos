# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

include(FetchContent)
FetchContent_Declare(
  doctest
  GIT_REPOSITORY https://github.com/doctest/doctest.git
  GIT_TAG v2.4.11
)
FetchContent_MakeAvailable(doctest)

add_executable(dimos_native_tests
  test_main.cpp
  test_transport.cpp
  test_log.cpp
)
target_link_libraries(dimos_native_tests PRIVATE dimos_native doctest::doctest)
target_compile_options(dimos_native_tests PRIVATE -Wall -Wextra -Werror)

include(${doctest_SOURCE_DIR}/scripts/cmake/doctest.cmake)
doctest_discover_tests(dimos_native_tests)

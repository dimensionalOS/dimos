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
  test_lcm_transport.cpp
  test_zenoh_transport.cpp
)
target_link_libraries(dimos_native_tests PRIVATE
  dimos_native
  doctest::doctest
)
target_compile_options(dimos_native_tests PRIVATE -Wall -Wextra -Werror)
# The session goldens live with the rust suite, which owns the wire shape
# alongside dimos/protocol/service/test_zenoh_wire.py.
target_compile_definitions(dimos_native_tests PRIVATE
  DIMOS_NATIVE_GOLDENS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../rust/dimos-module/tests/fixtures"
)

include(${doctest_SOURCE_DIR}/scripts/cmake/doctest.cmake)
doctest_discover_tests(dimos_native_tests)

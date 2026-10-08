# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.15)
project(dimos_external_native LANGUAGES NONE)
if(NOT UNIX)
  message(FATAL_ERROR "This NativeModule example requires POSIX signals")
endif()
find_program(CARGO_EXECUTABLE cargo REQUIRED)
set(CARGO_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/cargo")
add_custom_command(
  OUTPUT "${CARGO_OUTPUT}/release/package_probe"
  COMMAND "${CARGO_EXECUTABLE}" build --release --locked --offline
          --manifest-path "${CMAKE_CURRENT_SOURCE_DIR}/native/Cargo.toml"
          --target-dir "${CARGO_OUTPUT}"
  DEPENDS native/Cargo.toml native/Cargo.lock native/src/main.rs
  VERBATIM
)
add_custom_target(package_probe ALL DEPENDS "${CARGO_OUTPUT}/release/package_probe")
install(PROGRAMS "${CARGO_OUTPUT}/release/package_probe"
        DESTINATION dimos_external_native/bin)

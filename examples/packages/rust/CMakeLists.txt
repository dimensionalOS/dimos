# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.15)
project(dimos_package_rust LANGUAGES NONE)
find_program(CARGO cargo REQUIRED)
set(artifact "${CMAKE_CURRENT_BINARY_DIR}/cargo/release/package_ping")
add_custom_command(
  OUTPUT "${artifact}"
  COMMAND "${CARGO}" build --release --locked
          --manifest-path "${CMAKE_CURRENT_SOURCE_DIR}/native/Cargo.toml"
          --target-dir "${CMAKE_CURRENT_BINARY_DIR}/cargo"
  DEPENDS native/Cargo.toml native/Cargo.lock native/src/main.rs
  VERBATIM
)
add_custom_target(package_ping ALL DEPENDS "${artifact}")
install(PROGRAMS "${artifact}" DESTINATION dimos_package_rust/bin COMPONENT python)

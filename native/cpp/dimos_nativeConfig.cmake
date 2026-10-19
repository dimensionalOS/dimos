# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0
# Source-package entry point: compilation happens in the consumer's build tree.
if(NOT TARGET dimos_native::dimos_native)
  add_subdirectory("${CMAKE_CURRENT_LIST_DIR}" "${CMAKE_BINARY_DIR}/dimos_native" EXCLUDE_FROM_ALL)
  add_library(dimos_native::dimos_native ALIAS dimos_native)
endif()

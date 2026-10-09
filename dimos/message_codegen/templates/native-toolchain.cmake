# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0
# ExternalProject does not inherit the parent project's CMake cache.
set(_dimos_toolchain_args)
foreach(_variable CMAKE_TOOLCHAIN_FILE CMAKE_OSX_SYSROOT CMAKE_OSX_DEPLOYMENT_TARGET CMAKE_OSX_ARCHITECTURES)
  if(DEFINED ${_variable} AND NOT "${${_variable}}" STREQUAL "")
    string(REPLACE ";" "|" _value "${${_variable}}")
    list(APPEND _dimos_toolchain_args "-D${_variable}=${_value}")
  endif()
endforeach()
if(APPLE)
  set(_dimos_origin "@loader_path")
else()
  set(_dimos_origin "$ORIGIN")
endif()

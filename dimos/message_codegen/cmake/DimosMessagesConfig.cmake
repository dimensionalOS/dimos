# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

include_guard(GLOBAL)
find_package(Python3 COMPONENTS Interpreter REQUIRED)

function(dimos_message_project)
  if(CMAKE_CROSSCOMPILING)
    message(FATAL_ERROR "DimosMessages currently supports native-host builds; supply a matching prebuilt message SDK for cross compilation")
  endif()
  set(manifest "${CMAKE_CURRENT_BINARY_DIR}/dimos-messages.json")
  set(native_env "CXX=${CMAKE_CXX_COMPILER}" "CXXFLAGS=${CMAKE_CXX_FLAGS}")
  if(CMAKE_C_COMPILER)
    list(APPEND native_env "CC=${CMAKE_C_COMPILER}" "CFLAGS=${CMAKE_C_FLAGS}")
  endif()
  execute_process(
    COMMAND "${CMAKE_COMMAND}" -E env ${native_env}
            "${Python3_EXECUTABLE}" -m dimos.message_codegen.native cpp
            "${CMAKE_CURRENT_SOURCE_DIR}" "${manifest}"
    COMMAND_ERROR_IS_FATAL ANY
  )
  file(READ "${manifest}" metadata)
  string(JSON toolchain GET "${metadata}" toolchain)
  string(JSON module GET "${metadata}" module)
  include("${toolchain}")
  # A schema edit selects a new cache prefix even when the package version is
  # unchanged during local development. Do not keep CMake's previous *_DIR.
  string(JSON module_count LENGTH "${metadata}" modules)
  math(EXPR last_module "${module_count} - 1")
  foreach(index RANGE ${last_module})
    string(JSON owned GET "${metadata}" modules ${index})
    unset(${owned}_DIR)
    unset(${owned}_DIR CACHE)
  endforeach()
  find_package(${module} CONFIG REQUIRED)
  set(DIMOS_MESSAGE_MODULE "${module}" PARENT_SCOPE)
endfunction()

# Optional SDK modules. Message-only builds do not need transport libraries.
function(dimos_native_sdk)
  if(TARGET dimos_native)
    return()
  endif()
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" -m dimos.message_codegen.native sdk-root
    OUTPUT_VARIABLE sdk
    OUTPUT_STRIP_TRAILING_WHITESPACE
    COMMAND_ERROR_IS_FATAL ANY
  )
  add_subdirectory("${sdk}/cpp" "${CMAKE_CURRENT_BINARY_DIR}/dimos-native" EXCLUDE_FROM_ALL)
endfunction()

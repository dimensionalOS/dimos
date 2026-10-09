# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20)
project(dimos_native_dependencies NONE)
include(FetchContent)
include(ExternalProject)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
if(NOT DIMOS_SUPPORT_PREFIX)
  set(DIMOS_SUPPORT_PREFIX "${CMAKE_BINARY_DIR}/install")
endif()
set(_python_path "${DIMOS_SUPPORT_PREFIX}/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")
set(_environment "PYTHONPATH=${_python_path}" "AMENT_PREFIX_PATH=${DIMOS_SUPPORT_PREFIX}" "CMAKE_PREFIX_PATH=${DIMOS_SUPPORT_PREFIX}")
file(READ "${CMAKE_CURRENT_LIST_DIR}/native_sources.json" _lock)
string(JSON _count LENGTH "${_lock}" repositories)
math(EXPR _last "${_count}-1")
foreach(_i RANGE ${_last})
  string(JSON _name MEMBER "${_lock}" repositories ${_i})
  string(JSON _url GET "${_lock}" repositories ${_name} url)
  string(JSON _revision GET "${_lock}" repositories ${_name} revision)
  FetchContent_Declare(${_name} GIT_REPOSITORY "${_url}" GIT_TAG "${_revision}"
    SOURCE_SUBDIR _dimos_fetch_sources_only)
  FetchContent_MakeAvailable(${_name})
endforeach()
string(JSON _url GET "${_lock}" fastcdr url)
string(JSON _hash GET "${_lock}" fastcdr sha256)
FetchContent_Declare(fastcdr URL "${_url}" URL_HASH "SHA256=${_hash}" SOURCE_SUBDIR _dimos_fetch_sources_only)
FetchContent_MakeAvailable(fastcdr)
ExternalProject_Add(support_fastcdr SOURCE_DIR "${fastcdr_SOURCE_DIR}" DOWNLOAD_COMMAND ""
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${DIMOS_SUPPORT_PREFIX} -DBUILD_TESTING=OFF
    -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_RPATH=$ORIGIN)
set(_previous support_fastcdr)
string(JSON _count LENGTH "${_lock}" packages)
math(EXPR _last "${_count}-1")
foreach(_i RANGE ${_last})
  string(JSON _name GET "${_lock}" packages ${_i} name)
  string(JSON _repository GET "${_lock}" packages ${_i} repository)
  string(JSON _directory GET "${_lock}" packages ${_i} directory)
  string(JSON _python GET "${_lock}" packages ${_i} python)
  set(_source "${${_repository}_SOURCE_DIR}/${_directory}")
  if(_python)
    ExternalProject_Add(support_${_name} SOURCE_DIR "${_source}" DOWNLOAD_COMMAND ""
      CONFIGURE_COMMAND "" BUILD_COMMAND ""
      INSTALL_COMMAND "${CMAKE_COMMAND}" -E env ${_environment}
        "${Python3_EXECUTABLE}" -m pip install --no-deps --no-build-isolation
        --prefix "${DIMOS_SUPPORT_PREFIX}" "<SOURCE_DIR>"
      DEPENDS ${_previous})
  else()
    ExternalProject_Add(support_${_name} SOURCE_DIR "${_source}" DOWNLOAD_COMMAND ""
      CONFIGURE_COMMAND "${CMAKE_COMMAND}" -E env ${_environment}
        "${CMAKE_COMMAND}" -S "<SOURCE_DIR>" -B "<BINARY_DIR>"
        -DCMAKE_INSTALL_PREFIX=${DIMOS_SUPPORT_PREFIX}
        -DCMAKE_PREFIX_PATH=${DIMOS_SUPPORT_PREFIX}
        -DPython3_EXECUTABLE=${Python3_EXECUTABLE}
        -DBUILD_TESTING=OFF -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release
        -DCMAKE_INSTALL_RPATH=$ORIGIN
      BUILD_COMMAND "${CMAKE_COMMAND}" -E env ${_environment} "${CMAKE_COMMAND}" --build "<BINARY_DIR>" --parallel 2
      INSTALL_COMMAND "${CMAKE_COMMAND}" -E env ${_environment} "${CMAKE_COMMAND}" --install "<BINARY_DIR>"
      DEPENDS ${_previous})
  endif()
  set(_previous support_${_name})
endforeach()

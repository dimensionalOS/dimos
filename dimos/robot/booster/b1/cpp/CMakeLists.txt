cmake_minimum_required(VERSION 3.18)
project(booster_b1_native LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

foreach(
  NIX_PROVIDED_PATH
  IN ITEMS BOOSTER_SDK_ROOT DIMOS_LCM_SOURCE_DIR DIMOS_NATIVE_COMMON_DIR
)
  if(NOT DEFINED ${NIX_PROVIDED_PATH} OR "${${NIX_PROVIDED_PATH}}" STREQUAL "")
    message(FATAL_ERROR "${NIX_PROVIDED_PATH} must be provided by the Nix build")
  endif()
endforeach()

set(BOOSTER_SDK_ARCH "${CMAKE_SYSTEM_PROCESSOR}")
if(BOOSTER_SDK_ARCH MATCHES "^(AMD64|amd64|x64)$")
  set(BOOSTER_SDK_ARCH x86_64)
elseif(BOOSTER_SDK_ARCH MATCHES "^(ARM64|arm64)$")
  set(BOOSTER_SDK_ARCH aarch64)
endif()

set(
  BOOSTER_SDK_LIBRARY
  "${BOOSTER_SDK_ROOT}/lib/${BOOSTER_SDK_ARCH}/libbooster_robotics_sdk.a"
)
if(NOT EXISTS "${BOOSTER_SDK_ROOT}/include/booster/robot/b1/b1_loco_client.hpp")
  message(FATAL_ERROR "Booster Robotics SDK headers not found under ${BOOSTER_SDK_ROOT}/include")
endif()
if(NOT EXISTS "${BOOSTER_SDK_LIBRARY}")
  message(FATAL_ERROR "Booster Robotics SDK library not found: ${BOOSTER_SDK_LIBRARY}")
endif()

set(DIMOS_LCM_INCLUDE_DIR "${DIMOS_LCM_SOURCE_DIR}/generated/cpp_lcm_msgs")
if(NOT EXISTS "${DIMOS_LCM_INCLUDE_DIR}/geometry_msgs/Twist.hpp")
  message(FATAL_ERROR "Generated dimos-lcm C++ headers not found: ${DIMOS_LCM_INCLUDE_DIR}")
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(DIMOS_LCM REQUIRED IMPORTED_TARGET lcm)

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)

if(NOT EXISTS "${DIMOS_NATIVE_COMMON_DIR}/dimos_native_module.hpp")
  message(FATAL_ERROR "dimOS native module headers not found: ${DIMOS_NATIVE_COMMON_DIR}")
endif()

add_executable(
  booster_b1_teleop_native
  teleop.cpp
)
add_executable(
  booster_camera_native
  camera.cpp
)
add_library(
  booster_camera_conversion
  STATIC
  camera_conversion.cpp
)
target_include_directories(
  booster_camera_conversion
  PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(
  booster_camera_conversion
  PUBLIC
  ${OpenCV_LIBS}
)

function(configure_booster_native_target target)
  target_include_directories(
    ${target}
    BEFORE PRIVATE
    "${BOOSTER_SDK_ROOT}/include"
    ${OpenCV_INCLUDE_DIRS}
    "${DIMOS_LCM_INCLUDE_DIR}"
    "${DIMOS_NATIVE_COMMON_DIR}"
  )
  target_link_libraries(
    ${target}
    PRIVATE
    "${BOOSTER_SDK_LIBRARY}"
    PkgConfig::DIMOS_LCM
    ${OpenCV_LIBS}
    Threads::Threads
    ${CMAKE_DL_LIBS}
    rt
  )
endfunction()

configure_booster_native_target(booster_b1_teleop_native)
configure_booster_native_target(booster_camera_native)
target_link_libraries(booster_camera_native PRIVATE booster_camera_conversion)

include(CTest)
if(BUILD_TESTING)
  add_executable(
    booster_camera_conversion_test
    test_camera_conversion.cpp
  )
  target_link_libraries(
    booster_camera_conversion_test
    PRIVATE
    booster_camera_conversion
  )
  add_test(
    NAME booster_camera_conversion_test
    COMMAND booster_camera_conversion_test
  )
endif()

install(TARGETS booster_b1_teleop_native booster_camera_native DESTINATION bin)

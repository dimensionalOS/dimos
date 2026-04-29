cmake_minimum_required(VERSION 3.16)
project(arduino_wire_compat_test CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# dimos-lcm C++ headers (LCM-generated reference types)
set(DIMOS_LCM_DIR "" CACHE PATH "Path to dimos-lcm repo (contains generated/cpp_lcm_msgs)")
if(NOT DIMOS_LCM_DIR)
    message(FATAL_ERROR "Set -DDIMOS_LCM_DIR to the dimos-lcm source directory")
endif()
set(LCM_CPP_MSGS "${DIMOS_LCM_DIR}/generated/cpp_lcm_msgs")

# Arduino C message headers from dimos-lcm
set(ARDUINO_MSGS "${DIMOS_LCM_DIR}/generated/arduino_c_msgs")
set(ARDUINO_COMMON "${CMAKE_CURRENT_SOURCE_DIR}/../common")

# LCM library (for lcm_coretypes.h used by the C++ reference types)
find_package(lcm REQUIRED)

add_executable(test_wire_compat test_wire_compat.cpp)

target_include_directories(test_wire_compat PRIVATE
    ${ARDUINO_MSGS}       # C headers from dimos-lcm: geometry_msgs/Vector3.h etc.
    ${ARDUINO_COMMON}     # dsp_protocol.h, dimos_lcm_serial.h
    ${LCM_CPP_MSGS}       # LCM C++ reference: geometry_msgs/Vector3.hpp etc.
)

target_link_libraries(test_wire_compat PRIVATE lcm)

install(TARGETS test_wire_compat DESTINATION bin)

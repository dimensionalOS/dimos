cmake_minimum_required(VERSION 3.16)
project(arduino_bridge CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# dimos-lcm C++ headers (for fingerprint hash computation)
set(DIMOS_LCM_DIR "" CACHE PATH "Path to dimos-lcm repo")
if(NOT DIMOS_LCM_DIR)
    message(FATAL_ERROR "Set -DDIMOS_LCM_DIR to the dimos-lcm source directory")
endif()
set(LCM_CPP_MSGS "${DIMOS_LCM_DIR}/generated/cpp_lcm_msgs")

# Arduino common headers (dsp_protocol.h, dimos_lcm_serial.h)
set(ARDUINO_COMMON "${CMAKE_CURRENT_SOURCE_DIR}/../common")

# Arduino C message headers from dimos-lcm (lcm_coretypes_arduino.h, etc.)
set(ARDUINO_MSGS "${DIMOS_LCM_DIR}/generated/arduino_c_msgs")

# LCM library
find_package(lcm REQUIRED)

add_executable(arduino_bridge main.cpp)

target_include_directories(arduino_bridge PRIVATE
    ${LCM_CPP_MSGS}       # LCM C++ generated headers
    ${ARDUINO_COMMON}      # dsp_protocol.h, dimos_lcm_serial.h
    ${ARDUINO_MSGS}        # lcm_coretypes_arduino.h, dimos_lcm_pubsub.h
)

target_link_libraries(arduino_bridge PRIVATE lcm pthread)

install(TARGETS arduino_bridge DESTINATION bin)

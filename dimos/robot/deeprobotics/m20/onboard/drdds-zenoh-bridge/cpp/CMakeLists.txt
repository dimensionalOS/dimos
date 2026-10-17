cmake_minimum_required(VERSION 3.14)
project(m20_drdds_zenoh_bridge CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# dimos_lcm: generated C++ LCM message headers (sensor_msgs/, nav_msgs/, ...).
# These are header-only (inline encode()/getHash()); we do NOT link liblcm — the
# LCM library is only the udpm transport, and the carrier here is Zenoh instead.
# Offline builds (the M20 boxes have no clean internet): pass a local checkout via
#   -DFETCHCONTENT_SOURCE_DIR_DIMOS_LCM=/path/to/dimos-lcm
if(DEFINED DIMOS_LCM_DIR)
  set(dimos_lcm_SOURCE_DIR ${DIMOS_LCM_DIR})
else()
  include(FetchContent)
  FetchContent_Declare(dimos_lcm
    GIT_REPOSITORY https://github.com/dimensionalOS/dimos-lcm.git
    GIT_TAG 0a1c24710ce2f7a569e1673617102cc5254a75e2
  )
  FetchContent_GetProperties(dimos_lcm)
  if(NOT dimos_lcm_POPULATED)
    FetchContent_Populate(dimos_lcm)
  endif()
endif()

add_executable(m20_drdds_zenoh_bridge main.cpp)

# drdds/dridl live under /usr/local (DeepRobotics SDK); zenoh-c (libzenohc.so +
# headers) is installed to /usr/local too.
target_include_directories(m20_drdds_zenoh_bridge PRIVATE
  /usr/local/include
  /usr/local/include/drdds
  /usr/local/include/dridl
  ${dimos_lcm_SOURCE_DIR}/generated/cpp_lcm_msgs
)

target_link_directories(m20_drdds_zenoh_bridge PRIVATE
  /usr/local/lib
)

target_link_libraries(m20_drdds_zenoh_bridge
  drdds
  fastrtps
  fastcdr
  zenohc
  pthread
)
target_compile_options(m20_drdds_zenoh_bridge PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS m20_drdds_zenoh_bridge DESTINATION bin)

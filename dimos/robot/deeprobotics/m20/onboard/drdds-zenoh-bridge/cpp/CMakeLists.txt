cmake_minimum_required(VERSION 3.14)
project(m20_drdds_zenoh_bridge CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(dimos_generated 0.0.14 EXACT REQUIRED CONFIG)

add_executable(m20_drdds_zenoh_bridge main.cpp cdr_wire.cpp)

# drdds/dridl live under /usr/local (DeepRobotics SDK); zenoh-c (libzenohc.so +
# headers) is installed to /usr/local too.
target_include_directories(m20_drdds_zenoh_bridge PRIVATE
  /usr/local/include
  /usr/local/include/drdds
  /usr/local/include/dridl
)

target_link_directories(m20_drdds_zenoh_bridge PRIVATE
  /usr/local/lib
)

target_link_libraries(m20_drdds_zenoh_bridge
  dimos_generated::messages
  drdds
  fastrtps
  fastcdr
  zenohc
  pthread
)
target_compile_options(m20_drdds_zenoh_bridge PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS m20_drdds_zenoh_bridge DESTINATION bin)

cmake_minimum_required(VERSION 3.14)
project(upsample_ground_native CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
FetchContent_Declare(
  dimos_lcm
  GIT_REPOSITORY https://github.com/dimensionalOS/dimos-lcm.git
  GIT_TAG main
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(dimos_lcm)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)

set(
  DIMOS_NATIVE_COMMON_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../hardware/sensors/lidar/common"
)

add_executable(upsample_ground_native main.cpp)
# dimos-lcm's generated hash code stores function pointers in void pointers.
# GCC 9 needs -fpermissive for those generated headers.
target_compile_options(upsample_ground_native PRIVATE -Wall -Wextra -Wpedantic -fpermissive)
target_include_directories(
  upsample_ground_native
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${DIMOS_NATIVE_COMMON_DIR}
    ${LCM_INCLUDE_DIRS}
)
target_include_directories(
  upsample_ground_native
  SYSTEM PRIVATE ${dimos_lcm_SOURCE_DIR}/generated/cpp_lcm_msgs
)
target_link_directories(upsample_ground_native PRIVATE ${LCM_LIBRARY_DIRS})
target_link_libraries(upsample_ground_native PRIVATE ${LCM_LIBRARIES})

enable_testing()
add_executable(test_timestamp_synchronizer test_timestamp_synchronizer.cpp)
target_compile_options(test_timestamp_synchronizer PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(test_timestamp_synchronizer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME timestamp_synchronizer COMMAND test_timestamp_synchronizer)

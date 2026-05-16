cmake_minimum_required(VERSION 3.14)
project(rtab_map CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

include(FetchContent)
FetchContent_Declare(dimos_lcm
  GIT_REPOSITORY https://github.com/dimensionalOS/dimos-lcm.git
  GIT_TAG main
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(dimos_lcm)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common filters io)
find_package(OpenCV REQUIRED)

# RTAB-Map. The exported RTABMapConfig.cmake from nixpkgs unconditionally
# transitively depends on Qt6 GUI, VTK::GUISupportQt, and several optional
# sensor SDKs even when we don't use any of them, so it can't be used
# directly with a headless build. Resolve the headers and the core lib
# manually instead.
set(RTABMAP_EXPECTED_VERSION "0.23")
find_path(RTABMAP_INCLUDE_DIR
  NAMES rtabmap/core/Rtabmap.h
  PATH_SUFFIXES "rtabmap-${RTABMAP_EXPECTED_VERSION}"
  REQUIRED
)
find_library(RTABMAP_CORE_LIBRARY
  NAMES rtabmap_core
  REQUIRED
)
find_library(RTABMAP_UTILITE_LIBRARY
  NAMES rtabmap_utilite
  REQUIRED
)
message(STATUS "Found rtabmap headers: ${RTABMAP_INCLUDE_DIR}")
message(STATUS "Found rtabmap_core lib: ${RTABMAP_CORE_LIBRARY}")
message(STATUS "Found rtabmap_utilite lib: ${RTABMAP_UTILITE_LIBRARY}")

add_definitions(-DUSE_PCL)

add_executable(rtab_map main.cpp)

target_include_directories(rtab_map PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${dimos_lcm_SOURCE_DIR}/generated/cpp_lcm_msgs
  ${LCM_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${RTABMAP_INCLUDE_DIR}
)

target_link_libraries(rtab_map PRIVATE
  ${LCM_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${RTABMAP_CORE_LIBRARY}
  ${RTABMAP_UTILITE_LIBRARY}
)

target_link_directories(rtab_map PRIVATE
  ${LCM_LIBRARY_DIRS}
)

install(TARGETS rtab_map DESTINATION bin)

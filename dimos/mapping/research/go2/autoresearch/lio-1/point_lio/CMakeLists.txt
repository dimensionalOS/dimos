cmake_minimum_required(VERSION 3.10)
project(point_lio_noros LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# MP_PROC_NUM controls Point-LIO's internal OpenMP fanout; the original
# CMakeLists scaled it to N-2 cores. Match that here.
include(ProcessorCount)
ProcessorCount(N)
if(N GREATER 5)
  add_definitions(-DMP_EN -DMP_PROC_NUM=4)
elseif(N GREATER 3)
  math(EXPR PROC_NUM "${N} - 2")
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(Eigen3 REQUIRED)

# PCL's CMake config drags in all of VTK, including VTK::mpi which
# requires MPI::MPI_C. Skip find_package(PCL) entirely and link only the
# components we use directly. PCL headers themselves are header-only for
# our purposes (point types + voxel filter inlines); the .so's for
# common/io/filters/kdtree provide the heavy bits.
find_path(PCL_INCLUDE_DIR
  NAMES pcl/point_cloud.h
  HINTS /usr/include/pcl-1.14 /usr/include/pcl-1.12 /usr/include/pcl
)
if(NOT PCL_INCLUDE_DIR)
  message(FATAL_ERROR "pcl headers not found; try `apt install libpcl-dev`")
endif()
find_library(PCL_COMMON_LIB NAMES pcl_common  REQUIRED)
find_library(PCL_IO_LIB     NAMES pcl_io      REQUIRED)
find_library(PCL_FILTERS_LIB NAMES pcl_filters REQUIRED)
find_library(PCL_KDTREE_LIB NAMES pcl_kdtree  REQUIRED)

# Boost.System is header-only in modern Boost (≥1.69) and ships no compiled
# component/CMake config in nixpkgs, so request only filesystem. Point-LIO's
# own sources use no Boost directly; this is here for PCL's header needs.
find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(yaml-cpp REQUIRED)

# noros shim headers (include/ros/ros.h etc.) live in include/ and
# shadow real ROS headers. Put include/ first so they win even if a
# system ROS is installed.
include_directories(BEFORE include)
include_directories(${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIR})

add_executable(pointlio_mapping
  src/laserMapping.cpp
  include/ikd-Tree/ikd_Tree.cpp
  src/parameters.cpp
  src/preprocess.cpp
  src/Estimator.cpp
)
target_link_libraries(pointlio_mapping
  ${PCL_COMMON_LIB} ${PCL_IO_LIB} ${PCL_FILTERS_LIB} ${PCL_KDTREE_LIB}
  Boost::filesystem
  yaml-cpp
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(pointlio_mapping OpenMP::OpenMP_CXX)
endif()

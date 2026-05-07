cmake_minimum_required(VERSION 3.14)
project(pgo_native CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

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
find_package(PCL 1.8 REQUIRED COMPONENTS common filters kdtree registration io)
find_package(GTSAM REQUIRED)

add_definitions(-DUSE_PCL)

add_executable(pgo_native
  main.cpp
  simple_pgo.cpp
  commons.cpp
)

target_include_directories(pgo_native PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${dimos_lcm_SOURCE_DIR}/generated/cpp_lcm_msgs
  ${LCM_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

target_link_libraries(pgo_native PRIVATE
  ${LCM_LIBRARIES}
  ${PCL_LIBRARIES}
  gtsam
)

target_link_directories(pgo_native PRIVATE
  ${LCM_LIBRARY_DIRS}
)

install(TARGETS pgo_native DESTINATION bin)

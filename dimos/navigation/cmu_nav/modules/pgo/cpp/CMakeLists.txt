cmake_minimum_required(VERSION 3.14)
project(pgo CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

include(FetchContent)
FetchContent_Declare(dimos_lcm
  GIT_REPOSITORY https://github.com/dimensionalOS/dimos-lcm.git
  GIT_TAG main
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(dimos_lcm)

# JSON parser used by the SDK's stdin config. nix provides it via buildInputs.
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
  FetchContent_Declare(nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
  )
  FetchContent_MakeAvailable(nlohmann_json)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common filters kdtree registration io)
find_package(GTSAM REQUIRED)
find_package(Threads REQUIRED)

# The header-only dimos native C++ SDK. Overridable so nix can point at the
# flake input instead of the in-repo relative path.
if(NOT DEFINED DIMOS_NATIVE_CPP_DIR)
  set(DIMOS_NATIVE_CPP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../native/cpp)
endif()
add_subdirectory(${DIMOS_NATIVE_CPP_DIR} ${CMAKE_BINARY_DIR}/dimos_native)

add_definitions(-DUSE_PCL)

add_executable(pgo
  main.cpp
  simple_pgo.cpp
  commons.cpp
)

target_include_directories(pgo PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${dimos_lcm_SOURCE_DIR}/generated/cpp_lcm_msgs
  ${LCM_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

target_link_libraries(pgo PRIVATE
  dimos_native
  nlohmann_json::nlohmann_json
  Threads::Threads
  ${LCM_LIBRARIES}
  ${PCL_LIBRARIES}
  gtsam
)

target_link_directories(pgo PRIVATE
  ${LCM_LIBRARY_DIRS}
)

install(TARGETS pgo DESTINATION bin)

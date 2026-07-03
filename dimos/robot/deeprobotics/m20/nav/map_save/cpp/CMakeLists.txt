cmake_minimum_required(VERSION 3.14)
project(m20_pointcloud_map_save CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fpermissive")

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/result" CACHE PATH "" FORCE)
endif()

find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)
find_package(PCL 1.8 REQUIRED COMPONENTS common filters io)
find_program(LCM_GEN lcm-gen REQUIRED)

execute_process(
  COMMAND "${Python3_EXECUTABLE}" -c "import pathlib, dimos_lcm; print(pathlib.Path(dimos_lcm.__file__).resolve().parent / 'lcm_files')"
  OUTPUT_VARIABLE DIMOS_LCM_LCM_FILES
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(NOT EXISTS "${DIMOS_LCM_LCM_FILES}")
  message(FATAL_ERROR "Could not locate local dimos_lcm lcm_files directory: ${DIMOS_LCM_LCM_FILES}")
endif()

file(GLOB DIMOS_LCM_INPUTS "${DIMOS_LCM_LCM_FILES}/*.lcm")
if(NOT DIMOS_LCM_INPUTS)
  message(FATAL_ERROR "No .lcm files found in ${DIMOS_LCM_LCM_FILES}")
endif()

set(DIMOS_LCM_GEN_DIR "${CMAKE_BINARY_DIR}/generated/cpp_lcm_msgs")
add_custom_target(generate_dimos_lcm_headers
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${DIMOS_LCM_GEN_DIR}"
  COMMAND "${LCM_GEN}" --cpp --cpp-hpath "${DIMOS_LCM_GEN_DIR}" --cpp-include "${DIMOS_LCM_GEN_DIR}" ${DIMOS_LCM_INPUTS}
  BYPRODUCTS "${DIMOS_LCM_GEN_DIR}"
  DEPENDS ${DIMOS_LCM_INPUTS}
  VERBATIM
)

add_executable(m20_pointcloud_map_save main.cpp)
add_dependencies(m20_pointcloud_map_save generate_dimos_lcm_headers)

if(NOT DIMOS_NATIVE_COMMON_DIR)
  set(DIMOS_NATIVE_COMMON_DIR
      ${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../hardware/sensors/lidar/common)
endif()

target_include_directories(m20_pointcloud_map_save PRIVATE
  ${DIMOS_LCM_GEN_DIR}
  ${LCM_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${DIMOS_NATIVE_COMMON_DIR}
)

target_link_directories(m20_pointcloud_map_save PRIVATE
  ${LCM_LIBRARY_DIRS}
)

target_link_libraries(m20_pointcloud_map_save PRIVATE
  ${LCM_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS m20_pointcloud_map_save DESTINATION bin)

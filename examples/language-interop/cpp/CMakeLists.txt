cmake_minimum_required(VERSION 3.20)
project(robot_control LANGUAGES CXX)

find_package(dimos_generated CONFIG REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED IMPORTED_TARGET lcm)

add_executable(robot_control main.cpp)
target_compile_features(robot_control PRIVATE cxx_std_17)
target_link_libraries(robot_control PRIVATE dimos_generated::messages PkgConfig::LCM)

cmake_minimum_required(VERSION 3.14)
project(smart_nav_native CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/result" CACHE PATH "" FORCE)
endif()

# Option: USE_PCL (default ON). When OFF, uses lightweight implementations.
option(USE_PCL "Use PCL for point cloud operations" ON)

# Fetch dependencies
include(FetchContent)

# dimos-lcm C++ message headers
FetchContent_Declare(dimos_lcm
  GIT_REPOSITORY https://github.com/dimensionalOS/dimos-lcm.git
  GIT_TAG main
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(dimos_lcm)

# LCM
find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)

# Eigen3
find_package(Eigen3 REQUIRED)

# PCL (optional)
if(USE_PCL)
  find_package(PCL 1.8 REQUIRED COMPONENTS common filters kdtree)
  add_definitions(-DUSE_PCL)
endif()

# Common include directories
set(SMARTNAV_COMMON_INCLUDES
  ${CMAKE_CURRENT_SOURCE_DIR}/common
  ${dimos_lcm_SOURCE_DIR}/generated/cpp_lcm_msgs
  ${LCM_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

set(SMARTNAV_COMMON_LIBS
  ${LCM_LIBRARIES}
)

set(SMARTNAV_COMMON_LIB_DIRS
  ${LCM_LIBRARY_DIRS}
)

if(USE_PCL)
  list(APPEND SMARTNAV_COMMON_INCLUDES ${PCL_INCLUDE_DIRS})
  list(APPEND SMARTNAV_COMMON_LIBS ${PCL_LIBRARIES})
endif()

# --- Terrain Analysis ---
add_executable(terrain_analysis
  modules/terrain_analysis/main.cpp
)
target_include_directories(terrain_analysis PRIVATE ${SMARTNAV_COMMON_INCLUDES})
target_link_libraries(terrain_analysis PRIVATE ${SMARTNAV_COMMON_LIBS})
target_link_directories(terrain_analysis PRIVATE ${SMARTNAV_COMMON_LIB_DIRS})

# --- Local Planner ---
add_executable(local_planner
  modules/local_planner/main.cpp
)
target_include_directories(local_planner PRIVATE ${SMARTNAV_COMMON_INCLUDES})
target_link_libraries(local_planner PRIVATE ${SMARTNAV_COMMON_LIBS})
target_link_directories(local_planner PRIVATE ${SMARTNAV_COMMON_LIB_DIRS})

# --- Path Follower ---
add_executable(path_follower
  modules/path_follower/main.cpp
)
target_include_directories(path_follower PRIVATE ${SMARTNAV_COMMON_INCLUDES})
target_link_libraries(path_follower PRIVATE ${SMARTNAV_COMMON_LIBS})
target_link_directories(path_follower PRIVATE ${SMARTNAV_COMMON_LIB_DIRS})

# --- FAR Planner ---
add_executable(far_planner
  modules/far_planner/main.cpp
)
target_include_directories(far_planner PRIVATE ${SMARTNAV_COMMON_INCLUDES})
target_link_libraries(far_planner PRIVATE ${SMARTNAV_COMMON_LIBS})
target_link_directories(far_planner PRIVATE ${SMARTNAV_COMMON_LIB_DIRS})
# FAR planner uses OpenCV for contour detection
find_package(OpenCV QUIET COMPONENTS core imgproc)
if(OpenCV_FOUND)
  target_include_directories(far_planner PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(far_planner PRIVATE ${OpenCV_LIBS})
  target_compile_definitions(far_planner PRIVATE HAS_OPENCV)
endif()

# --- PGO (Pose Graph Optimization) ---
find_package(GTSAM QUIET)
if(USE_PCL AND GTSAM_FOUND)
  add_executable(pgo
    modules/pgo/main.cpp
  )
  target_include_directories(pgo PRIVATE ${SMARTNAV_COMMON_INCLUDES})
  target_link_libraries(pgo PRIVATE ${SMARTNAV_COMMON_LIBS} gtsam)
  target_link_directories(pgo PRIVATE ${SMARTNAV_COMMON_LIB_DIRS})
  # PCL registration component needed for ICP
  find_package(PCL 1.8 REQUIRED COMPONENTS registration)
  target_include_directories(pgo PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_libraries(pgo PRIVATE ${PCL_LIBRARIES})
endif()

# --- AriseSLAM ---
find_package(Ceres QUIET)
if(USE_PCL AND Ceres_FOUND)
  add_executable(arise_slam
    modules/arise_slam/main.cpp
  )
  target_include_directories(arise_slam PRIVATE ${SMARTNAV_COMMON_INCLUDES})
  target_link_libraries(arise_slam PRIVATE ${SMARTNAV_COMMON_LIBS} Ceres::ceres)
  target_link_directories(arise_slam PRIVATE ${SMARTNAV_COMMON_LIB_DIRS})
endif()

# --- TARE Planner ---
add_executable(tare_planner
  modules/tare_planner/main.cpp
)
target_include_directories(tare_planner PRIVATE ${SMARTNAV_COMMON_INCLUDES})
target_link_libraries(tare_planner PRIVATE ${SMARTNAV_COMMON_LIBS})
target_link_directories(tare_planner PRIVATE ${SMARTNAV_COMMON_LIB_DIRS})

# Install all targets
set(SMARTNAV_INSTALL_TARGETS
  terrain_analysis
  local_planner
  path_follower
  far_planner
  tare_planner
)
if(USE_PCL AND GTSAM_FOUND)
  list(APPEND SMARTNAV_INSTALL_TARGETS pgo)
endif()
if(USE_PCL AND Ceres_FOUND)
  list(APPEND SMARTNAV_INSTALL_TARGETS arise_slam)
endif()
install(TARGETS ${SMARTNAV_INSTALL_TARGETS} DESTINATION bin)

# The one piece of native code in this project, and it exists to answer one question:
# does the AprilTag reference C library beat OpenCV's ArUco on this phone, on this flight's
# frames, at equal detection rate. See docs/measurements/2026-07-27-apriltag-c-vs-opencv.md.
#
# It lives outside `src/main` on purpose. AGP compiles `externalNativeBuild` for the *application*
# variant regardless of where the sources sit, so putting them here keeps a measurement's C code
# out of the shipped Kotlin tree while still producing a real `libapriltagjni.so` in the APK —
# which is what the size delta in the measurement is measuring.
cmake_minimum_required(VERSION 3.22.1)
project(apriltagjni C)

# The vendored library, unmodified, at 0e16a12 — see apriltag/VENDOR.md. Globbing is normally a
# mistake in CMake; here it is right, because the file list is upstream's and any edit to it would
# be a re-vendor rather than a source change of ours.
file(GLOB APRILTAG_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/apriltag/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/apriltag/common/*.c")

add_library(apriltagjni SHARED tag_jni.c ${APRILTAG_SOURCES})

# Upstream's own includes are of the form `#include "common/zarray.h"`, so the library's root is
# the include root.
target_include_directories(apriltagjni PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/apriltag")

# **-O3 is not a preference, it is the difference between a measurement and a fiction.** AGP
# configures the *debug* variant's native build with `CMAKE_BUILD_TYPE=Debug`, i.e. `-O0 -g`, and
# every number in this comparison comes from a debug build because that is what the instrumented
# profiler installs. An unoptimised apriltag is several times slower than an optimised one, and
# comparing it against OpenCV's prebuilt release AAR would have been a comparison of build flags.
# Flags here land after `CMAKE_C_FLAGS_<CONFIG>` on the command line, and the last `-O` wins.
target_compile_options(apriltagjni PRIVATE
    -O3
    -ffast-math
    -fvisibility=hidden
    -Wno-unused-variable
    -Wno-unused-but-set-variable)

find_library(log-lib log)
target_link_libraries(apriltagjni ${log-lib} m)

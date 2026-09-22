#!/usr/bin/env bash
set -euo pipefail

# Generate/install the built-in C++ message package after setup_message_codegen.sh.
# Uses Python's standard library only; no ROS installation or Python extension build.
cd "$(dirname "$0")/.."
python3 -m dimos.message_codegen.generate --output build/message-codegen/native
cmake -S build/message-codegen/native/cpp -B build/message-codegen/native/cmake \
  -DDIMOS_BUILD_PYTHON=OFF \
  -DCMAKE_PREFIX_PATH="$PWD/build/message-codegen/install${CMAKE_PREFIX_PATH:+;$CMAKE_PREFIX_PATH}" \
  -DCMAKE_INSTALL_PREFIX="$PWD/build/message-codegen/install"
cmake --install build/message-codegen/native/cmake

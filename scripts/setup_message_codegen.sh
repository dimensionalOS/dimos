#!/usr/bin/env bash
set -euo pipefail

# Install the pinned standalone CDR dependency into an ignored local prefix.
# This is a build-environment setup step, never part of message generation.
cd "$(dirname "$0")/.."
mkdir -p build/message-codegen
curl --fail --location --retry 3 \
  https://github.com/eProsima/Fast-CDR/archive/refs/tags/v2.4.0.tar.gz \
  -o build/message-codegen/fastcdr.tar.gz
echo '79d8466107dd6b7d1defe961c4aa31735038937cf9dd1175cf6b0da0df2209ab  build/message-codegen/fastcdr.tar.gz' | sha256sum -c -
tar -xzf build/message-codegen/fastcdr.tar.gz -C build/message-codegen
cmake -S build/message-codegen/Fast-CDR-2.4.0 -B build/message-codegen/fastcdr-build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PWD/build/message-codegen/install" \
  -DBUILD_TESTING=OFF -DBUILD_SHARED_LIBS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON
cmake --build build/message-codegen/fastcdr-build -j 2
cmake --install build/message-codegen/fastcdr-build

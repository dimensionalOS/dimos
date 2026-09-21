#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
message_version="${1:?Pass the release version}"
message_output="$PWD/build/message-codegen/release"
.venv/bin/python -m dimos.message_codegen.generate \
  --version "$message_version" --output "$message_output"
cmake -S "$message_output/cpp" -B "$message_output/cmake" \
  -DDIMOS_BUILD_PYTHON=OFF -DCMAKE_PREFIX_PATH="$PWD/build/message-codegen/install" \
  -DCMAKE_INSTALL_PREFIX="$message_output/install"
cmake --build "$message_output/cmake"
cmake --install "$message_output/cmake"
cargo package --manifest-path "$message_output/rust/Cargo.toml" --allow-dirty
mkdir -p "$message_output/dist"
tar -czf "$message_output/dist/dimos-messages-cmake-$message_version.tar.gz" \
  -C "$message_output/install" .
cp "$message_output/rust/target/package/dimos-generated-messages-$message_version.crate" \
  "$message_output/dist/"

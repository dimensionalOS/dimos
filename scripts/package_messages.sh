#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ $# -ne 0 ]]; then
  echo "Message version comes from packages/dimos-generated/pyproject.toml; no version argument is accepted" >&2
  exit 2
fi
message_output="$PWD/build/message-codegen/release"
message_version="$(.venv/bin/python - <<'PYTHON'
from pathlib import Path
import tomllib
from dimos.message_codegen.generate import generate
version = tomllib.loads(Path("packages/dimos-generated/pyproject.toml").read_text())["project"]["version"]
# Built-ins must never absorb custom providers installed in the maintainer environment.
generate([], Path("build/message-codegen/release"), version=version, shared=True)
print(version)
PYTHON
)"
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

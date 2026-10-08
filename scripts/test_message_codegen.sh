#!/usr/bin/env bash
set -euo pipefail

# Run the same native builds, tests, and visible demos locally and in CI.
cd "$(dirname "$0")/.."
.venv/bin/python -m dimos.message_codegen.generate \
  --package-root examples/message-codegen --output build/message-codegen/demo
c++ -std=c++17 -Ibuild/message-codegen/install/include -Ibuild/message-codegen/demo/cpp \
  examples/message-codegen/relay.cpp build/message-codegen/install/lib/libfastcdr.a \
  -o build/message-codegen/demo/cpp-relay
mkdir -p build/message-codegen/demo/rust/src/bin build/message-codegen/demo/evidence
cp examples/message-codegen/relay.rs build/message-codegen/demo/rust/src/bin/relay.rs
cargo build --manifest-path build/message-codegen/demo/rust/Cargo.toml
export PYTHONPATH="$PWD/build/message-codegen/demo/python${PYTHONPATH:+:$PYTHONPATH}"
.venv/bin/python -m pytest dimos/message_codegen --noconftest -o addopts='' -q \
  | tee build/message-codegen/demo/evidence/pytest.txt
.venv/bin/python examples/message-codegen/demo_relay.py --build build/message-codegen/demo \
  | tee build/message-codegen/demo/evidence/relay.txt
.venv/bin/python examples/message-codegen/demo_conformance.py --build build/message-codegen/demo \
  | tee build/message-codegen/demo/evidence/conformance.txt
.venv/bin/python examples/message-codegen/demo_buffers.py --functional-only \
  | tee build/message-codegen/demo/evidence/buffers.txt

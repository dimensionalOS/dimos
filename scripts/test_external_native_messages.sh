#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
# Run test_message_packages.sh first: it adds application_note locally and installs packages.
application="$PWD/build/message-codegen/external-app"
native_demo="$PWD/build/message-codegen/external-native"
mkdir -p "$native_demo/rust/src" "$native_demo/evidence"
cp examples/message-codegen/external-native/relay.rs "$native_demo/rust/src/main.rs"
cat > "$native_demo/rust/Cargo.toml" <<MANIFEST
[package]
name = "external-native-relay"
version = "0.1.0"
edition = "2024"
[workspace]
[dependencies]
external-telemetry-messages = { path = "$application/rust-crate/external-telemetry-messages-0.1.0" }
dimos-module = { path = "$PWD/native/rust/dimos-module" }
tokio = { version = "1", features = ["rt-multi-thread", "macros"] }
tracing = "0.1"
MANIFEST
cargo build --offline --manifest-path "$native_demo/rust/Cargo.toml"
cargo test --offline --manifest-path "$native_demo/rust/Cargo.toml"
cmake -S examples/message-codegen/external-native -B "$native_demo/cpp" \
  -DCMAKE_PREFIX_PATH="$PWD/build/message-codegen/install;$application/install${CMAKE_PREFIX_PATH:+;$CMAKE_PREFIX_PATH}" \
  -DFETCHCONTENT_SOURCE_DIR_PFR="$PWD/build/native-cpp/_deps/pfr-src"
cmake --build "$native_demo/cpp" -j 2
application_site=$("$application/venv/bin/python" -c 'import sysconfig; print(sysconfig.get_path("platlib"))')
PYTHONPATH="$PWD:$PWD/build/message-codegen/demo/cpp/build:$application_site${PYTHONPATH:+:$PYTHONPATH}" \
  .venv/bin/python examples/message-codegen/demo_external_native.py --build "$native_demo" \
  | tee "$native_demo/evidence/exchange.txt"

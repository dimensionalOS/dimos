#!/usr/bin/env bash
# Rebuild the wasm bundle for the demo (requires: rustup target add
# wasm32-unknown-unknown; cargo install wasm-bindgen-cli --version 0.2.126)
set -euo pipefail
cd "$(dirname "$0")/.."
cargo build --release --target wasm32-unknown-unknown --no-default-features --features wasm
wasm-bindgen --target web --out-dir web/pkg \
    target/wasm32-unknown-unknown/release/dimos_repulsive_field.wasm
echo "serve with: python3 -m http.server -d web 8094"

#!/usr/bin/env bash
set -euo pipefail

checkout="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source_dir="${1:-}"
temporary=""
if [[ -z "$source_dir" ]]; then
  temporary="$(mktemp -d)"
  trap 'rm -rf "$temporary"' EXIT
  git clone https://github.com/Kaweees/can-motor-control.git "$temporary/source"
  source_dir="$temporary/source"
  git -C "$source_dir" checkout 20572f3a41dd98d692da44556de0a0d10c2c8d9d
fi
if [[ "$(git -C "$source_dir" rev-parse HEAD)" != "20572f3a41dd98d692da44556de0a0d10c2c8d9d" ]]; then
  echo "Expected the tested anvil parity commit 20572f3" >&2
  exit 1
fi
wheel_dir="$(mktemp -d)"
trap 'rm -rf "$wheel_dir"; [[ -z "$temporary" ]] || rm -rf "$temporary"' EXIT
uvx maturin build --release --manifest-path "$source_dir/crates/can-motor-control-py/Cargo.toml" --out "$wheel_dir"
uv pip install --python "$checkout/.venv/bin/python" --no-deps "$wheel_dir"/*.whl
"$checkout/.venv/bin/python" -c 'import importlib.metadata; assert importlib.metadata.version("can-motor-control")=="0.1.0"'

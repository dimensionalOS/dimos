#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
package_demo="$PWD/build/message-codegen/external-app"
mkdir -p "$package_demo/local" "$package_demo/evidence"
cp -R examples/message-codegen/demo_msgs "$package_demo/local/"
.venv/bin/python - "$package_demo/local/demo_msgs/msg/Telemetry.msg" <<'PY'
from pathlib import Path
import sys
path = Path(sys.argv[1])
path.write_text(path.read_text() + '\nstring application_note "added-locally"\n')
PY
.venv/bin/python -m dimos.message_codegen.generate \
  --package-root "$package_demo/local" --type demo_msgs/msg/Telemetry \
  --python-module external_telemetry --package --output "$package_demo/generated"
export CMAKE_PREFIX_PATH="$PWD/build/message-codegen/install"
uv build --no-build-isolation --python .venv/bin/python \
  "$package_demo/generated/python" --out-dir "$package_demo/dist"
uv venv --clear --python .venv/bin/python "$package_demo/venv"
uv pip install --offline --no-deps --python "$package_demo/venv/bin/python" "$package_demo"/dist/*.whl
cmake -S "$package_demo/generated/cpp" -B "$package_demo/cpp-package" \
  -DDIMOS_BUILD_PYTHON=OFF -DCMAKE_INSTALL_PREFIX="$package_demo/install"
cmake --build "$package_demo/cpp-package"
cmake --install "$package_demo/cpp-package"
.venv/bin/python -m dimos.message_codegen.generate --type geometry_msgs/msg/Point \
  --output "$package_demo/standard"
cmake -S "$package_demo/standard/cpp" -B "$package_demo/standard-build" \
  -DDIMOS_BUILD_PYTHON=OFF -DCMAKE_INSTALL_PREFIX="$package_demo/install"
cmake --install "$package_demo/standard-build"
cmake -S examples/message-codegen/external-app -B "$package_demo/cpp-consumer" \
  -DCMAKE_PREFIX_PATH="$package_demo/install;$CMAKE_PREFIX_PATH"
cmake --build "$package_demo/cpp-consumer" --parallel 2
cargo package --offline --manifest-path "$package_demo/generated/rust/Cargo.toml" --allow-dirty
mkdir -p "$package_demo/rust-crate" "$package_demo/rust-consumer/src"
tar -xzf "$package_demo/generated/rust/target/package/external-telemetry-messages-0.1.0.crate" \
  -C "$package_demo/rust-crate"
cp examples/message-codegen/external-app/relay.rs "$package_demo/rust-consumer/src/main.rs"
cat > "$package_demo/rust-consumer/Cargo.toml" <<MANIFEST
[package]
name = "external-consumer"
version = "0.1.0"
edition = "2024"
[workspace]
[dependencies]
external-telemetry-messages = { path = "../rust-crate/external-telemetry-messages-0.1.0" }
MANIFEST
cargo build --offline --manifest-path "$package_demo/rust-consumer/Cargo.toml"
# The installed application receives neither PYTHONPATH nor the DimOS checkout.
env -u PYTHONPATH "$package_demo/venv/bin/python" -I \
  examples/message-codegen/external-app/demo_installed.py --build "$package_demo" \
  | tee "$package_demo/evidence/installed-relay.txt"
cat > "$package_demo/typed_consumer.py" <<'PY'
from external_telemetry.demo_msgs.msg import Telemetry

value = Telemetry(application_note="typed")
result: str = value.application_note
PY
.venv/bin/python -m mypy --strict --config-file=/dev/null \
  --python-executable "$package_demo/venv/bin/python" "$package_demo/typed_consumer.py" \
  | tee "$package_demo/evidence/installed-typing.txt"

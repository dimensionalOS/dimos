#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
# The supported external-project example uses normal package/SDK builds.
# The original bounded Telemetry fixture remains covered by CDR-L09 xfails.
python="${DIMOS_RUNTIME_PYTHON:-$PWD/.venv/bin/python}"
project="$PWD/examples/message-project"
"$python" scripts/package_native_sdk.py --output "$project/sdk"
"$python" -c 'from dimos.cli.dimos import cli_main; cli_main()' build --project "$project"
"$python" -m pip install --no-deps "$project"/dist/story_messages-*.whl
(cd "$project/cpp" && cmake --preset dimos && cmake --build --preset dimos)
cargo build --locked --manifest-path "$project/rust/Cargo.toml"
"$python" "$project/demo_blueprint.py" --transport zenoh

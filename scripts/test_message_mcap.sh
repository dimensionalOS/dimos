#!/usr/bin/env bash
set -euo pipefail

# Supported custom DeviceReading, separately from the unchanged bounded Telemetry fixture.
cd "$(dirname "$0")/.."
output="$PWD/build/message-codegen/viewers"
mkdir -p "$output/evidence"
python="${DIMOS_RUNTIME_PYTHON:-${DIMOS_CODEGEN_PYTHON:-$PWD/.venv/bin/python}}"
"$python" - <<'PYTHON'
from importlib.util import find_spec
from pathlib import Path
from dimos.message_codegen.generate import generate
from dimos.message_codegen.ownership import Dependency
spec = find_spec("dimos_generated_schemas")
assert spec is not None and spec.origin is not None
builtin = Dependency.load(Path(spec.origin).parent / "package")
generate([Path("examples/message-codegen/user-story")], Path("build/message-codegen/viewer-message"),
         ["story_msgs/msg/DeviceReading"], "story_messages", dependencies=(builtin,), languages=("python",))
PYTHON
"$python" -m pip install --no-deps --no-build-isolation --target "$output/site" --upgrade \
  build/message-codegen/viewer-message/python
export PYTHONPATH="$output/site:$PWD${PYTHONPATH:+:$PYTHONPATH}"
"$python" -m pytest dimos/protocol/test_cdr_mcap.py --noconftest -o addopts='' -q \
  | tee "$output/evidence/pytest.txt"
"$python" examples/message-codegen/demo_mcap.py --output "$output/demo.mcap" \
  | tee "$output/evidence/producer.txt"
if [[ ! -d examples/message-codegen/viewer-checker/node_modules ]]; then
  npm ci --ignore-scripts --prefix examples/message-codegen/viewer-checker
fi
npm ls --prefix examples/message-codegen/viewer-checker --depth=0
node examples/message-codegen/viewer-checker/check.mjs "$output/demo.mcap" \
  | tee "$output/evidence/foxglove.txt"
# The native Rerun executable has no Python message-provider imports.
unset PYTHONPATH
export RERUN_ANALYTICS_ENABLED=false
"${DIMOS_RERUN_BIN:-$(dirname "$python")/rerun}" mcap convert "$output/demo.mcap" \
  -d ros2msg -d ros2_reflection --disable-raw-fallback -o "$output/demo.rrd"
"${DIMOS_RERUN_BIN:-$(dirname "$python")/rerun}" rrd print "$output/demo.rrd" -v > "$output/evidence/rerun-summary.txt"
"${DIMOS_RERUN_BIN:-$(dirname "$python")/rerun}" rrd print "$output/demo.rrd" --entity /telemetry -vvv \
  > "$output/evidence/rerun-telemetry.txt"
"$python" examples/message-codegen/check_rerun_mcap.py "$output/evidence"

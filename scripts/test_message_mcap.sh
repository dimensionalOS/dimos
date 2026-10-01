#!/usr/bin/env bash
set -euo pipefail

# Run after test_message_codegen.sh. Viewers consume only the resulting MCAP.
cd "$(dirname "$0")/.."
output="$PWD/build/message-codegen/viewers"
mkdir -p "$output/evidence"
export PYTHONPATH="$PWD:$PWD/build/message-codegen/demo/cpp/build${PYTHONPATH:+:$PYTHONPATH}"
.venv/bin/pytest dimos/protocol/test_cdr_mcap.py --noconftest -o addopts='' -q \
  | tee "$output/evidence/pytest.txt"
.venv/bin/python examples/message-codegen/demo_mcap.py --output "$output/demo.mcap" \
  | tee "$output/evidence/producer.txt"
npm ci --ignore-scripts --prefix examples/message-codegen/viewer-checker
node examples/message-codegen/viewer-checker/check.mjs "$output/demo.mcap" \
  | tee "$output/evidence/foxglove.txt"
# The native Rerun executable has no Python message-provider imports.
unset PYTHONPATH
export RERUN_ANALYTICS_ENABLED=false
.venv/bin/rerun mcap convert "$output/demo.mcap" \
  -d ros2msg -d ros2_reflection -o "$output/demo.rrd"
.venv/bin/rerun rrd print "$output/demo.rrd" -v > "$output/evidence/rerun-summary.txt"
.venv/bin/rerun rrd print "$output/demo.rrd" --entity /telemetry -vvv \
  > "$output/evidence/rerun-telemetry.txt"
.venv/bin/python examples/message-codegen/check_rerun_mcap.py "$output/evidence"

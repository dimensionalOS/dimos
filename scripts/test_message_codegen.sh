#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
python="${DIMOS_CODEGEN_PYTHON:-$PWD/.venv/bin/python}"
evidence="$PWD/build/message-codegen/demo/evidence"
mkdir -p "$evidence"
# Bounded Telemetry's unchanged generation assertions are precise strict xfails
# (CDR-L09), not substituted unbounded definitions. Valid native paths run below.
"$python" -m pytest --noconftest -o addopts='' -q -ra \
  dimos/message_codegen/test_definitions.py dimos/message_codegen/test_native.py \
  dimos/message_codegen/test_python_source.py dimos/message_codegen/test_packages.py \
  dimos/message_codegen/test_project.py dimos/message_codegen/test_ownership.py \
  dimos/message_codegen/test_native_libraries.py dimos/message_codegen/test_stubs.py \
  | tee "$evidence/pytest.txt"
DIMOS_NATIVE_ACCEPTANCE=1 DIMOS_OWNERSHIP_EVIDENCE="$evidence" \
  "$python" -m pytest --noconftest -o addopts='' -q \
  dimos/message_codegen/test_ownership_native.py | tee "$evidence/ownership.txt"

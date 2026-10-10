#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
python="${DIMOS_CODEGEN_PYTHON:-$PWD/.venv/bin/python}"
evidence="$PWD/build/message-codegen/demo/evidence"
mkdir -p "$evidence"
"$python" -m pytest --noconftest -o addopts='' -q -ra \
  --cov=dimos/message_codegen --cov-report="xml:$evidence/coverage.xml" \
  dimos/message_codegen/test_definitions.py dimos/message_codegen/test_ownership.py \
  dimos/message_codegen/test_native_libraries.py dimos/message_codegen/test_native_portability.py \
  dimos/message_codegen/test_stubs.py dimos/message_codegen/_vendor/parser_tests \
  | tee "$evidence/pytest.txt"
DIMOS_NATIVE_ACCEPTANCE=1 DIMOS_OWNERSHIP_EVIDENCE="$evidence" \
  "$python" -m pytest --noconftest -o addopts='' -q \
  dimos/message_codegen/test_native_consumer.py dimos/message_codegen/test_region_native.py | tee "$evidence/ownership.txt"

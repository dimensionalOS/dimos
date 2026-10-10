#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
bash scripts/test_builtin_message_source.sh
"${DIMOS_CODEGEN_PYTHON:-$PWD/.venv/bin/python}" -m pytest --noconftest -o addopts='' -q \
  dimos/message_codegen/test_packages.py dimos/message_codegen/test_native.py \
  dimos/message_codegen/test_python_source.py
bash scripts/package_messages.sh

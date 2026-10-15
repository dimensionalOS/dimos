#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
"${DIMOS_CODEGEN_PYTHON:-$PWD/.venv/bin/python}" -m pytest --noconftest -o addopts='' -q \
  dimos/message_codegen/test_builtin_codec.py dimos/message_codegen/test_builtin_catalog.py dimos/message_codegen/test_builtin_alignment.py \
  dimos/message_codegen/test_builtin_install.py dimos/message_codegen/test_builtin_preparation.py

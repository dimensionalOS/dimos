#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
: "${DIMOS_MESSAGE_WHEELHOUSE:?Prepare matching message wheels and offline build dependencies first}"
python="${DIMOS_CODEGEN_PYTHON:-$PWD/.venv/bin/python}"
"$python" -m pytest --noconftest -o addopts='' -q \
  dimos/message_codegen/test_project_install.py \
  dimos/message_codegen/test_repository_tutorial.py \
  dimos/message_codegen/test_documented_project.py
bash scripts/package_messages.sh
DIMOS_BUILTIN_PACKAGE="$PWD/build/message-codegen/release" \
  "$python" -m pytest --noconftest -o addopts='' -q \
  dimos/message_codegen/test_builtin_artifacts.py

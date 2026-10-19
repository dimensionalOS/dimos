#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
uv build --wheel --out-dir build/message-codegen/dist packages/dimos-generated
bash scripts/test_message_packages.sh

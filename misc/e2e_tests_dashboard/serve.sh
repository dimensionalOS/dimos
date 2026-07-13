#!/usr/bin/env bash
# Serve the e2e-tests dashboard from the repo root (so source-file links under
# /dimos/e2e_tests/ resolve). Usage: ./serve.sh [port]
set -euo pipefail
port="${1:-8643}"
cd "$(dirname "$0")/../.."
echo "e2e-tests dashboard: http://127.0.0.1:${port}/misc/e2e_tests_dashboard/"
exec python3 -m http.server "${port}" --bind 127.0.0.1

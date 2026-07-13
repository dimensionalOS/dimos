#!/usr/bin/env bash
# Serve the skills dashboard from the repo root (so /floorplan.dxf, source
# files, and data/ folders all resolve). Usage: ./serve.sh [port]
set -euo pipefail
port="${1:-8642}"
cd "$(dirname "$0")/../.."
echo "Skills dashboard: http://127.0.0.1:${port}/misc/skills_dashboard/"
exec python3 -m http.server "${port}" --bind 127.0.0.1

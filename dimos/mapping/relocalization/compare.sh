#!/usr/bin/env bash
# Runs the global (baseline, every attempt = full FPFH+RANSAC+ICP search) and
# tracking (our method, mode-switched) relocalization benchmarks back-to-back
# against the same recording, then builds a comparison table + plots.
#
# Usage:
#   ./compare.sh [recording] [map_file] [max_attempts]
#   ./compare.sh go2_hongkong_office go2_hongkong_office_twopass_map 15

set -euo pipefail

RECORDING="${1:-go2_hongkong_office}"
MAP_FILE="${2:-go2_hongkong_office_twopass_map}"
MAX_ATTEMPTS="${3:-15}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
OUT_DIR="/tmp/reloc_comparison_$(date +%s)"
mkdir -p "$OUT_DIR"

GLOBAL_LOG="$OUT_DIR/global.log"
TRACKING_LOG="$OUT_DIR/tracking.log"

cd "$REPO_ROOT"
export PATH="$HOME/.local/bin:$PATH"

echo "=== [1/3] running GLOBAL (baseline, every attempt = full search) ==="
PYTHONUNBUFFERED=1 uv run python -m dimos.mapping.relocalization.eval "$RECORDING" \
  --map-file "$MAP_FILE" --mode global --max-attempts "$MAX_ATTEMPTS" \
  2>&1 | tee "$GLOBAL_LOG"

echo
echo "=== [2/3] running TRACKING (our method) ==="
PYTHONUNBUFFERED=1 uv run python -m dimos.mapping.relocalization.eval "$RECORDING" \
  --map-file "$MAP_FILE" --mode tracking --max-attempts "$MAX_ATTEMPTS" \
  2>&1 | tee "$TRACKING_LOG"

echo
echo "=== [3/3] building comparison table + plots ==="
uv run python -m dimos.mapping.relocalization.plot_comparison \
  --global-log "$GLOBAL_LOG" --tracking-log "$TRACKING_LOG" --out-dir "$OUT_DIR"

echo
echo "logs:  $GLOBAL_LOG"
echo "       $TRACKING_LOG"
echo "plots: $OUT_DIR/*.png"

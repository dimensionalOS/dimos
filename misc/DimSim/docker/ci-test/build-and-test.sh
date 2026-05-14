#!/usr/bin/env bash
set -euo pipefail

# Navigate to Dimensional/ (parent of DimSim/ and dimos/)
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DIMENSIONAL_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

echo "Build context: $DIMENSIONAL_DIR"
echo "Building CI test image..."

docker build \
  -f "$SCRIPT_DIR/Dockerfile" \
  -t dimsim-ci-test \
  "$DIMENSIONAL_DIR"

echo ""
echo "Running integration test..."
docker run --rm \
  --cap-add=NET_ADMIN \
  dimsim-ci-test

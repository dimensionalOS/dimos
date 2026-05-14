#!/usr/bin/env bash
set -euo pipefail

# Creates release artifacts for DimSim distribution.
#   dimsim-core-v{VERSION}.tar.gz  — frontend bundle (no scenes)
#   scene-apt-v{VERSION}.tar.gz    — apt scene (gzipped JSON)

cd "$(dirname "$0")/.."

VERSION=${1:-"0.1.0"}

echo "Packaging DimSim v${VERSION}..."

# Verify dist/ exists
if [ ! -f "dist/index.html" ]; then
  echo "Error: dist/ not found. Run 'npm run build' first."
  exit 1
fi

# Core: everything in dist/ except sims/
echo "Packaging core assets..."
tar -czf "dimsim-core-v${VERSION}.tar.gz" \
  -C dist \
  --exclude='sims' \
  .

echo "Packaging apt scene..."
gzip -c dist/sims/apt.json > "scene-apt-v${VERSION}.tar.gz"

echo "Packaging evals..."
tar -czf "dimsim-evals-v${VERSION}.tar.gz" \
  -C evals \
  .

echo ""
echo "Release artifacts:"
ls -lh "dimsim-core-v${VERSION}.tar.gz" "scene-apt-v${VERSION}.tar.gz" "dimsim-evals-v${VERSION}.tar.gz"

echo ""
echo "Upload to GitHub Release:"
echo "  gh release create v${VERSION} dimsim-core-v${VERSION}.tar.gz scene-apt-v${VERSION}.tar.gz dimsim-evals-v${VERSION}.tar.gz"

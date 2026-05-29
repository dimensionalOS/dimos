#!/usr/bin/env bash
# Environment prep for the LIO autoresearch experiment. Run once before the
# experiment loop. Builds the Point-LIO substrate. Python deps (numpy,
# matplotlib) and the dimos package (for get_data) come from the dimos venv —
# this package no longer carries its own uv env. Run inside `nix develop` (or
# with the dimos .venv active) so cmake/eigen/pcl/yaml-cpp/boost are present.
set -euo pipefail
cd "$(dirname "$0")"

# --- build tools / native deps come from the nix dev shell ---
echo ">> checking build tools..."
command -v cmake >/dev/null || { echo "ERROR: cmake not found (run inside 'nix develop')"; exit 1; }

# --- dimos venv: numpy/matplotlib + dimos.get_data all live here ---
echo ">> checking dimos venv"
if ! python -c "import dimos, numpy, matplotlib" 2>/dev/null; then
  root=$(git rev-parse --show-toplevel)
  # shellcheck disable=SC1091
  [ -f "$root/.venv/bin/activate" ] && . "$root/.venv/bin/activate"
fi
python -c "import dimos, numpy, matplotlib" || {
  echo "ERROR: dimos venv not active. Activate the dimos .venv or run inside 'nix develop'."; exit 1; }

# --- build the Point-LIO substrate (fixed; not edited by the agent) ---
echo ">> building point_lio"
cmake -S point_lio -B point_lio/build -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build point_lio/build -j"$(nproc 2>/dev/null || echo 4)"

# --- sanity check (pulls the data via get_data on first run) ---
echo ">> data + harness check"
python prepare.py
echo ">> setup done. Run a baseline with:  python train.py > run.log 2>&1"

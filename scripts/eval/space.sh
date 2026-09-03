#!/usr/bin/env bash
# Fetch Apple's SPACE benchmark for dimos/evals/suites/space_*.py.
#
# Neither the code nor the data is redistributed with dimos: the code is under
# the Apple Sample Code License and the dataset under CC BY-NC-ND 4.0. This
# script only downloads them to a local cache.
#
#   https://github.com/apple/ml-space-benchmark
#
# Override the destinations with DIMOS_SPACE_REPO / DIMOS_SPACE_DATA_DIR.
set -euo pipefail

CACHE="${XDG_CACHE_HOME:-$HOME/.cache}/dimos/space"
REPO="${DIMOS_SPACE_REPO:-$CACHE/ml-space-benchmark}"
DATA_DIR="${DIMOS_SPACE_DATA_DIR:-$CACHE/SPACE_data_release}"
TARBALL_URL="https://ml-site.cdn-apple.com/datasets/space/space.tar.gz"
# The revision every score in the eval report was produced against. Scoring
# comes from this checkout unmodified, so the pin is what makes runs comparable.
SPACE_REV="${DIMOS_SPACE_REV:-564e43932adc84543800dd56b99cee37efaeabd8}"

mkdir -p "$CACHE"

if [ -d "$REPO/.git" ]; then
    echo "benchmark checkout already present: $REPO"
else
    echo "cloning ml-space-benchmark -> $REPO"
    git clone https://github.com/apple/ml-space-benchmark "$REPO"
fi
git -C "$REPO" checkout --quiet "$SPACE_REV"

if [ -d "$DATA_DIR" ]; then
    echo "dataset already present: $DATA_DIR"
else
    # ~3.4 GB, streamed straight into tar so the tarball is never stored.
    # Staged in a temp dir and moved on success: a half-extracted tree left
    # behind by an interrupted transfer would look complete to the check above.
    echo "downloading SPACE dataset (~3.4 GB) -> $DATA_DIR"
    mkdir -p "$(dirname "$DATA_DIR")"
    staging="$(mktemp -d "$(dirname "$DATA_DIR")/.space-download-XXXXXX")"
    trap 'rm -rf "$staging"' EXIT
    curl -fL "$TARBALL_URL" | tar -xz -C "$staging"
    mv "$staging/SPACE_data_release" "$DATA_DIR"
fi

echo
echo "done. install the parser's imports with:  pip install 'dimos[space]'"
echo "then run:  dimos evals run dimos.evals.suites.space.distance --tags bevimage --limit 20 --system-prompt ''"

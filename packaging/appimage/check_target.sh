#!/usr/bin/env bash
# Checks the image tree on its target. build.sh runs this in an ubuntu:26.04
# container that has the documented packages and no network. The tree is at /app.

set -euo pipefail

# Libraries of optional features, which the target does not have: the NVIDIA driver,
# TensorRT and RDMA networking.
readonly OPTIONAL='lib(cuda|nvinfer|nvonnxparser|mlx5|rdmacm|ibverbs)\.so'

export LD_LIBRARY_PATH=/app/usr/lib
# A library that is somewhere in the tree is loaded by the package that brings it,
# whatever ldd says about one file on its own.
find /app \( -type f -o -type l \) -name '*.so*' -printf '%f\n' | sort -u > /tmp/in-the-tree
missing="$(find /app -type f \( -name '*.so' -o -name '*.so.*' -o -executable \) -print0 |
  while IFS= read -r -d '' file; do
    [[ "$(head -c 4 "$file" | tr -d '\0')" == $'\x7fELF' ]] || continue
    { ldd "$file" 2>/dev/null || true; } | awk -v file="$file" '/not found/ {print $1 "\t" file}'
  done | awk -F'\t' 'NR == FNR {have[$1]; next} !($1 in have)' /tmp/in-the-tree - |
  { grep -v -E "^$OPTIONAL" || true; })"
if [[ -n "$missing" ]]; then
  echo "These binaries need a library that the target does not have:"
  echo "$missing"
  exit 1
fi
echo "every binary finds its libraries"

/app/AppRun --help > /dev/null
/app/AppRun list > /dev/null
echo "dimos --help and dimos list run"
# Starts the relay from the installed wheel and fetches the cockpit from it.
/app/AppRun bundle-exec python3 /wheel_smoke.py

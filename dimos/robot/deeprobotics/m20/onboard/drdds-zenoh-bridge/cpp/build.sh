#!/usr/bin/env bash
# Build the bidirectional DrDDS/Zenoh bridge on an M20 AArch64 computer.
set -euo pipefail

bridge_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cmake_args=(
  -S "$bridge_dir"
  -B "$bridge_dir/build"
  -DCMAKE_BUILD_TYPE=Release
)
if [[ -n "${DIMOS_LCM_DIR:-}" ]]; then
  cmake_args+=("-DDIMOS_LCM_DIR=${DIMOS_LCM_DIR}")
elif [[ -d /tmp/dimos-lcm ]]; then
  cmake_args+=("-DDIMOS_LCM_DIR=/tmp/dimos-lcm")
fi

cmake "${cmake_args[@]}"
cmake --build "$bridge_dir/build" --parallel "${M20_BUILD_JOBS:-4}"
echo "built: $bridge_dir/build/m20_drdds_zenoh_bridge"

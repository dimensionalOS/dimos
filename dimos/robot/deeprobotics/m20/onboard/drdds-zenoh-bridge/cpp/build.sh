#!/usr/bin/env bash
# Build the bidirectional DrDDS/Zenoh bridge on an M20 AArch64 computer.
set -euo pipefail

bridge_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cmake_args=(
  -S "$bridge_dir"
  -B "$bridge_dir/build"
  -DCMAKE_BUILD_TYPE=Release
)
if [[ -z "${DIMOS_CDR_TOOLCHAIN:-}" ]]; then
  echo "Set DIMOS_CDR_TOOLCHAIN to the prepared generated-message toolchain.cmake" >&2
  exit 1
fi
cmake_args+=("-DCMAKE_TOOLCHAIN_FILE=${DIMOS_CDR_TOOLCHAIN}")

cmake "${cmake_args[@]}"
cmake --build "$bridge_dir/build" --parallel "${M20_BUILD_JOBS:-4}"
echo "built: $bridge_dir/build/m20_drdds_zenoh_bridge"

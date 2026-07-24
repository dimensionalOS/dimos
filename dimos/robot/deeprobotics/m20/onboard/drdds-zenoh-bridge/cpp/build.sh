#!/usr/bin/env bash
# Build the drdds->zenoh bridge. Must run ON a robot box (aarch64 + DeepRobotics
# SDK under /usr/local + zenoh-c installed). The M20 boxes have no clean internet,
# so if a local dimos-lcm checkout is staged at /tmp/dimos-lcm we feed it to
# FetchContent instead of cloning from GitHub.
set -euo pipefail
cd "$(dirname "$0")"

EXTRA=()
if [ -d /tmp/dimos-lcm ]; then
  EXTRA+=("-DFETCHCONTENT_SOURCE_DIR_DIMOS_LCM=/tmp/dimos-lcm")
fi

cmake -B build -S . "${EXTRA[@]}"
cmake --build build -j
echo "built: $(pwd)/build/m20_drdds_zenoh_bridge"

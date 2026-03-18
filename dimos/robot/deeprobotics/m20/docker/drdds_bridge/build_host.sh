#!/bin/bash
# Build drdds_recv on NOS host.
# Requires: cmake, g++ (installed on NOS)
# Links against: /usr/local/lib/libdrdds.so, libfastrtps.so.2.14

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build_host"
INSTALL_DIR="/opt/drdds_bridge"

echo "=== Building drdds_recv on host ==="
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

cmake "${SCRIPT_DIR}" \
    -DBUILD_DRDDS_RECV=ON \
    -DBUILD_ROS2_PUB=OFF \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}" \
    -DCMAKE_BUILD_TYPE=Release

make -j2
sudo mkdir -p "${INSTALL_DIR}/lib/drdds_bridge"
sudo cp drdds_recv "${INSTALL_DIR}/lib/drdds_bridge/"

echo "=== drdds_recv installed to ${INSTALL_DIR}/lib/drdds_bridge/drdds_recv ==="
echo "Run with: LD_LIBRARY_PATH=/usr/local/lib ${INSTALL_DIR}/lib/drdds_bridge/drdds_recv"

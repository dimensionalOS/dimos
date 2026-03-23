#!/bin/bash
#
# Build the nav_cmd_pub pybind11 module on NOS.
#
# Compiles against the host's libdrdds.so (FastDDS 2.14) and the host's
# glibc (2.31). No rclpy or ROS2 Humble dependency.
#
# Prerequisites:
#   - g++ 9.3.0+ (installed on NOS)
#   - pybind11 in the dimos venv: uv pip install pybind11
#   - drdds headers at /usr/local/include/drdds/
#   - libdrdds.so at /usr/local/lib/
#
# Output: nav_cmd_pub*.so in the dimos venv site-packages

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="${SCRIPT_DIR}/src/nav_cmd_publisher.cpp"

# Detect venv
VENV="${DIMOS_VENV:-${HOME}/dimos-venv}"
if [ ! -f "${VENV}/bin/activate" ]; then
    echo "ERROR: dimos venv not found at ${VENV}"
    echo "Run: uv venv --python 3.10 ${VENV}"
    exit 1
fi
source "${VENV}/bin/activate"

# Ensure pybind11 is installed
python3 -c "import pybind11" 2>/dev/null || {
    echo "Installing pybind11..."
    pip install pybind11
}

# Get compiler flags
PY_INCLUDES=$(python3 -m pybind11 --includes)
PY_EXT_SUFFIX=$(python3 -c "import sysconfig; print(sysconfig.get_config_var('EXT_SUFFIX'))")
SITE_PACKAGES=$(python3 -c "import site; print(site.getsitepackages()[0])")

OUTPUT="${SITE_PACKAGES}/nav_cmd_pub${PY_EXT_SUFFIX}"

echo "=== Building nav_cmd_pub ==="
echo "Source:  ${SRC}"
echo "Output:  ${OUTPUT}"
echo "Python:  $(python3 --version)"
echo "Includes: ${PY_INCLUDES}"

# Include paths: /usr/local/include (drdds core) + /usr/local/include/dridl (IDL types
# including builtin_interfaces/msg/Time.h) + all IDL subdirectories
IDL_INCLUDES=$(find /usr/local/include/dridl -mindepth 1 -maxdepth 1 -type d -exec echo -n "-I{}/msg " \;)

g++ -O2 -shared -std=c++17 -fPIC \
    ${PY_INCLUDES} \
    -I/usr/local/include \
    -I/usr/local/include/dridl \
    ${IDL_INCLUDES} \
    "${SRC}" \
    -o "${OUTPUT}" \
    -L/usr/local/lib \
    -ldrdds -lfastrtps -lfastcdr \
    -Wl,-rpath,/usr/local/lib

echo ""
echo "=== Verification ==="
python3 -c "
import nav_cmd_pub
print(f'Module loaded: {nav_cmd_pub.__doc__}')
print('OK')
" && echo "BUILD PASS" || { echo "BUILD FAIL"; exit 1; }

echo ""
echo "=== Build complete ==="
echo "Module installed at: ${OUTPUT}"

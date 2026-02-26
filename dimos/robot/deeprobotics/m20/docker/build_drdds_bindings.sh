#!/bin/bash
#
# Rebuild the drdds ROS2 package from .msg definitions for the container's
# Python/ROS2 versions.
#
# The vendor Humble image ships pre-compiled drdds libs that may be
# ABI-incompatible with the container's ROS2 Humble packages (different
# rosidl_runtime_c, FastRTPS, or FastCDR versions).  This script uses
# colcon + the rosidl generators already installed in the container to
# rebuild everything from the .msg files, guaranteeing ABI compatibility.
#
# Must be run inside the Docker container where:
#   - ROS2 Humble is sourced (/opt/ros/humble/setup.bash)
#   - colcon, cmake, gcc, python3, numpy are available
#
# Vendor .msg files are probed at:
#   /opt/ros/humble-vendor/share/drdds/msg/  (mounted from host)
#   /opt/ros/humble/share/drdds/msg/         (inside container)
#   /root-ro/opt/ros/humble/share/drdds/msg/ (host read-only partition)

set -e

echo "=== Building drdds from .msg definitions ==="

# --- Discover vendor .msg path ---

VENDOR_MSG=""
for c in /opt/ros/humble-vendor/share/drdds/msg \
         /opt/ros/humble/share/drdds/msg \
         /root-ro/opt/ros/humble/share/drdds/msg \
         /opt/dimos/docker/drdds_msgs/msg; do
    [ -f "${c}/NavCmd.msg" ] && { VENDOR_MSG="${c}"; break; }
done
[ -z "${VENDOR_MSG}" ] && { echo "ERROR: Cannot find drdds .msg files"; exit 1; }
echo "Vendor .msg files: ${VENDOR_MSG}"

# --- Create colcon workspace ---

WSDIR=$(mktemp -d /tmp/drdds-ws.XXXXXX)
PKG="${WSDIR}/src/drdds"
mkdir -p "${PKG}/msg"

echo "Workspace: ${WSDIR}"

# Copy all .msg files
cp "${VENDOR_MSG}"/*.msg "${PKG}/msg/"
N_MSG=$(ls "${PKG}/msg/"*.msg | wc -l)
echo "Copied ${N_MSG} .msg files"

# Generate the .msg file list for CMakeLists.txt
MSG_LIST=""
for f in "${PKG}"/msg/*.msg; do
    MSG_LIST="${MSG_LIST}  \"msg/$(basename ${f})\"\n"
done

# --- Create package.xml ---

cat > "${PKG}/package.xml" << 'PKGXML'
<?xml version="1.0"?>
<package format="3">
  <name>drdds</name>
  <version>0.0.0</version>
  <description>DeepRobotics DDS message definitions (rebuilt for container)</description>
  <maintainer email="dev@dimos.ai">dimos</maintainer>
  <license>Proprietary</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
PKGXML

# --- Create CMakeLists.txt ---

cat > "${PKG}/CMakeLists.txt" << CMAKELISTS
cmake_minimum_required(VERSION 3.8)
project(drdds)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(\${PROJECT_NAME}
$(echo -e "${MSG_LIST}")
)

ament_package()
CMAKELISTS

echo ""
echo "=== Building with colcon ==="

cd "${WSDIR}"
# Limit parallel jobs to avoid OOM on resource-constrained NOS
colcon build \
    --packages-select drdds \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 2 \
    2>&1

echo ""
echo "=== Installing to /opt/drdds ==="

# The colcon install tree has everything we need
INSTALL_SRC="${WSDIR}/install/drdds"
INSTALL_DST="/opt/drdds"

# Copy libraries
mkdir -p "${INSTALL_DST}/lib"
cp -a "${INSTALL_SRC}/lib"/lib*.so* "${INSTALL_DST}/lib/" 2>/dev/null || true

# Copy Python packages (includes extension modules)
mkdir -p "${INSTALL_DST}/lib/python3"
if [ -d "${INSTALL_SRC}/local/lib/python3.10/dist-packages/drdds" ]; then
    cp -a "${INSTALL_SRC}/local/lib/python3.10/dist-packages" "${INSTALL_DST}/lib/python3/site-packages" 2>/dev/null || \
    cp -a "${INSTALL_SRC}/local/lib/python3.10/dist-packages/drdds" "${INSTALL_DST}/lib/python3/site-packages/drdds"
elif [ -d "${INSTALL_SRC}/lib/python3.10/site-packages/drdds" ]; then
    cp -a "${INSTALL_SRC}/lib/python3.10/site-packages/drdds" "${INSTALL_DST}/lib/python3/site-packages/drdds"
else
    echo "WARNING: Could not find Python packages in colcon install tree"
    echo "Install tree contents:"
    find "${INSTALL_SRC}" -name "*.py" -o -name "*.so" | head -20
fi

# Copy share (msg definitions, cmake config)
mkdir -p "${INSTALL_DST}/share"
cp -a "${INSTALL_SRC}/share/drdds" "${INSTALL_DST}/share/" 2>/dev/null || true

# Copy include headers
mkdir -p "${INSTALL_DST}/include"
cp -a "${INSTALL_SRC}/include/"* "${INSTALL_DST}/include/" 2>/dev/null || true

# ldconfig
if [ -d /etc/ld.so.conf.d ]; then
    echo "${INSTALL_DST}/lib" > /etc/ld.so.conf.d/drdds.conf
    ldconfig 2>/dev/null || true
fi

echo ""
echo "=== Verification ==="
export PYTHONPATH="${INSTALL_DST}/lib/python3/site-packages:${PYTHONPATH}"
export LD_LIBRARY_PATH="${INSTALL_DST}/lib:${LD_LIBRARY_PATH}"

python3 -c "
from drdds.msg import NavCmd
n = NavCmd()
field = 'meta' if hasattr(n, 'meta') else 'header'
print(f'NavCmd OK (header field: {field})')
n.data.x_vel = 0.1
print(f'NavCmd.data.x_vel = {n.data.x_vel}')
" && echo "NavCmd import: PASS" || { echo "NavCmd import: FAIL"; exit 1; }

python3 -c "
try:
    from drdds.msg import MotionInfo
    print('MotionInfo (Foxy)')
except ImportError:
    from drdds.msg import MotionStatus
    print('MotionStatus (Humble)')
" && echo "Motion type: PASS" || echo "Motion type: FAIL (non-critical)"

# DDS publisher test (verifies type support ABI compatibility)
python3 -c "
import rclpy
rclpy.init()
node = rclpy.create_node('drdds_build_test')
from drdds.msg import NavCmd
pub = node.create_publisher(NavCmd, '/DRDDS_BUILD_TEST', 10)
msg = NavCmd()
msg.data.x_vel = 0.1
pub.publish(msg)
print('DDS publisher: OK')
node.destroy_node()
rclpy.shutdown()
" && echo "DDS publisher: PASS" || { echo "DDS publisher: FAIL"; exit 1; }

# Cleanup
rm -rf "${WSDIR}"

echo ""
echo "=== Build complete ==="
echo "PYTHONPATH: ${INSTALL_DST}/lib/python3/site-packages"
echo "LD_LIBRARY_PATH: ${INSTALL_DST}/lib"

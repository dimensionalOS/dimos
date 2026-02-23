#!/bin/bash
#
# Deploy dimos M20 mac_bridge to NOS (Navigation Operating System)
#
# Usage:
#   ./deploy_nos.sh [hostname] [user]              # Full dimos deploy
#   ./deploy_nos.sh --mac-bridge [hostname] [user]  # Mac bridge only (fast)
#
#   hostname: SSH target for NOS. Use jump host syntax for NAT'd access.
#             Default: direct SSH via jump host (user@10.21.31.106 via 10.21.41.1)
#   user:     SSH user on target (default: user)
#
# Prerequisites:
#   - SSH access to NOS (via AOS jump host or Tailscale)
#   - Python 3.8 on NOS (comes with Ubuntu 20.04)
#   - ROS 2 Foxy installed at /opt/ros/foxy
#   - drdds-ros2-msgs package installed
#
# What this does:
#   Mac bridge deploy (--mac-bridge, default):
#     1. Copies mac_bridge.py to /opt/dimos/ on NOS
#     2. Installs systemd service (dimos-mac-bridge)
#     3. Starts the bridge service
#
# Network path: Mac → AOS WiFi (10.21.41.1:9731) → DNAT → NOS (10.21.31.106:9731)
# NOS DDS: eth0 (10.21.33.106) shares L2 with AOS eth0 (10.21.33.103)
#
# Reference: M20-SETUP.md, INVESTIGATION_LOG.md Session 10

set -e

# Parse --mac-bridge flag
MAC_BRIDGE=false
POSITIONAL=()
for arg in "$@"; do
    case $arg in
        --mac-bridge)
            MAC_BRIDGE=true
            shift
            ;;
        *)
            POSITIONAL+=("$arg")
            ;;
    esac
done
set -- "${POSITIONAL[@]}"

# Configuration
NOS_HOST="${1:-10.21.31.106}"
NOS_USER="${2:-user}"
JUMP_HOST="${JUMP_HOST:-user@10.21.41.1}"
DEPLOY_DIR="/opt/dimos"
VENV_DIR="${DEPLOY_DIR}/venv"
SSH_OPTS="-o ProxyJump=${JUMP_HOST}"

# Backwards compat aliases
GOS_HOST="${NOS_HOST}"
GOS_USER="${NOS_USER}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# dimos root is 4 levels up from m20/ directory
DIMOS_ROOT="$(cd "${SCRIPT_DIR}/../../../../.." && pwd)"

if [ "$MAC_BRIDGE" = true ]; then
    echo "=== M20 Mac Bridge Deploy (NOS) ==="
else
    echo "=== dimos M20 NOS Deployment ==="
fi
echo "Target: ${NOS_USER}@${NOS_HOST} (via ${JUMP_HOST})"
echo "Deploy dir: ${DEPLOY_DIR}"
echo ""

# Check SSH connection
echo "Checking SSH connection..."
if ! ssh ${SSH_OPTS} -o ConnectTimeout=10 "${NOS_USER}@${NOS_HOST}" "echo 'Connected'" 2>/dev/null; then
    echo "ERROR: Cannot connect to ${NOS_HOST} via ${JUMP_HOST}"
    echo "Make sure:"
    echo "  - You're on the M20 WiFi network"
    echo "  - AOS (${JUMP_HOST}) is reachable"
    echo "  - NOS (${NOS_HOST}) is online"
    echo "  - You have SSH access as '${NOS_USER}'"
    exit 1
fi
echo "Connection OK"

# Prompt for sudo password
if [ -z "${SUDO_PASS}" ]; then
    read -sp "Sudo password for ${NOS_USER}@${NOS_HOST}: " SUDO_PASS
    echo ""
fi

remote_sudo() {
    printf '%s\n' "${SUDO_PASS}" | ssh ${SSH_OPTS} "${NOS_USER}@${NOS_HOST}" "sudo -S $*" 2>&1 | { grep -v '^\[sudo\] password' || true; }
}

remote_ssh() {
    ssh ${SSH_OPTS} "${NOS_USER}@${NOS_HOST}" "$@"
}

# Verify sudo
if ! printf '%s\n' "${SUDO_PASS}" | ssh ${SSH_OPTS} "${NOS_USER}@${NOS_HOST}" "sudo -S true" 2>/dev/null; then
    echo "ERROR: Invalid sudo password"
    exit 1
fi

# --- Mac Bridge deploy ---

if [ "$MAC_BRIDGE" = true ]; then
    # Check ROS 2 Foxy installation
    echo "Checking ROS 2 Foxy..."
    remote_ssh << 'CHECK_ROS'
set -e
if [ ! -d /opt/ros/foxy ]; then
    echo "ERROR: /opt/ros/foxy not found"
    exit 1
fi
PYTHONPATH=/opt/ros/foxy/lib/python3.8/site-packages LD_LIBRARY_PATH=/opt/ros/foxy/lib:/opt/drdds/lib python3.8 -c "import rclpy; print('rclpy OK')"
PYTHONPATH=/opt/ros/foxy/lib/python3.8/site-packages:/opt/drdds/lib/python3.8/site-packages LD_LIBRARY_PATH=/opt/ros/foxy/lib:/opt/drdds/lib python3.8 -c "from drdds.msg import NavCmd, MotionInfo; print('drdds OK')"
echo "ROS 2 Foxy + drdds: OK"
CHECK_ROS

    # Create deploy directory
    remote_sudo mkdir -p ${DEPLOY_DIR}

    # Stop existing service
    echo "Stopping existing bridge service (if running)..."
    remote_sudo systemctl stop dimos-mac-bridge 2>/dev/null || true

    # Copy mac_bridge.py (atomic: scp to .tmp, then mv)
    echo "Deploying mac_bridge.py..."
    scp ${SSH_OPTS} "${SCRIPT_DIR}/mac_bridge.py" "${NOS_USER}@${NOS_HOST}:/tmp/mac_bridge.py.tmp"
    remote_sudo cp /tmp/mac_bridge.py.tmp ${DEPLOY_DIR}/mac_bridge.py
    remote_ssh "rm -f /tmp/mac_bridge.py.tmp"

    # Install systemd service
    echo "Installing systemd service..."
    cat << 'SERVICE_EOF' | remote_ssh "cat > /tmp/dimos-mac-bridge.service"
[Unit]
Description=dimos Mac Bridge (ROS2 → TCP)
After=network-online.target
StartLimitIntervalSec=300
StartLimitBurst=5

[Service]
Type=simple
ExecStart=/bin/bash -c 'source /opt/ros/foxy/setup.bash && /usr/bin/python3 /opt/dimos/mac_bridge.py --port 9731'
Restart=on-failure
RestartSec=3
Environment=LD_LIBRARY_PATH=/opt/ros/foxy/lib:/opt/drdds/lib
Environment=PYTHONPATH=/opt/ros/foxy/lib/python3.8/site-packages:/opt/drdds/lib/python3.8/site-packages
Environment=ROS_DOMAIN_ID=0
Environment=RMW_IMPLEMENTATION=rmw_fastrtps_cpp
StandardOutput=journal
StandardError=journal
User=root

[Install]
WantedBy=multi-user.target
SERVICE_EOF
    remote_sudo mv /tmp/dimos-mac-bridge.service /etc/systemd/system/dimos-mac-bridge.service
    remote_sudo systemctl daemon-reload
    remote_sudo systemctl enable dimos-mac-bridge

    # Start service
    echo "Starting Mac bridge service..."
    remote_sudo systemctl start dimos-mac-bridge

    # Verify
    sleep 3
    echo ""
    echo "Service status:"
    remote_ssh "systemctl is-active dimos-mac-bridge && echo 'Bridge is RUNNING' || echo 'Bridge FAILED to start'"
    echo ""
    echo "Recent logs:"
    remote_ssh "journalctl -u dimos-mac-bridge -n 10 --no-pager" 2>/dev/null || true

    echo ""
    echo "=== Mac Bridge Deployment Complete (NOS) ==="
    echo ""
    echo "Bridge is listening on NOS port 9731"
    echo "Mac connects to: bridge_host=\"10.21.41.1\" (AOS WiFi, NAT → NOS)"
    echo ""
    echo "Useful commands:"
    echo "  Restart:  ssh ${SSH_OPTS} ${NOS_USER}@${NOS_HOST} 'echo \"'\"'\"'\" | sudo -S systemctl restart dimos-mac-bridge'"
    echo "  Logs:     ssh ${SSH_OPTS} ${NOS_USER}@${NOS_HOST} 'journalctl -u dimos-mac-bridge -f'"
    echo "  Status:   ssh ${SSH_OPTS} ${NOS_USER}@${NOS_HOST} 'systemctl status dimos-mac-bridge'"
    exit 0
fi

# --- Full dimos deploy (legacy, not recommended for NOS) ---

echo "Source: ${DIMOS_ROOT}"
echo ""
echo "WARNING: Full deploy syncs the entire dimos codebase to NOS."
echo "         For most use cases, use --mac-bridge instead."
echo ""

# Check ROS 2 Foxy installation
echo "Checking ROS 2 Foxy..."
remote_ssh << 'CHECK_ROS'
set -e
if [ ! -d /opt/ros/foxy ]; then
    echo "ERROR: /opt/ros/foxy not found"
    exit 1
fi
PYTHONPATH=/opt/ros/foxy/lib/python3.8/site-packages LD_LIBRARY_PATH=/opt/ros/foxy/lib:/opt/drdds/lib python3.8 -c "import rclpy; print('rclpy OK')"
echo "ROS 2 Foxy: OK"
CHECK_ROS

# Check drdds-ros2-msgs
echo "Checking drdds-ros2-msgs..."
remote_ssh << 'CHECK_DRDDS'
PYTHONPATH=/opt/ros/foxy/lib/python3.8/site-packages:/opt/drdds/lib/python3.8/site-packages LD_LIBRARY_PATH=/opt/ros/foxy/lib:/opt/drdds/lib python3.8 -c "from drdds.msg import NavCmd, MotionInfo; print('drdds OK')" 2>/dev/null || echo "WARNING: drdds-ros2-msgs not found — /NAV_CMD will not be available"
CHECK_DRDDS

# Create deploy directory
echo "Creating deploy directory..."
remote_sudo mkdir -p ${DEPLOY_DIR}
remote_sudo chown -R ${NOS_USER}:${NOS_USER} ${DEPLOY_DIR}

# Sync dimos source
echo "Syncing dimos source..."
rsync -avz --delete \
    -e "ssh ${SSH_OPTS}" \
    --exclude '__pycache__' \
    --exclude '*.pyc' \
    --exclude '.git' \
    --exclude 'venv' \
    --exclude '.pytest_cache' \
    --exclude '.mypy_cache' \
    --exclude '*.egg-info' \
    "${DIMOS_ROOT}/" "${NOS_USER}@${NOS_HOST}:${DEPLOY_DIR}/src/"

# Set up venv and install
echo "Setting up Python environment..."
remote_ssh << REMOTE_SETUP
set -e

# Create venv with system-site-packages (for rclpy + drdds access)
if [ ! -d "${VENV_DIR}" ]; then
    echo "Creating virtual environment with system-site-packages..."
    python3.8 -m venv --system-site-packages ${VENV_DIR}
fi

# Ensure system-site-packages is enabled
if grep -q 'include-system-site-packages = false' ${VENV_DIR}/pyvenv.cfg 2>/dev/null; then
    echo "Enabling system site-packages..."
    sed -i 's/include-system-site-packages = false/include-system-site-packages = true/' ${VENV_DIR}/pyvenv.cfg
fi

# Install dimos
source ${VENV_DIR}/bin/activate
export LD_LIBRARY_PATH=/opt/ros/foxy/lib:/opt/drdds/lib:\${LD_LIBRARY_PATH}
pip install --upgrade pip
pip install -e "${DEPLOY_DIR}/src/"

# Verify
python3 -c "
import dimos
from dimos.robot.deeprobotics.m20.ros_sensors import M20ROSSensors
print(f'dimos installed: {dimos.__version__ if hasattr(dimos, \"__version__\") else \"dev\"}')
print('M20ROSSensors: OK')
"

echo "Python environment ready"
REMOTE_SETUP

echo ""
echo "=== Deployment Complete (NOS) ==="
echo ""
echo "IMPORTANT — Before running dimos with /NAV_CMD velocity:"
echo "  The built-in planner service on AOS may conflict with dimos velocity"
echo "  commands (dev guide 2.3.1). To disable it on AOS:"
echo "    ssh user@10.21.41.1 'echo \"'\"'\"'\" | sudo -S systemctl stop planner.service'"
echo ""
echo "Run dimos on NOS:"
echo "  ssh ${SSH_OPTS} ${NOS_USER}@${NOS_HOST}"
echo "  source ${VENV_DIR}/bin/activate"
echo "  export LD_LIBRARY_PATH=/opt/ros/foxy/lib:/opt/drdds/lib:\$LD_LIBRARY_PATH"
echo "  python3 -m dimos.robot.deeprobotics.m20  # or your entrypoint"
echo ""
echo "Useful commands:"
echo "  Check ROS topics:  ssh ${SSH_OPTS} ${NOS_USER}@${NOS_HOST} 'source /opt/ros/foxy/setup.bash && ros2 topic list'"
echo "  Check /ODOM:       ssh ${SSH_OPTS} ${NOS_USER}@${NOS_HOST} 'source /opt/ros/foxy/setup.bash && ros2 topic echo /ODOM'"

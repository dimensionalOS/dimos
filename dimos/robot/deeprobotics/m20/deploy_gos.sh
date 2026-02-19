#!/bin/bash
#
# Deploy dimos M20 integration to GOS (General Operating System)
#
# Usage:
#   ./deploy_gos.sh [hostname] [user]              # Full dimos deploy
#   ./deploy_gos.sh --mac-bridge [hostname] [user]  # Mac bridge only (fast)
#
#   hostname: Tailscale hostname or IP of the M20 GOS (default: m20-770-gogo)
#   user:     SSH user on GOS (default: user)
#
# Prerequisites:
#   - SSH access to GOS via Tailscale
#   - Python 3.8 on GOS (comes with Ubuntu 20.04)
#   - ROS 2 Foxy installed at /opt/ros/foxy
#   - drdds-ros2-msgs package installed
#
# What this does:
#   Full deploy:
#     1. Syncs dimos source to /opt/dimos on GOS
#     2. Creates venv with --system-site-packages (rclpy + drdds access)
#     3. Installs dimos into venv
#     4. Optionally installs a systemd service
#   Mac bridge deploy (--mac-bridge):
#     1. Copies mac_bridge.py to /opt/dimos/ on GOS
#     2. Installs systemd service (dimos-mac-bridge)
#     3. Starts the bridge service
#     4. Cleans up legacy 3.8 compat artifacts
#
# Reference: M20-SETUP.md, relay deploy/deploy.sh

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
GOS_HOST="${1:-m20-770-gogo}"
GOS_USER="${2:-user}"
DEPLOY_DIR="/opt/dimos"
VENV_DIR="${DEPLOY_DIR}/venv"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# dimos root is 4 levels up from m20/ directory
DIMOS_ROOT="$(cd "${SCRIPT_DIR}/../../../../.." && pwd)"

if [ "$MAC_BRIDGE" = true ]; then
    echo "=== M20 Mac Bridge Deploy ==="
else
    echo "=== dimos M20 GOS Deployment ==="
fi
echo "Target: ${GOS_USER}@${GOS_HOST}"
echo "Deploy dir: ${DEPLOY_DIR}"
echo ""

# Check SSH connection
echo "Checking SSH connection..."
if ! ssh -o ConnectTimeout=5 "${GOS_USER}@${GOS_HOST}" "echo 'Connected'" 2>/dev/null; then
    echo "ERROR: Cannot connect to ${GOS_HOST}"
    echo "Make sure:"
    echo "  - Tailscale is running"
    echo "  - The M20 GOS is online"
    echo "  - You have SSH access as '${GOS_USER}'"
    exit 1
fi
echo "Connection OK"

# Prompt for sudo password
if [ -z "${SUDO_PASS}" ]; then
    read -sp "Sudo password for ${GOS_USER}@${GOS_HOST}: " SUDO_PASS
    echo ""
fi

remote_sudo() {
    printf '%s\n' "${SUDO_PASS}" | ssh "${GOS_USER}@${GOS_HOST}" "sudo -S $*" 2>&1 | { grep -v '^\[sudo\] password' || true; }
}

# Verify sudo
if ! printf '%s\n' "${SUDO_PASS}" | ssh "${GOS_USER}@${GOS_HOST}" "sudo -S true" 2>/dev/null; then
    echo "ERROR: Invalid sudo password"
    exit 1
fi

# --- Mac Bridge deploy ---

if [ "$MAC_BRIDGE" = true ]; then
    # Check ROS 2 Foxy installation
    echo "Checking ROS 2 Foxy..."
    ssh "${GOS_USER}@${GOS_HOST}" << 'CHECK_ROS'
set -e
if [ ! -d /opt/ros/foxy ]; then
    echo "ERROR: /opt/ros/foxy not found"
    exit 1
fi
PYTHONPATH=/opt/ros/foxy/lib/python3.8/site-packages LD_LIBRARY_PATH=/opt/ros/foxy/lib python3.8 -c "import rclpy; print('rclpy OK')"
echo "ROS 2 Foxy: OK"
CHECK_ROS

    # Create deploy directory
    remote_sudo mkdir -p ${DEPLOY_DIR}
    remote_sudo chown -R ${GOS_USER}:${GOS_USER} ${DEPLOY_DIR}

    # Stop existing service
    echo "Stopping existing bridge service (if running)..."
    remote_sudo systemctl stop dimos-mac-bridge 2>/dev/null || true

    # Copy mac_bridge.py (atomic: scp to .tmp, then mv)
    echo "Deploying mac_bridge.py..."
    scp "${SCRIPT_DIR}/mac_bridge.py" "${GOS_USER}@${GOS_HOST}:${DEPLOY_DIR}/mac_bridge.py.tmp"
    ssh "${GOS_USER}@${GOS_HOST}" "mv ${DEPLOY_DIR}/mac_bridge.py.tmp ${DEPLOY_DIR}/mac_bridge.py"

    # Install systemd service
    echo "Installing systemd service..."
    cat << 'SERVICE_EOF' | ssh "${GOS_USER}@${GOS_HOST}" "cat > /tmp/dimos-mac-bridge.service"
[Unit]
Description=dimos Mac Bridge (ROS2 → TCP)
After=network-online.target

[Service]
Type=simple
ExecStart=/usr/bin/python3 /opt/dimos/mac_bridge.py --port 9731
Restart=on-failure
RestartSec=3
StartLimitIntervalSec=300
StartLimitBurst=5
Environment=LD_LIBRARY_PATH=/opt/ros/foxy/lib:/opt/drdds/lib
Environment=PYTHONPATH=/opt/ros/foxy/lib/python3.8/site-packages:/opt/drdds/lib/python3.8/site-packages
Environment=ROS_DOMAIN_ID=0
StandardOutput=journal
StandardError=journal
User=user

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
    ssh "${GOS_USER}@${GOS_HOST}" "systemctl is-active dimos-mac-bridge && echo 'Bridge is RUNNING' || echo 'Bridge FAILED to start'"
    echo ""
    echo "Recent logs:"
    ssh "${GOS_USER}@${GOS_HOST}" "journalctl -u dimos-mac-bridge -n 10 --no-pager" 2>/dev/null || true

    # Clean up legacy 3.8 compat artifacts
    echo ""
    echo "Cleaning up legacy artifacts..."
    ssh "${GOS_USER}@${GOS_HOST}" "rm -rf ${DEPLOY_DIR}/stubs ${DEPLOY_DIR}/venv ${DEPLOY_DIR}/src" 2>/dev/null || true
    echo "Legacy cleanup done"

    echo ""
    echo "=== Mac Bridge Deployment Complete ==="
    echo ""
    echo "Bridge is listening on port 9731"
    echo "Connect from Mac with: bridge_host=\"${GOS_HOST}\""
    echo ""
    echo "Useful commands:"
    echo "  Restart:  ssh ${GOS_USER}@${GOS_HOST} 'sudo systemctl restart dimos-mac-bridge'"
    echo "  Logs:     ssh ${GOS_USER}@${GOS_HOST} 'journalctl -u dimos-mac-bridge -f'"
    echo "  Status:   ssh ${GOS_USER}@${GOS_HOST} 'systemctl status dimos-mac-bridge'"
    exit 0
fi

# --- Full dimos deploy (legacy) ---

echo "Source: ${DIMOS_ROOT}"
echo ""

# Check ROS 2 Foxy installation
echo "Checking ROS 2 Foxy..."
ssh "${GOS_USER}@${GOS_HOST}" << 'CHECK_ROS'
set -e
if [ ! -d /opt/ros/foxy ]; then
    echo "ERROR: /opt/ros/foxy not found"
    exit 1
fi
PYTHONPATH=/opt/ros/foxy/lib/python3.8/site-packages LD_LIBRARY_PATH=/opt/ros/foxy/lib python3.8 -c "import rclpy; print('rclpy OK')"
echo "ROS 2 Foxy: OK"
CHECK_ROS

# Check drdds-ros2-msgs
echo "Checking drdds-ros2-msgs..."
ssh "${GOS_USER}@${GOS_HOST}" << 'CHECK_DRDDS'
PYTHONPATH=/opt/ros/foxy/lib/python3.8/site-packages LD_LIBRARY_PATH=/opt/ros/foxy/lib python3.8 -c "from drdds.msg import NavCmd, MotionInfo; print('drdds OK')" 2>/dev/null || echo "WARNING: drdds-ros2-msgs not found — /NAV_CMD will not be available"
CHECK_DRDDS

# Create deploy directory
echo "Creating deploy directory..."
remote_sudo mkdir -p ${DEPLOY_DIR}
remote_sudo chown -R ${GOS_USER}:${GOS_USER} ${DEPLOY_DIR}

# Sync dimos source
echo "Syncing dimos source..."
rsync -avz --delete \
    --exclude '__pycache__' \
    --exclude '*.pyc' \
    --exclude '.git' \
    --exclude 'venv' \
    --exclude '.pytest_cache' \
    --exclude '.mypy_cache' \
    --exclude '*.egg-info' \
    "${DIMOS_ROOT}/" "${GOS_USER}@${GOS_HOST}:${DEPLOY_DIR}/src/"

# Set up venv and install
echo "Setting up Python environment..."
ssh "${GOS_USER}@${GOS_HOST}" << REMOTE_SETUP
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
export LD_LIBRARY_PATH=/opt/ros/foxy/lib:\${LD_LIBRARY_PATH}
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
echo "=== Deployment Complete ==="
echo ""
echo "IMPORTANT — Before running dimos with /NAV_CMD velocity:"
echo "  The built-in planner service on AOS may conflict with dimos velocity"
echo "  commands (dev guide 2.3.1). To disable it on AOS (10.21.31.103):"
echo "    sudo systemctl stop planner.service"
echo "    sudo systemctl disable planner.service  # persist across reboots"
echo "  Also disable charge_manager if not using autonomous charging:"
echo "    sudo systemctl stop charge_manager.service"
echo ""
echo "Run dimos on GOS:"
echo "  ssh ${GOS_USER}@${GOS_HOST}"
echo "  source ${VENV_DIR}/bin/activate"
echo "  export LD_LIBRARY_PATH=/opt/ros/foxy/lib:\$LD_LIBRARY_PATH"
echo "  python3 -m dimos.robot.deeprobotics.m20  # or your entrypoint"
echo ""
echo "Useful commands:"
echo "  Check ROS topics:  ssh ${GOS_USER}@${GOS_HOST} 'source /opt/ros/foxy/setup.bash && ros2 topic list'"
echo "  Check /NAV_CMD:    ssh ${GOS_USER}@${GOS_HOST} 'source /opt/ros/foxy/setup.bash && ros2 topic info /NAV_CMD'"
echo "  Check /ODOM:       ssh ${GOS_USER}@${GOS_HOST} 'source /opt/ros/foxy/setup.bash && ros2 topic echo /ODOM'"

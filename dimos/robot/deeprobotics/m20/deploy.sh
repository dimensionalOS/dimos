#!/bin/bash
#
# M20 NOS deployment helper
#
# Syncs code, sets up NOS prerequisites, and manages the nav stack.
# Everything runs natively via nix — no Docker needed.
#
# Prerequisites:
#   SSH keys copied to NOS and AOS (ssh-copy-id)
#   NOS user has passwordless sudo (or set SUDO_PASS env var)
#
# Usage:
#   ./deploy.sh sync [--host <hostname>]          # Sync dimos source to NOS
#   ./deploy.sh setup [--host <hostname>]         # NOS prerequisites (after reboot)
#   ./deploy.sh start [--host <hostname>]         # Start smartnav on NOS
#   ./deploy.sh stop [--host <hostname>]          # Stop smartnav
#   ./deploy.sh restart [--host <hostname>]       # Stop + start
#   ./deploy.sh bridge-start [--host <hostname>]  # Start drdds SHM bridge
#   ./deploy.sh bridge-stop [--host <hostname>]   # Stop bridge
#   ./deploy.sh status [--host <hostname>]        # Show status
#   ./deploy.sh viewer [--host <hostname>]        # Start tunnels + dimos-viewer
#
# Quick deploy after code changes:
#   ./deploy.sh sync --host m20-770-gogo && ./deploy.sh restart --host m20-770-gogo
#
# Full deploy after reboot:
#   ./deploy.sh setup --host m20-770-gogo
#   ./deploy.sh bridge-start --host m20-770-gogo
#   ./deploy.sh start --host m20-770-gogo
#   ./deploy.sh viewer --host m20-770-gogo
#
# Options:
#   --host <hostname>   Access robot via Tailscale (e.g., m20-770-gogo)
#                       Without --host: uses AOS WiFi (10.21.41.1)

set -e

CMD="${1:-help}"
shift 2>/dev/null || true

REMOTE_HOST=""
positional=()
while [ $# -gt 0 ]; do
    case "$1" in
        --host) REMOTE_HOST="$2"; shift 2 ;;
        *) positional+=("$1"); shift ;;
    esac
done
set -- "${positional[@]}"

NOS_HOST="${1:-10.21.31.106}"
NOS_USER="${2:-user}"
AOS_INTERNAL="10.21.31.103"
DEPLOY_DIR="/var/opt/robot/data/dimos"
VENV="/home/user/dimos-venv"
SMARTNAV_MODULE="dimos.robot.deeprobotics.m20.blueprints.nav.m20_smartnav_native"
SMARTNAV_LOG="/tmp/smartnav_native.log"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIMOS_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"

if [ -n "${REMOTE_HOST}" ]; then
    JUMP_HOST="${NOS_USER}@${REMOTE_HOST}"
else
    JUMP_HOST="${JUMP_HOST:-user@10.21.41.1}"
fi

SSH_OPTS="-o ProxyJump=${JUMP_HOST}"

remote_ssh() {
    ssh ${SSH_OPTS} "${NOS_USER}@${NOS_HOST}" "$@"
}

remote_sudo() {
    if [ -n "${SUDO_PASS}" ]; then
        remote_ssh "echo '${SUDO_PASS}' | sudo -S $*" 2>&1 | grep -v '^\[sudo\]' || true
    else
        remote_ssh "sudo $*"
    fi
}

RSYNC_EXCLUDES=(
    --exclude '__pycache__'
    --exclude '*.pyc'
    --exclude '.git'
    --exclude 'venv'
    --exclude '.venv'
    --exclude '.pytest_cache'
    --exclude '.mypy_cache'
    --exclude '*.egg-info'
    --exclude '.beads'
    --exclude '.runtime'
    --exclude 'node_modules'
    --exclude 'data'
    --exclude 'logs'
    --exclude '.claude'
)

case "${CMD}" in
    sync)
        echo "=== Syncing dimos source to NOS ==="
        rsync -avz --delete \
            -e "ssh ${SSH_OPTS}" \
            "${RSYNC_EXCLUDES[@]}" \
            "${DIMOS_ROOT}/" "${NOS_USER}@${NOS_HOST}:${DEPLOY_DIR}/"
        # Ensure nix binary symlinks exist
        remote_ssh "
            mkdir -p ${DEPLOY_DIR}/dimos/robot/deeprobotics/m20/drdds_bridge/cpp/result/bin
            mkdir -p ${DEPLOY_DIR}/dimos/navigation/smart_nav/modules/arise_slam/result/bin
            mkdir -p ${DEPLOY_DIR}/dimos/navigation/smart_nav/modules/terrain_analysis/result/bin
            mkdir -p ${DEPLOY_DIR}/dimos/navigation/smart_nav/modules/local_planner/result/bin
            mkdir -p ${DEPLOY_DIR}/dimos/navigation/smart_nav/modules/path_follower/result/bin
            for store_path in /nix/store/*-drdds_lidar_bridge-*/bin/drdds_lidar_bridge; do
                [ -f \"\$store_path\" ] && ln -sf \"\$store_path\" ${DEPLOY_DIR}/dimos/robot/deeprobotics/m20/drdds_bridge/cpp/result/bin/drdds_lidar_bridge
            done
            for store_path in /nix/store/*-smartnav-arise-slam-*/bin/arise_slam; do
                [ -f \"\$store_path\" ] && ln -sf \"\$store_path\" ${DEPLOY_DIR}/dimos/navigation/smart_nav/modules/arise_slam/result/bin/arise_slam
            done
            for store_path in /nix/store/*-smartnav-terrain-analysis-*/bin/terrain_analysis; do
                [ -f \"\$store_path\" ] && ln -sf \"\$store_path\" ${DEPLOY_DIR}/dimos/navigation/smart_nav/modules/terrain_analysis/result/bin/terrain_analysis
            done
            for store_path in /nix/store/*-smartnav-local-planner-*/bin/local_planner; do
                [ -f \"\$store_path\" ] && ln -sf \"\$store_path\" ${DEPLOY_DIR}/dimos/navigation/smart_nav/modules/local_planner/result/bin/local_planner
            done
            for store_path in /nix/store/*-smartnav-path-follower-*/bin/path_follower; do
                [ -f \"\$store_path\" ] && ln -sf \"\$store_path\" ${DEPLOY_DIR}/dimos/navigation/smart_nav/modules/path_follower/result/bin/path_follower
            done
            # nav_cmd_pub (built via cmake, not nix)
            [ -f /tmp/drdds_bridge_build/build/nav_cmd_pub ] && \
                ln -sf /tmp/drdds_bridge_build/build/nav_cmd_pub ${DEPLOY_DIR}/dimos/robot/deeprobotics/m20/drdds_bridge/cpp/result/bin/nav_cmd_pub
        "
        echo "=== Sync Complete ==="
        ;;

    setup)
        echo "=== Setting up NOS prerequisites ==="
        remote_sudo ip link set lo multicast on
        remote_sudo ip route add 224.0.0.0/4 dev lo 2>/dev/null || true
        remote_sudo ip route add 10.21.33.103/32 via 10.21.31.103 2>/dev/null || true
        remote_sudo mount --bind /var/opt/robot/data/nix /nix 2>/dev/null || true
        echo "=== Setup Complete ==="
        ;;

    start)
        echo "=== Starting smartnav ==="
        remote_ssh "pkill -f m20_smartnav 2>/dev/null" || true
        sleep 3
        remote_ssh "pkill -9 -f m20_smartnav 2>/dev/null; pkill -9 -f dimos-venv 2>/dev/null" || true
        sleep 1
        remote_ssh "fuser -k 9877/tcp 2>/dev/null; fuser -k 3030/tcp 2>/dev/null; fuser -k 7779/tcp 2>/dev/null" || true
        remote_ssh "cd ${DEPLOY_DIR} && ${VENV}/bin/python -m ${SMARTNAV_MODULE} > ${SMARTNAV_LOG} 2>&1 &"
        echo "  Waiting for startup..."
        sleep 30
        echo "  Status:"
        remote_ssh "grep -a -E 'matched|gRPC|Navigation|error' ${SMARTNAV_LOG} | grep -v sec_nsec | grep -v wireframe | head -5" || true
        echo "=== Started ==="
        ;;

    stop)
        echo "=== Stopping smartnav ==="
        # Graceful shutdown first (sends sit-down command to robot)
        remote_ssh "pkill -f m20_smartnav 2>/dev/null" || true
        sleep 5
        # Force kill if still running
        remote_ssh "pkill -9 -f m20_smartnav 2>/dev/null; pkill -9 -f dimos-venv 2>/dev/null" || true
        sleep 1
        remote_ssh "fuser -k 9877/tcp 2>/dev/null; fuser -k 3030/tcp 2>/dev/null; fuser -k 7779/tcp 2>/dev/null" || true
        echo "=== Stopped ==="
        ;;

    restart)
        if [ -n "${REMOTE_HOST}" ]; then
            "$0" stop --host "${REMOTE_HOST}"
            "$0" start --host "${REMOTE_HOST}"
        else
            "$0" stop
            "$0" start
        fi
        ;;

    viewer)
        echo "=== Starting dimos-viewer ==="
        pkill -f "rerun" 2>/dev/null || true
        pkill -f "ssh.*-L.*9877.*${JUMP_HOST}" 2>/dev/null || true
        sleep 1
        # Set up SSH tunnels
        ssh -f -N -o ConnectTimeout=10 ${SSH_OPTS} \
            -L 9877:${NOS_HOST}:9877 \
            -L 7779:${NOS_HOST}:7779 \
            -L 3030:${NOS_HOST}:3030 \
            "${NOS_USER}@${NOS_HOST}" 2>/dev/null
        sleep 1
        # Launch viewer
        DIMOS_DEBUG=1 "${DIMOS_ROOT}/.venv/bin/rerun" \
            --connect "rerun+http://127.0.0.1:9877/proxy" \
            --ws-url "ws://127.0.0.1:3030/ws" &
        echo "  dimos-viewer PID: $!"
        echo "=== Viewer started ==="
        ;;

    bridge-start)
        echo "=== Starting drdds bridge ==="

        echo "  Stopping rsdriver for discovery ordering..."
        remote_sudo systemctl stop rsdriver 2>/dev/null || true
        sleep 2

        echo "  Cleaning stale SHM segments..."
        remote_sudo rm -f /dev/shm/fastrtps_* /dev/shm/fast_datasharing_* /dev/shm/sem.fastrtps_* /dev/shm/drdds_bridge_* /dev/shm/sem.drdds_bridge_*

        echo "  Starting drdds_recv..."
        remote_sudo "nohup /opt/drdds_bridge/lib/drdds_bridge/drdds_recv > /var/log/drdds_recv.log 2>&1 &"
        sleep 3

        echo "  Restarting rsdriver (30s init delay)..."
        remote_sudo systemctl start rsdriver
        sleep 35

        echo "  Verifying bridge data flow..."
        remote_ssh "tail -3 /var/log/drdds_recv.log 2>/dev/null" || true

        echo "=== Bridge running ==="
        ;;

    bridge-stop)
        echo "Stopping drdds bridge..."
        remote_ssh "pkill -f drdds_recv 2>/dev/null" || true
        echo "Bridge stopped."
        ;;

    status)
        echo "=== drdds bridge ==="
        remote_ssh "pgrep -a drdds_recv 2>/dev/null" || echo "  NOT RUNNING"
        echo ""
        echo "=== dimos processes ==="
        remote_ssh "pgrep -a -f m20_smartnav 2>/dev/null" || echo "  smartnav not running"
        remote_ssh "pgrep -a nav_cmd_pub 2>/dev/null" || echo "  nav_cmd_pub not running"
        remote_ssh "pgrep -a drdds_lidar 2>/dev/null" || echo "  lidar bridge not running"
        remote_ssh "pgrep -a arise_slam 2>/dev/null" || echo "  arise_slam not running"
        echo ""
        echo "=== Ports ==="
        remote_ssh "ss -tlnp 2>/dev/null | grep -E '9877|7779|3030'" || echo "  no ports listening"
        echo ""
        echo "=== AOS services ==="
        remote_ssh "ssh user@${AOS_INTERNAL} 'for s in basic_server ctrlmcu rl_deploy rsdriver; do printf \"  %-15s %s\n\" \$s \$(systemctl is-active \$s); done'" 2>/dev/null || echo "  (could not reach AOS)"
        ;;

    *)
        echo "Usage: $0 <command> [--host <hostname>]"
        echo ""
        echo "Commands:"
        echo "  sync         - Sync dimos source to NOS + fix nix symlinks"
        echo "  setup        - NOS prerequisites (multicast, nix mount, route)"
        echo "  start        - Start smartnav on NOS"
        echo "  stop         - Stop smartnav"
        echo "  restart      - Stop + start"
        echo "  viewer       - Start SSH tunnels + dimos-viewer locally"
        echo "  bridge-start - Start drdds SHM bridge (with rsdriver ordering)"
        echo "  bridge-stop  - Stop drdds bridge"
        echo "  status       - Show status"
        echo ""
        echo "Quick deploy:  $0 sync --host m20-770-gogo && $0 restart --host m20-770-gogo"
        echo "After reboot:  $0 setup && $0 bridge-start && $0 start && $0 viewer"
        echo ""
        echo "Options:"
        echo "  --host <hostname>  Access robot via Tailscale (e.g., m20-770-gogo)"
        exit 1
        ;;
esac

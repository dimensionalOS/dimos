#!/bin/bash
#
# Deploy dimos Docker container to M20 NOS
#
# Usage:
#   ./deploy.sh build     # Sync code + build Docker image on NOS
#   ./deploy.sh start     # Start container (with lio health check)
#   ./deploy.sh stop      # Stop container
#   ./deploy.sh logs      # Tail container logs
#   ./deploy.sh shell     # Open shell in running container
#   ./deploy.sh status    # Show container + lio status
#
# Prerequisites:
#   - SSH access to NOS via AOS jump host
#   - DeepRobotics Humble image loaded and tagged as dimos/m20-humble-base:latest
#   - Docker + docker compose installed on NOS
#
# Network: Mac → AOS WiFi (10.21.41.1) → DNAT → NOS (10.21.31.106)

set -e

# Configuration
NOS_HOST="${1:-10.21.31.106}"
NOS_USER="${2:-user}"
JUMP_HOST="${JUMP_HOST:-user@10.21.41.1}"
AOS_HOST="${JUMP_HOST}"
SSH_OPTS="-o ProxyJump=${JUMP_HOST}"
DEPLOY_DIR="/opt/dimos/src"
CONTAINER_NAME="dimos-m20"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIMOS_ROOT="$(cd "${SCRIPT_DIR}/../../../../.." && pwd)"

# Shift past positional args to get command
CMD="${1:-help}"
shift 2>/dev/null || true

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

aos_sudo() {
    printf '%s\n' "${SUDO_PASS}" | ssh "${AOS_HOST}" "sudo -S $*" 2>&1 | { grep -v '^\[sudo\] password' || true; }
}

aos_ssh() {
    ssh "${AOS_HOST}" "$@"
}

lio_publishing_data() {
    # Verify lio publishes /ODOM — check from NOS (shared L2 with AOS)
    remote_ssh "source /opt/ros/foxy/setup.bash && \
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && \
        timeout 5 ros2 topic echo /ODOM --once" >/dev/null 2>&1
}

ensure_lio_enabled() {
    echo "Ensuring lio_perception is enabled on AOS..."

    # Fast path: already running, enable finished, and publishing data
    if aos_ssh "pgrep -f lio_ddsnode" >/dev/null 2>&1 && \
       ! aos_ssh "pgrep -f 'lio_command 1'" >/dev/null 2>&1; then
        echo "  lio_ddsnode running, lio_command finished — checking data flow..."
        if lio_publishing_data; then
            echo "  /ODOM data confirmed — lio is healthy"
            return 0
        fi
        echo "  No /ODOM data — restarting..."
    elif aos_ssh "pgrep -f 'lio_command 1'" >/dev/null 2>&1; then
        echo "  lio_command still in progress — waiting..."
    else
        echo "  lio_ddsnode not running — starting lio_perception..."
    fi

    aos_sudo systemctl restart lio_perception

    local max_wait=45
    local elapsed=0
    while [ $elapsed -lt $max_wait ]; do
        sleep 3
        elapsed=$((elapsed + 3))
        if aos_ssh "pgrep -f 'lio_command 1'" >/dev/null 2>&1; then
            echo "  Waiting for lio_command... (${elapsed}s)"
            continue
        fi
        if lio_publishing_data; then
            echo "  /ODOM data confirmed (${elapsed}s) — lio is healthy"
            return 0
        fi
        echo "  lio_command finished but no /ODOM data yet... (${elapsed}s)"
    done

    echo "  WARNING: lio not confirmed healthy after ${max_wait}s"
    return 0
}

ensure_nat() {
    echo "Ensuring AOS NAT rules for web UI + Rerun..."
    for port in 7779 9876; do
        aos_sudo iptables -t nat -C PREROUTING -i wlan0 -p tcp --dport $port \
            -j DNAT --to-destination 10.21.31.106:$port 2>/dev/null || \
        aos_sudo iptables -t nat -A PREROUTING -i wlan0 -p tcp --dport $port \
            -j DNAT --to-destination 10.21.31.106:$port
    done
    echo "  NAT rules OK (7779 → NOS, 9876 → NOS)"
}

check_conflicts() {
    echo "Checking for service conflicts..."
    # height_map_nav on AOS conflicts with dimos /NAV_CMD
    if aos_ssh "systemctl is-active height_map_nav" 2>/dev/null | grep -q active; then
        echo "  WARNING: height_map_nav is active on AOS — will conflict with /NAV_CMD"
        echo "  Run: ssh ${AOS_HOST} 'echo \"'\" | sudo -S systemctl stop height_map_nav'"
    fi
    # planner on NOS conflicts with dimos velocity commands
    if remote_ssh "systemctl is-active planner" 2>/dev/null | grep -q active; then
        echo "  WARNING: planner is active on NOS — will conflict with /NAV_CMD"
        echo "  Run: ssh ${SSH_OPTS} ${NOS_USER}@${NOS_HOST} 'echo \"'\" | sudo -S systemctl stop planner'"
    fi
    # Stop mac_bridge if running (we're replacing it)
    if remote_ssh "systemctl is-active dimos-mac-bridge" 2>/dev/null | grep -q active; then
        echo "  Stopping dimos-mac-bridge (replaced by Docker container)..."
        remote_sudo systemctl stop dimos-mac-bridge
    fi
}

case "${CMD}" in
    build)
        echo "=== dimos M20 NOS Docker Build ==="
        echo "Target: ${NOS_USER}@${NOS_HOST} (via ${JUMP_HOST})"
        echo ""

        # Check SSH
        echo "Checking SSH..."
        remote_ssh "echo 'Connected'" || { echo "ERROR: Cannot connect"; exit 1; }

        # Verify base image exists
        echo "Checking base image..."
        if ! remote_sudo docker images dimos/m20-humble-base:latest --format '{{.ID}}' | grep -q .; then
            echo "ERROR: Base image dimos/m20-humble-base:latest not found on NOS"
            echo "Load it first: sudo docker load -i /tmp/ysc_ros2_backup.tar.gz"
            echo "Then tag it:   sudo docker tag <IMAGE_ID> dimos/m20-humble-base:latest"
            exit 1
        fi

        # Sync source
        echo "Syncing dimos source to NOS..."
        remote_sudo mkdir -p ${DEPLOY_DIR}
        remote_sudo chown -R ${NOS_USER}:${NOS_USER} /opt/dimos
        rsync -avz --delete \
            -e "ssh ${SSH_OPTS}" \
            --exclude '__pycache__' \
            --exclude '*.pyc' \
            --exclude '.git' \
            --exclude 'venv' \
            --exclude '.venv' \
            --exclude '.pytest_cache' \
            --exclude '.mypy_cache' \
            --exclude '*.egg-info' \
            --exclude '.beads' \
            --exclude '.runtime' \
            --exclude 'state.json' \
            "${DIMOS_ROOT}/" "${NOS_USER}@${NOS_HOST}:${DEPLOY_DIR}/"

        # Build image on NOS
        echo "Building Docker image on NOS (this may take a while on first build)..."
        remote_ssh "cd ${DEPLOY_DIR} && sudo docker build \
            -t dimos-m20-nos:latest \
            -f dimos/robot/deeprobotics/m20/docker/Dockerfile \
            ."

        echo ""
        echo "=== Build Complete ==="
        echo "Image: dimos-m20-nos:latest"
        ;;

    start)
        echo "=== Starting dimos M20 Docker Container ==="

        check_conflicts
        ensure_lio_enabled
        ensure_nat

        echo "Starting container..."
        remote_sudo docker run -d \
            --name ${CONTAINER_NAME} \
            --privileged \
            --network host \
            --ipc host \
            -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
            -e ROS_DOMAIN_ID=0 \
            --restart no \
            dimos-m20-nos:latest

        sleep 5
        echo ""
        echo "Container status:"
        remote_sudo docker ps --filter name=${CONTAINER_NAME} --format 'table {{.Status}}\t{{.Names}}'
        echo ""
        echo "Access from Mac:"
        echo "  Web UI:  http://10.21.41.1:7779/command-center"
        echo "  Rerun:   rerun --connect rerun+http://10.21.41.1:9876/proxy"
        ;;

    stop)
        echo "Stopping ${CONTAINER_NAME}..."
        remote_sudo docker stop ${CONTAINER_NAME} 2>/dev/null || true
        remote_sudo docker rm ${CONTAINER_NAME} 2>/dev/null || true
        echo "Stopped."
        ;;

    logs)
        remote_sudo docker logs -f ${CONTAINER_NAME}
        ;;

    shell)
        echo "Opening shell in ${CONTAINER_NAME}..."
        ssh -t ${SSH_OPTS} "${NOS_USER}@${NOS_HOST}" "sudo docker exec -it ${CONTAINER_NAME} /bin/bash"
        ;;

    status)
        echo "=== Container ==="
        remote_sudo docker ps -a --filter name=${CONTAINER_NAME} --format 'table {{.Status}}\t{{.Names}}'
        echo ""
        echo "=== lio (AOS) ==="
        if aos_ssh "pgrep -f lio_ddsnode" >/dev/null 2>&1; then
            echo "  lio_ddsnode: RUNNING"
        else
            echo "  lio_ddsnode: NOT RUNNING"
        fi
        if aos_ssh "pgrep -f 'lio_command 1'" >/dev/null 2>&1; then
            echo "  lio_command: IN PROGRESS"
        else
            echo "  lio_command: finished"
        fi
        echo ""
        echo "=== Services ==="
        echo -n "  height_map_nav (AOS): "
        aos_ssh "systemctl is-active height_map_nav" 2>/dev/null || echo "inactive"
        echo -n "  dimos-mac-bridge (NOS): "
        remote_ssh "systemctl is-active dimos-mac-bridge" 2>/dev/null || echo "inactive"
        ;;

    *)
        echo "Usage: $0 {build|start|stop|logs|shell|status} [hostname] [user]"
        echo ""
        echo "Commands:"
        echo "  build   - Sync code + build Docker image on NOS"
        echo "  start   - Start container (with lio health check + NAT)"
        echo "  stop    - Stop and remove container"
        echo "  logs    - Tail container logs"
        echo "  shell   - Open shell in running container"
        echo "  status  - Show container + service status"
        exit 1
        ;;
esac

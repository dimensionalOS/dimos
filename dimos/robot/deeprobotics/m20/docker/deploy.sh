#!/bin/bash
#
# Deploy dimos Docker container to M20 NOS
#
# Usage:
#   ./deploy.sh push      # Build image on Mac + push to GHCR
#   ./deploy.sh pull      # Pull latest image on NOS from GHCR
#   ./deploy.sh dev       # Sync Python source into running container (fast, ~1-2 min)
#   ./deploy.sh start     # Start container (with lio health check)
#   ./deploy.sh stop      # Stop container
#   ./deploy.sh restart   # Restart running container
#   ./deploy.sh logs      # Tail container logs
#   ./deploy.sh shell     # Open shell in running container
#   ./deploy.sh status    # Show container + lio status
#
# Options:
#   --host <hostname>     # Access robot via Tailscale (e.g., m20-781-mochi)
#                         # Without --host: uses AOS WiFi (10.21.41.1) as gateway
#
# Workflows:
#   Full deploy:  ./deploy.sh push && ./deploy.sh pull && ./deploy.sh start
#   Iterate:      ./deploy.sh dev   (rsync + docker cp + restart, ~1-2 min)
#   Quick restart: ./deploy.sh restart  (no sync, ~10-30 sec)
#
# Remote deploy (via Tailscale):
#   ./deploy.sh push
#   ./deploy.sh pull --host m20-781-mochi
#   ./deploy.sh start --host m20-781-mochi
#
# Use `push+pull` when dependencies, Dockerfile, or C++ extensions change.
# Use `dev` for pure Python changes — 5-10x faster.
#
# Prerequisites:
#   - GHCR auth: docker login ghcr.io (Mac for push, NOS for pull)
#   - SSH access to NOS (via AOS WiFi or Tailscale → GOS)
#   - Docker installed on Mac and NOS
#
# Network modes:
#   Local:  Mac → AOS WiFi (10.21.41.1) → DNAT → NOS (10.21.31.106)
#   Remote: Mac → Tailscale → GOS (10.21.31.104) → DNAT → NOS (10.21.31.106)
# Registry: Mac → internet → GHCR; NOS → GOS 5G → GHCR

set -e

# Parse: first arg is command, then --host flag, then optional positional host/user
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

# Configuration
NOS_HOST="${1:-10.21.31.106}"
NOS_USER="${2:-user}"
AOS_INTERNAL="10.21.31.103"
DEPLOY_DIR="/opt/dimos/src"
CONTAINER_NAME="dimos-m20"
GHCR_IMAGE="ghcr.io/aphexcx/m20-nos:latest"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIMOS_ROOT="$(cd "${SCRIPT_DIR}/../../../../.." && pwd)"

# --- Network mode ---
# Local:  AOS WiFi is gateway (direct connection, NAT on wlan0)
# Remote: GOS via Tailscale is gateway (NAT on tailscale0)
if [ -n "${REMOTE_HOST}" ]; then
    GATEWAY_HOST="${NOS_USER}@${REMOTE_HOST}"
    JUMP_HOST="${GATEWAY_HOST}"
    AOS_HOST="${NOS_USER}@${AOS_INTERNAL}"
    NAT_IFACE="tailscale0"
    ACCESS_HOST="${REMOTE_HOST}"
    REMOTE_MODE=1
else
    JUMP_HOST="${JUMP_HOST:-user@10.21.41.1}"
    GATEWAY_HOST="${JUMP_HOST}"
    AOS_HOST="${JUMP_HOST}"
    NAT_IFACE="wlan0"
    ACCESS_HOST="10.21.41.1"
    REMOTE_MODE=""
fi

# --- SSH ControlMaster multiplexing ---
# Reuses a single TCP connection for all SSH calls, saving ~2-5s per call.
SSH_CONTROL_DIR="/tmp/deploy-ssh-mux"
mkdir -p "${SSH_CONTROL_DIR}"
SSH_MUX_OPTS="-o ControlMaster=auto -o ControlPath=${SSH_CONTROL_DIR}/%r@%h:%p -o ControlPersist=600"

# NOS: always reached via jump host (AOS in local, GOS in remote)
SSH_OPTS="${SSH_MUX_OPTS} -o ProxyJump=${JUMP_HOST}"

# Gateway: direct connection (AOS WiFi in local, GOS Tailscale in remote)
GATEWAY_SSH_OPTS="${SSH_MUX_OPTS}"

# AOS: direct in local mode, via GOS in remote mode
if [ -n "${REMOTE_MODE}" ]; then
    AOS_SSH_OPTS="${SSH_MUX_OPTS} -o ProxyJump=${GATEWAY_HOST}"
else
    AOS_SSH_OPTS="${SSH_MUX_OPTS}"
fi

setup_ssh_mux() {
    # Gateway (GOS via Tailscale or AOS via WiFi)
    ssh -fNM ${GATEWAY_SSH_OPTS} "${GATEWAY_HOST}" 2>/dev/null || true
    # NOS via gateway
    ssh -fNM ${SSH_MUX_OPTS} -o ProxyJump=${JUMP_HOST} "${NOS_USER}@${NOS_HOST}" 2>/dev/null || true
    if [ -n "${REMOTE_MODE}" ]; then
        # AOS via GOS (only needed in remote mode; in local mode AOS=gateway)
        ssh -fNM ${AOS_SSH_OPTS} "${AOS_HOST}" 2>/dev/null || true
    fi
}

# M20 robots all use the same sudo password
SUDO_PASS="${SUDO_PASS:-"'"}"

# Establish SSH multiplexing early (except for push which is local-only)
if [ "${CMD}" != "push" ] && [ "${CMD}" != "help" ]; then
    [ -n "${REMOTE_MODE}" ] && echo "Remote mode: ${REMOTE_HOST} (Tailscale → GOS → NOS)"
    setup_ssh_mux
fi

# --- SSH helper functions ---

# NOS (where Docker container runs)
remote_sudo() {
    printf '%s\n' "${SUDO_PASS}" | ssh ${SSH_OPTS} "${NOS_USER}@${NOS_HOST}" "sudo -S $*" 2>&1 | { grep -v '^\[sudo\] password' || true; }
}

remote_ssh() {
    ssh ${SSH_OPTS} "${NOS_USER}@${NOS_HOST}" "$@"
}

# Gateway (NAT host — GOS in remote, AOS in local)
gateway_sudo() {
    printf '%s\n' "${SUDO_PASS}" | ssh ${GATEWAY_SSH_OPTS} "${GATEWAY_HOST}" "sudo -S $*" 2>&1 | { grep -v '^\[sudo\] password' || true; }
}

gateway_ssh() {
    ssh ${GATEWAY_SSH_OPTS} "${GATEWAY_HOST}" "$@"
}

# AOS (lio_perception, height_map_nav — always on AOS regardless of mode)
aos_sudo() {
    printf '%s\n' "${SUDO_PASS}" | ssh ${AOS_SSH_OPTS} "${AOS_HOST}" "sudo -S $*" 2>&1 | { grep -v '^\[sudo\] password' || true; }
}

aos_ssh() {
    ssh ${AOS_SSH_OPTS} "${AOS_HOST}" "$@"
}

# Common rsync excludes for source sync
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
    --exclude 'state.json'
    --exclude 'data'
    --exclude 'logs'
    --exclude 'docs'
    --exclude 'assets'
    --exclude '.claude'
    --exclude '.github'
    --exclude 'examples'
    --exclude 'mail'
    --exclude 'node_modules'
)

sync_source() {
    echo "Syncing dimos source to NOS..."
    remote_sudo mkdir -p ${DEPLOY_DIR}
    # Only fix ownership if needed (avoid slow recursive chown)
    remote_ssh "[ -O ${DEPLOY_DIR} ]" 2>/dev/null || \
        remote_sudo chown ${NOS_USER}:${NOS_USER} ${DEPLOY_DIR}
    rsync -avz --delete \
        -e "ssh ${SSH_OPTS}" \
        "${RSYNC_EXCLUDES[@]}" \
        "${DIMOS_ROOT}/" "${NOS_USER}@${NOS_HOST}:${DEPLOY_DIR}/"
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
    local gw_label
    if [ -n "${REMOTE_MODE}" ]; then
        gw_label="GOS (${NAT_IFACE})"
    else
        gw_label="AOS (${NAT_IFACE})"
    fi
    echo "Ensuring NAT rules on ${gw_label} for web UI + Rerun..."
    local rules_changed=0

    # DNAT rules: redirect incoming traffic on gateway to NOS
    for port in 7779 9876; do
        if ! gateway_sudo iptables -t nat -C PREROUTING -i ${NAT_IFACE} -p tcp --dport $port \
            -j DNAT --to-destination ${NOS_HOST}:$port 2>/dev/null; then
            gateway_sudo iptables -t nat -A PREROUTING -i ${NAT_IFACE} -p tcp --dport $port \
                -j DNAT --to-destination ${NOS_HOST}:$port
            rules_changed=1
        fi
    done

    # FORWARD rules: allow DNAT'd traffic through (default FORWARD policy is DROP on GOS)
    # Determine which interface NOS is on (eth0 = internal LAN on GOS)
    local nos_iface="eth0"
    if ! gateway_sudo iptables -C FORWARD -i ${NAT_IFACE} -o ${nos_iface} -p tcp \
        -d ${NOS_HOST} -j ACCEPT 2>/dev/null; then
        gateway_sudo iptables -A FORWARD -i ${NAT_IFACE} -o ${nos_iface} -p tcp \
            -d ${NOS_HOST} -j ACCEPT
        gateway_sudo iptables -A FORWARD -i ${nos_iface} -o ${NAT_IFACE} \
            -m state --state RELATED,ESTABLISHED -j ACCEPT
        rules_changed=1
    fi

    # Persist rules across reboots via /etc/iptables.rules (loaded by rc.local)
    if [ $rules_changed -eq 1 ]; then
        gateway_sudo "iptables-save > /etc/iptables.rules"
        echo "  NAT rules saved to /etc/iptables.rules (persistent)"
    fi

    echo "  NAT rules OK (${gw_label}: 7779 → NOS, 9876 → NOS)"
}

check_conflicts() {
    echo "Checking for service conflicts..."
    # height_map_nav on AOS conflicts with dimos /NAV_CMD
    if aos_ssh "systemctl is-active height_map_nav" 2>/dev/null | grep -q active; then
        echo "  Disabling height_map_nav on AOS (conflicts with /NAV_CMD)..."
        aos_sudo systemctl disable --now height_map_nav
    fi
    # planner on NOS conflicts with dimos velocity commands
    if remote_ssh "systemctl is-active planner" 2>/dev/null | grep -q active; then
        echo "  Disabling planner on NOS (conflicts with /NAV_CMD)..."
        remote_sudo systemctl disable --now planner
    fi
    # Stop mac_bridge if running (we're replacing it)
    if remote_ssh "systemctl is-active dimos-mac-bridge" 2>/dev/null | grep -q active; then
        echo "  Stopping dimos-mac-bridge (replaced by Docker container)..."
        remote_sudo systemctl stop dimos-mac-bridge
    fi
    if remote_ssh "pgrep -f mac_bridge" >/dev/null 2>&1; then
        echo "  Killing mac_bridge process (replaced by Docker container)..."
        remote_sudo pkill -f mac_bridge
    fi
    # Kill Foxy ros2 daemon — thrashes at 78% CPU due to Humble DDS discovery
    if remote_ssh "pgrep -f ros2_daemon" >/dev/null 2>&1; then
        echo "  Killing ros2 daemon (Foxy/Humble DDS conflict)..."
        remote_sudo pkill -f ros2_daemon
    fi
    # Stop NOS services not needed by dimos (saves CPU)
    # multicast-relay: relays lidar multicast — dimos uses DDS /ODOM not raw /LIDAR/POINTS
    # yesense: IMU driver — /IMU comes from main controller independently
    # Note: rsdriver must stay running (charge_manager + reflective_column need /LIDAR/POINTS)
    for svc in multicast-relay yesense; do
        if remote_ssh "systemctl is-active ${svc}" 2>/dev/null | grep -q active; then
            echo "  Stopping ${svc} (not needed by dimos)..."
            remote_sudo systemctl stop ${svc}
        fi
    done
}

ensure_clock_synced() {
    echo "Checking NOS clock..."
    local nos_date
    nos_date=$(remote_ssh date +%Y-%m-%d 2>/dev/null) || { echo "  WARNING: Could not check clock"; return 0; }
    local current_date
    current_date=$(date +%Y-%m-%d)

    if [ "$nos_date" != "$current_date" ]; then
        echo "  NOS clock is wrong (${nos_date}, expected ${current_date}) — fixing..."

        # Ensure chrony is installed, running, and synced
        if remote_ssh "which chronyd" >/dev/null 2>&1; then
            remote_sudo systemctl enable --now chrony
            remote_sudo chronyc makestep >/dev/null 2>&1 || true
            sleep 2
        else
            echo "  chrony not installed — installing..."
            remote_sudo apt-get update -qq >/dev/null 2>&1
            remote_sudo apt-get install -y -qq chrony >/dev/null 2>&1
            remote_sudo systemctl enable --now chrony
            remote_sudo chronyc makestep >/dev/null 2>&1 || true
            sleep 2
        fi

        # Verify fix
        nos_date=$(remote_ssh date +%Y-%m-%d 2>/dev/null)
        if [ "$nos_date" = "$current_date" ]; then
            echo "  Clock synced via chrony ($(remote_ssh date 2>/dev/null))"
        else
            # Fallback: set time manually
            echo "  chrony sync failed — setting time manually..."
            remote_sudo date -s "$(date -u '+%Y-%m-%d %H:%M:%S')" >/dev/null 2>&1
            echo "  Clock set to $(remote_ssh date 2>/dev/null)"
        fi
    else
        # Clock date is correct, but ensure chrony is running for ongoing sync
        if remote_ssh "which chronyd" >/dev/null 2>&1; then
            remote_sudo systemctl enable --now chrony 2>/dev/null || true
        fi
        echo "  Clock OK ($(remote_ssh date 2>/dev/null))"
    fi
}

ensure_docker_sequential_downloads() {
    local config="/etc/docker/daemon.json"
    if remote_ssh "cat ${config} 2>/dev/null" | grep -q '"max-concurrent-downloads"'; then
        return 0
    fi

    echo "Setting max-concurrent-downloads=1 for reliable pulls over flaky networks..."
    if remote_ssh "test -s ${config}" 2>/dev/null; then
        # Existing config — merge in the setting (simple json, use python)
        remote_sudo "python3 -c \"
import json
with open('${config}') as f: c = json.load(f)
c['max-concurrent-downloads'] = 1
with open('${config}', 'w') as f: json.dump(c, f, indent=2)
\""
    else
        remote_sudo "echo '{\"max-concurrent-downloads\": 1}' > ${config}"
    fi
    echo "  Restarting Docker daemon to apply..."
    remote_sudo systemctl restart docker
    sleep 3
}

case "${CMD}" in
    push)
        echo "=== Building + Pushing M20 Docker Image ==="
        echo "Image: ${GHCR_IMAGE}"
        echo "Context: ${DIMOS_ROOT}"
        echo ""

        # Build on Mac (ARM64 native — same arch as NOS RK3588)
        # BuildKit required for --mount=type=cache in Dockerfile
        echo "Building Docker image..."
        DOCKER_BUILDKIT=1 docker build \
            -t "${GHCR_IMAGE}" \
            -f "${DIMOS_ROOT}/dimos/robot/deeprobotics/m20/docker/Dockerfile" \
            "${DIMOS_ROOT}"

        echo ""
        echo "Pushing to GHCR..."
        docker push "${GHCR_IMAGE}"

        echo ""
        echo "=== Push Complete ==="
        echo "Deploy to robot: $0 pull && $0 start"
        ;;

    pull)
        echo "=== Pulling M20 Docker Image on NOS ==="
        echo "Image: ${GHCR_IMAGE}"
        echo ""

        # Ensure NOS clock is correct (TLS fails with wrong date)
        ensure_clock_synced

        # Ensure sequential layer downloads (more resilient on flaky connections)
        ensure_docker_sequential_downloads

        # Retry pull on failure — docker caches completed layers so retries are cheap
        _max_retries=5
        _attempt=1
        while [ $_attempt -le $_max_retries ]; do
            echo "Pull attempt ${_attempt}/${_max_retries}..."
            if remote_ssh docker pull "${GHCR_IMAGE}"; then
                echo ""
                echo "=== Pull Complete ==="
                break
            fi
            if [ $_attempt -eq $_max_retries ]; then
                echo ""
                echo "ERROR: Pull failed after ${_max_retries} attempts."
                echo "Cached layers are preserved — retry with: $0 pull"
                exit 1
            fi
            echo "  Pull failed — retrying in 10s (cached layers preserved)..."
            sleep 10
            _attempt=$((_attempt + 1))
        done
        ;;

    dev)
        echo "=== dimos M20 Fast Dev Sync ==="
        echo "Target: ${NOS_USER}@${NOS_HOST} (via ${JUMP_HOST})"
        echo ""

        # Verify container exists
        if ! remote_ssh docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            echo "ERROR: Container '${CONTAINER_NAME}' not found."
            echo "Run './deploy.sh pull && ./deploy.sh start' first."
            exit 1
        fi

        # Start container if not running (docker cp needs a running container)
        if ! remote_ssh docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            echo "Container not running — starting it first..."
            remote_ssh docker start ${CONTAINER_NAME}
            sleep 2
        fi

        sync_source

        # Copy updated source into the running container.
        # docker cp merges into the existing directory — preserving
        # compiled C++ extensions (.so) that were built during docker build.
        echo "Copying source into container..."
        remote_ssh docker cp ${DEPLOY_DIR}/dimos/. ${CONTAINER_NAME}:/opt/dimos/src/dimos/

        # Also sync docker support files
        remote_ssh docker cp ${DEPLOY_DIR}/dimos/robot/deeprobotics/m20/docker/launch_nos.py ${CONTAINER_NAME}:/opt/dimos/launch_nos.py
        remote_ssh docker cp ${DEPLOY_DIR}/dimos/robot/deeprobotics/m20/docker/entrypoint.sh ${CONTAINER_NAME}:/opt/dimos/entrypoint.sh
        remote_ssh docker cp ${DEPLOY_DIR}/dimos/robot/deeprobotics/m20/docker/fastdds.xml ${CONTAINER_NAME}:/opt/dimos/docker/fastdds.xml

        # docker cp doesn't always preserve execute bits from rsync'd files
        remote_ssh docker exec ${CONTAINER_NAME} chmod +x /opt/dimos/entrypoint.sh

        echo "Restarting container..."
        remote_ssh docker restart ${CONTAINER_NAME}

        sleep 3
        echo ""
        echo "Container status:"
        remote_ssh docker ps --filter name=${CONTAINER_NAME}
        echo ""
        echo "=== Dev Sync Complete ==="
        echo "Tail logs: ./deploy.sh logs"
        ;;

    start)
        echo "=== Starting dimos M20 Docker Container ==="

        check_conflicts
        ensure_lio_enabled
        ensure_nat

        # Remove old container if exists
        remote_ssh docker rm -f ${CONTAINER_NAME} 2>/dev/null || true

        echo "Starting container..."
        remote_ssh docker run -d \
            --name ${CONTAINER_NAME} \
            --privileged \
            --network host \
            --ipc host \
            -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
            -e ROS_DOMAIN_ID=0 \
            -e CI=1 \
            --restart no \
            "${GHCR_IMAGE}"

        sleep 5
        echo ""
        echo "Container status:"
        remote_ssh docker ps --filter name=${CONTAINER_NAME}
        echo ""
        echo "Access from Mac:"
        echo "  Web UI:  http://${ACCESS_HOST}:7779/command-center"
        echo "  Rerun:   rerun --connect rerun+http://${ACCESS_HOST}:9876/proxy"
        ;;

    stop)
        echo "Stopping ${CONTAINER_NAME}..."
        remote_ssh docker stop ${CONTAINER_NAME} 2>/dev/null || true
        remote_ssh docker rm ${CONTAINER_NAME} 2>/dev/null || true
        echo "Stopped."
        ;;

    restart)
        echo "Restarting ${CONTAINER_NAME}..."
        remote_ssh docker restart ${CONTAINER_NAME}
        sleep 3
        echo ""
        echo "Container status:"
        remote_ssh docker ps --filter name=${CONTAINER_NAME}
        ;;

    logs)
        remote_ssh docker logs -f ${CONTAINER_NAME}
        ;;

    shell)
        echo "Opening shell in ${CONTAINER_NAME}..."
        ssh -t ${SSH_OPTS} "${NOS_USER}@${NOS_HOST}" "docker exec -it ${CONTAINER_NAME} /bin/bash"
        ;;

    status)
        echo "=== Container ==="
        remote_ssh docker ps -a --filter name=${CONTAINER_NAME}
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

    bridge-build)
        echo "=== Building drdds bridge on NOS host ==="
        # Sync bridge source to NOS
        rsync -avz --delete \
            -e "ssh ${SSH_OPTS}" \
            "${DIMOS_ROOT}/dimos/robot/deeprobotics/m20/docker/drdds_bridge/" \
            "${NOS_USER}@${NOS_HOST}:/tmp/drdds_bridge/"

        # Build on host (uses host's drdds SDK + FastDDS 2.14)
        remote_ssh "cd /tmp/drdds_bridge && chmod +x build_host.sh && bash build_host.sh"
        echo "=== Bridge build complete ==="
        ;;

    bridge-start)
        echo "=== Starting drdds bridge ==="
        # Start drdds_recv on host (background, uses host's drdds libs)
        remote_ssh "pkill -f drdds_recv 2>/dev/null; sleep 1; \
            nohup /opt/drdds_bridge/lib/drdds_bridge/drdds_recv \
            > /var/log/drdds_recv.log 2>&1 &"
        echo "  drdds_recv started on host"

        # Start ros2_pub in nav container (reads SHM, publishes ROS2 topics)
        remote_ssh "docker exec -d ${CONTAINER_NAME} bash -c '\
            source /opt/ros/humble/setup.bash && \
            source /ros2_ws/install/setup.bash && \
            export FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/config/custom_fastdds.xml && \
            ros2 run drdds_bridge ros2_pub 2>&1 | tee /var/log/ros2_pub.log'" 2>/dev/null || \
        remote_ssh "docker exec -d dimos-nav bash -c '\
            source /opt/ros/humble/setup.bash && \
            source /ros2_ws/install/setup.bash && \
            ros2 run drdds_bridge ros2_pub 2>&1 | tee /var/log/ros2_pub.log'"
        echo "  ros2_pub started in container"
        echo "=== Bridge running ==="
        ;;

    bridge-stop)
        echo "Stopping drdds bridge..."
        remote_ssh "pkill -f drdds_recv 2>/dev/null"
        remote_ssh "docker exec ${CONTAINER_NAME} pkill -f ros2_pub 2>/dev/null" || \
            remote_ssh "docker exec dimos-nav pkill -f ros2_pub 2>/dev/null" || true
        echo "Bridge stopped."
        ;;

    bridge-logs)
        echo "=== drdds_recv (host) ==="
        remote_ssh "tail -20 /var/log/drdds_recv.log 2>/dev/null" || echo "  No logs"
        echo ""
        echo "=== ros2_pub (container) ==="
        remote_ssh "docker exec ${CONTAINER_NAME} cat /var/log/ros2_pub.log 2>/dev/null | tail -20" || \
            remote_ssh "docker exec dimos-nav cat /var/log/ros2_pub.log 2>/dev/null | tail -20" || echo "  No logs"
        ;;

    *)
        echo "Usage: $0 <command> [--host <hostname>]"
        echo ""
        echo "Commands:"
        echo "  push    - Build image on Mac + push to GHCR (~3-5 min)"
        echo "  pull    - Pull latest image on NOS from GHCR (auto-retries, clock sync)"
        echo "  dev     - Sync Python source into running container (~1-2 min)"
        echo "  start   - Start container (with lio health check + NAT)"
        echo "  stop    - Stop and remove container"
        echo "  restart - Restart running container (no sync, ~10-30 sec)"
        echo "  logs    - Tail container logs"
        echo "  shell   - Open shell in running container"
        echo "  status  - Show container + service status"
        echo "  bridge-build - Build drdds bridge on NOS host"
        echo "  bridge-start - Start bridge (drdds_recv + ros2_pub)"
        echo "  bridge-stop  - Stop bridge processes"
        echo "  bridge-logs  - Show bridge logs"
        echo ""
        echo "Options:"
        echo "  --host <hostname>  Access robot via Tailscale (e.g., m20-781-mochi)"
        echo "                     Without --host: uses AOS WiFi (10.21.41.1)"
        echo ""
        echo "Workflows:"
        echo "  Local (on WiFi):"
        echo "    Full deploy:   ./deploy.sh push && ./deploy.sh pull && ./deploy.sh start"
        echo "    Iterate (fast): ./deploy.sh dev"
        echo "    Quick restart:  ./deploy.sh restart"
        echo ""
        echo "  Remote (via Tailscale):"
        echo "    Full deploy:   ./deploy.sh push && ./deploy.sh pull --host m20-781-mochi && ./deploy.sh start --host m20-781-mochi"
        echo "    Iterate (fast): ./deploy.sh dev --host m20-781-mochi"
        echo "    Quick restart:  ./deploy.sh restart --host m20-781-mochi"
        exit 1
        ;;
esac

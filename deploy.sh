#!/usr/bin/env bash
#
# deploy.sh — M20 ROSNav deployment manager
#
# Manages the split NOS host + nav container architecture:
#   - DimOS runs natively on the NOS host (Python 3.10 venv)
#   - CMU nav stack runs in a Humble Docker container
#
# Usage: ./deploy.sh <subcommand> [options]
#
# Subcommands:
#   setup   — One-time NOS host provisioning
#   start   — Launch dimos on NOS host
#   stop    — Stop host dimos process and nav container
#   status  — Show health of host process, nav container, and ROS topics
#   dev     — Sync source changes to NOS venv for rapid iteration
#   logs    — Aggregate logs from host process and nav container
#

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# --- Colors ---
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

# --- Constants ---
DIMOS_VENV="/opt/dimos/venv"
DIMOS_SRC="${SCRIPT_DIR}"
NAV_IMAGE="dimos_autonomy_stack:humble"
DRDDS_BUILDER_NAME="dimos-drdds-builder"
DIMOS_PID_FILE="/var/run/dimos-nos.pid"
NAV_CONTAINER="dimos-nav"
LAUNCH_SCRIPT="launch_nos.py"
DIMOS_LOG="/var/log/dimos-nos.log"

# --- AOS connection defaults (override via environment or .env) ---
AOS_IP="${AOS_IP:-192.168.123.161}"
AOS_USER="${AOS_USER:-unitree}"
AOS_SSH_OPTS="${AOS_SSH_OPTS:--o StrictHostKeyChecking=no -o ConnectTimeout=5}"

# --- Helpers ---

info()  { echo -e "${GREEN}[deploy]${NC} $*"; }
warn()  { echo -e "${YELLOW}[deploy]${NC} $*"; }
error() { echo -e "${RED}[deploy]${NC} $*" >&2; }

aos_ssh() {
    ssh ${AOS_SSH_OPTS} "${AOS_USER}@${AOS_IP}" "$@"
}

aos_sudo() {
    aos_ssh "sudo $*"
}

ensure_lio_disabled() {
    # Stop lio_perception on the AOS board so that the nav container's
    # FASTLIO2 can use the lidar exclusively. Idempotent.
    info "Checking lio_perception on AOS (${AOS_IP})..."

    if ! aos_ssh "pgrep -x lio_perception" >/dev/null 2>&1; then
        info "lio_perception is already stopped on AOS."
        return 0
    fi

    warn "Stopping lio_perception on AOS..."
    aos_sudo "systemctl stop lio_perception.service" 2>/dev/null || true
    aos_sudo "systemctl disable lio_perception.service" 2>/dev/null || true

    if aos_ssh "pgrep -x lio_perception" >/dev/null 2>&1; then
        warn "lio_perception still running, sending SIGTERM..."
        aos_sudo "pkill -x lio_perception" 2>/dev/null || true

        for i in $(seq 1 10); do
            if ! aos_ssh "pgrep -x lio_perception" >/dev/null 2>&1; then
                break
            fi
            sleep 0.5
        done

        if aos_ssh "pgrep -x lio_perception" >/dev/null 2>&1; then
            warn "Force-killing lio_perception..."
            aos_sudo "pkill -9 -x lio_perception" 2>/dev/null || true
            sleep 1
        fi
    fi

    if aos_ssh "pgrep -x lio_perception" >/dev/null 2>&1; then
        error "Failed to stop lio_perception on AOS."
        return 1
    fi

    info "lio_perception stopped successfully on AOS."
    return 0
}

get_dimos_pid() {
    local pid=""

    if [[ -f "$DIMOS_PID_FILE" ]]; then
        pid=$(<"$DIMOS_PID_FILE")
        if kill -0 "$pid" 2>/dev/null && \
           grep -q "$LAUNCH_SCRIPT" "/proc/$pid/cmdline" 2>/dev/null; then
            echo "$pid"
            return 0
        fi
    fi

    pid=$(pgrep -f "$LAUNCH_SCRIPT" 2>/dev/null | head -1) || true
    if [[ -n "$pid" ]]; then
        echo "$pid"
        return 0
    fi

    return 1
}

nav_container_running() {
    docker ps --format '{{.Names}}' 2>/dev/null | grep -q "^${NAV_CONTAINER}$"
}

# =====================================================================
# setup — one-time NOS host provisioning
# =====================================================================
cmd_setup() {
    echo -e "${BOLD}=== NOS Host Provisioning ===${NC}"

    setup_install_uv
    setup_create_venv
    setup_install_dimos
    setup_install_drdds

    echo ""
    info "=== Setup complete ==="
    info "Python : ${DIMOS_VENV}/bin/python3"
    info "Activate: source ${DIMOS_VENV}/bin/activate"
}

setup_install_uv() {
    if command -v uv &>/dev/null; then
        info "uv already installed ($(uv --version))"
        return
    fi
    info "Installing uv..."
    curl -LsSf https://astral.sh/uv/install.sh | sh
    export PATH="${HOME}/.local/bin:${PATH}"
    info "uv installed ($(uv --version))"
}

setup_create_venv() {
    if [ -d "${DIMOS_VENV}" ] && [ -x "${DIMOS_VENV}/bin/python3" ]; then
        local py_ver
        py_ver=$("${DIMOS_VENV}/bin/python3" --version 2>&1 | awk '{print $2}')
        if [[ "${py_ver}" == 3.10.* ]]; then
            info "Python 3.10 venv already exists at ${DIMOS_VENV} (${py_ver})"
            return
        fi
        warn "Existing venv has Python ${py_ver} — recreating with 3.10"
        sudo rm -rf "${DIMOS_VENV}"
    fi

    info "Creating Python 3.10 venv at ${DIMOS_VENV}..."
    sudo mkdir -p "$(dirname "${DIMOS_VENV}")"
    sudo uv venv --python 3.10 "${DIMOS_VENV}"
    sudo chown -R "$(id -u):$(id -g)" "${DIMOS_VENV}"
    info "Venv created ($("${DIMOS_VENV}/bin/python3" --version))"
}

setup_install_dimos() {
    info "Installing dimos into ${DIMOS_VENV}..."
    "${DIMOS_VENV}/bin/pip" install --upgrade pip
    "${DIMOS_VENV}/bin/pip" install -e "${DIMOS_SRC}"
    info "dimos installed"
}

setup_install_drdds() {
    info "Installing drdds bindings..."

    if ! docker image inspect "${NAV_IMAGE}" &>/dev/null; then
        error "Nav container image '${NAV_IMAGE}' not found."
        error "Build it first:  cd docker/navigation && ./build.sh"
        return 1
    fi

    docker rm -f "${DRDDS_BUILDER_NAME}" &>/dev/null || true

    info "Building drdds inside nav container..."
    docker run --name "${DRDDS_BUILDER_NAME}" \
        "${NAV_IMAGE}" \
        bash -c '
set -e
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
source /opt/dimos-venv/bin/activate

mkdir -p /tmp/drdds_out

if [ -f /workspace/dimos/drdds/setup.py ] || [ -f /workspace/dimos/drdds/pyproject.toml ]; then
    pip install --target /tmp/drdds_out /workspace/dimos/drdds
elif python3 -c "import drdds" 2>/dev/null; then
    DRDDS_LOC=$(python3 -c "import drdds, os; print(os.path.dirname(drdds.__file__))")
    cp -r "${DRDDS_LOC}" /tmp/drdds_out/
elif pip install --target /tmp/drdds_out drdds 2>/dev/null; then
    true
else
    echo "WARNING: drdds package not found — skipping"
    exit 0
fi

echo "DRDDS_READY"
'

    local build_output
    build_output=$(docker logs "${DRDDS_BUILDER_NAME}" 2>&1 || true)

    if echo "${build_output}" | grep -q "DRDDS_READY"; then
        local host_site_packages
        host_site_packages=$("${DIMOS_VENV}/bin/python3" -c \
            "import site; print(site.getsitepackages()[0])")
        info "Copying drdds artifacts to ${host_site_packages}..."
        docker cp "${DRDDS_BUILDER_NAME}:/tmp/drdds_out/." "${host_site_packages}/"
    else
        warn "drdds not found in nav container — skipping"
        warn "Ensure drdds is available and re-run: deploy.sh setup"
    fi

    docker rm -f "${DRDDS_BUILDER_NAME}" &>/dev/null || true

    if "${DIMOS_VENV}/bin/python3" -c "import drdds" 2>/dev/null; then
        info "drdds installed and importable"
    else
        warn "drdds import check failed — package may not be available yet"
    fi
}

# =====================================================================
# start — launch dimos on NOS host
# =====================================================================
cmd_start() {
    echo -e "${BOLD}Starting DimOS on NOS host...${NC}"

    # Fail if already running
    local pid
    if pid=$(get_dimos_pid); then
        error "DimOS is already running (PID $pid)."
        error "Run './deploy.sh stop' first."
        return 1
    fi

    # Verify venv exists
    if [[ ! -x "$DIMOS_VENV/bin/python3" ]]; then
        error "Python venv not found at $DIMOS_VENV"
        error "Run './deploy.sh setup' first."
        return 1
    fi

    # Verify drdds is importable
    info "Checking drdds import..."
    if ! "$DIMOS_VENV/bin/python3" -c "import drdds" 2>/dev/null; then
        error "drdds is not importable."
        error "Run './deploy.sh setup' to install drdds bindings."
        return 1
    fi
    info "drdds OK"

    # Disable lio_perception on AOS so FASTLIO2 owns the lidar
    ensure_lio_disabled || {
        error "Failed to disable lio_perception. Aborting start."
        return 1
    }

    # Locate the launch script
    local launch_script="$SCRIPT_DIR/dimos/robot/deeprobotics/m20/docker/launch_nos.py"
    if [[ ! -f "$launch_script" ]]; then
        error "Launch script not found: $launch_script"
        return 1
    fi

    # Launch via nohup, record PID
    info "Launching dimos (logging to $DIMOS_LOG)..."
    nohup "$DIMOS_VENV/bin/python3" "$launch_script" \
        >> "$DIMOS_LOG" 2>&1 &
    local new_pid=$!

    echo "$new_pid" > "$DIMOS_PID_FILE"
    info "DimOS started (PID $new_pid)"

    # Brief health check — verify process survived initial import
    sleep 2
    if ! kill -0 "$new_pid" 2>/dev/null; then
        error "DimOS process exited immediately. Check logs:"
        error "  tail -20 $DIMOS_LOG"
        rm -f "$DIMOS_PID_FILE"
        return 1
    fi

    info "DimOS is running. Nav container starts automatically via DockerModule."
    info "Monitor with: ./deploy.sh status"
}

# =====================================================================
# stop
# =====================================================================
cmd_stop() {
    echo -e "${BOLD}Stopping DimOS (host + nav container)...${NC}"
    local exit_code=0

    local pid
    if pid=$(get_dimos_pid); then
        info "Sending SIGTERM to dimos host process (PID $pid)..."
        kill -TERM "$pid" 2>/dev/null || true

        local waited=0
        while kill -0 "$pid" 2>/dev/null && (( waited < 10 )); do
            sleep 1
            (( waited++ ))
        done

        if kill -0 "$pid" 2>/dev/null; then
            warn "Process $pid did not exit gracefully, sending SIGKILL..."
            kill -KILL "$pid" 2>/dev/null || true
            sleep 1
        fi

        if kill -0 "$pid" 2>/dev/null; then
            error "Failed to stop dimos host process (PID $pid)"
            exit_code=1
        else
            info "Host dimos process stopped."
        fi

        rm -f "$DIMOS_PID_FILE" 2>/dev/null || true
    else
        info "No dimos host process running."
    fi

    if nav_container_running; then
        info "Stopping nav container '${NAV_CONTAINER}'..."
        docker stop --time 15 "$NAV_CONTAINER" 2>/dev/null || true

        if nav_container_running; then
            warn "Container still running, forcing removal..."
            docker kill "$NAV_CONTAINER" 2>/dev/null || true
            docker rm -f "$NAV_CONTAINER" 2>/dev/null || true
        fi

        if nav_container_running; then
            error "Failed to stop nav container '${NAV_CONTAINER}'"
            exit_code=1
        else
            info "Nav container stopped."
        fi
    else
        info "No nav container running."
    fi

    echo ""
    if (( exit_code == 0 )); then
        info "All services stopped."
    else
        error "Some services failed to stop (see above)."
    fi

    return $exit_code
}

# =====================================================================
# status
# =====================================================================
cmd_status() {
    echo -e "${BOLD}DimOS ROSNav Status${NC}"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    echo -e "\n${CYAN}Host Process${NC}"
    local pid
    if pid=$(get_dimos_pid); then
        local uptime
        uptime=$(ps -p "$pid" -o etime= 2>/dev/null | xargs) || uptime="unknown"
        echo -e "  Status:  ${GREEN}● running${NC} (PID $pid)"
        echo -e "  Uptime:  $uptime"
    else
        echo -e "  Status:  ${RED}● stopped${NC}"
    fi

    echo -e "\n${CYAN}Nav Container${NC}"
    if nav_container_running; then
        local container_status
        container_status=$(docker inspect --format '{{.State.Status}} (up {{.State.StartedAt}})' \
            "$NAV_CONTAINER" 2>/dev/null) || container_status="running"
        echo -e "  Status:  ${GREEN}● running${NC}"
        echo -e "  Detail:  $container_status"
    else
        if docker ps -a --format '{{.Names}}' 2>/dev/null | grep -q "^${NAV_CONTAINER}$"; then
            local exit_info
            exit_info=$(docker inspect --format 'exited (code {{.State.ExitCode}})' \
                "$NAV_CONTAINER" 2>/dev/null) || exit_info="exited"
            echo -e "  Status:  ${YELLOW}● $exit_info${NC}"
        else
            echo -e "  Status:  ${RED}● not found${NC}"
        fi
    fi

    echo -e "\n${CYAN}ROS Topics${NC}"
    if command -v ros2 &>/dev/null || [[ -f "$DIMOS_VENV/bin/activate" ]]; then
        if ! command -v ros2 &>/dev/null && [[ -f "$DIMOS_VENV/bin/activate" ]]; then
            # shellcheck disable=SC1091
            source "$DIMOS_VENV/bin/activate" 2>/dev/null || true
        fi

        if command -v ros2 &>/dev/null; then
            local topics=("/IMU" "/LIDAR/POINTS" "/registered_scan")
            for topic in "${topics[@]}"; do
                local hz
                hz=$(timeout 3 ros2 topic hz "$topic" --window 3 2>/dev/null | \
                     grep -oP '[\d.]+' | head -1) || hz=""
                if [[ -n "$hz" ]]; then
                    echo -e "  $topic: ${GREEN}${hz} Hz${NC}"
                else
                    echo -e "  $topic: ${RED}no data${NC}"
                fi
            done
        else
            echo -e "  ${YELLOW}ros2 command not available${NC}"
        fi
    else
        echo -e "  ${YELLOW}ros2 not found (venv not installed?)${NC}"
    fi

    echo ""
}

# =====================================================================
# dev
# =====================================================================
cmd_dev() {
    echo -e "${BOLD}Syncing source to NOS...${NC}"

    local src_dir="$SCRIPT_DIR"
    local dst_dir="$DIMOS_VENV/lib/python3.10/site-packages/dimos"

    if [[ ! -d "$src_dir/dimos" ]]; then
        error "Source directory '$src_dir/dimos' not found."
        error "Run this command from the dimos repo root."
        return 1
    fi

    if [[ ! -d "$dst_dir" ]]; then
        if [[ -d "$DIMOS_VENV/src/dimos/dimos" ]]; then
            dst_dir="$DIMOS_VENV/src/dimos/dimos"
        else
            error "Destination '$dst_dir' not found."
            error "Run 'deploy.sh setup' first to install dimos in the venv."
            return 1
        fi
    fi

    info "Source: $src_dir/dimos/"
    info "Dest:   $dst_dir/"

    rsync -av --delete \
        --exclude '__pycache__' \
        --exclude '*.pyc' \
        --exclude '.git' \
        --exclude '*.egg-info' \
        "$src_dir/dimos/" "$dst_dir/"

    info "Source synced. Restart dimos to pick up changes:"
    echo "  ./deploy.sh stop && ./deploy.sh start"
}

# =====================================================================
# logs
# =====================================================================
cmd_logs() {
    local follow=false
    local lines=50

    while [[ $# -gt 0 ]]; do
        case $1 in
            -f|--follow) follow=true; shift ;;
            -n|--lines)  lines="$2"; shift 2 ;;
            *) warn "Unknown option: $1"; shift ;;
        esac
    done

    echo -e "${BOLD}DimOS Logs${NC}"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    echo -e "\n${CYAN}═══ Host Process Logs ═══${NC}"
    if [[ -f "$DIMOS_LOG" ]]; then
        tail -n "$lines" "$DIMOS_LOG"
    else
        if command -v journalctl &>/dev/null; then
            journalctl -u dimos-nos --no-pager -n "$lines" 2>/dev/null || \
                echo "  (no host logs found at $DIMOS_LOG or in journalctl)"
        else
            echo "  (no host logs found at $DIMOS_LOG)"
        fi
    fi

    echo -e "\n${CYAN}═══ Nav Container Logs ═══${NC}"
    if docker ps -a --format '{{.Names}}' 2>/dev/null | grep -q "^${NAV_CONTAINER}$"; then
        docker logs "$NAV_CONTAINER" --tail "$lines" 2>&1
    else
        echo "  (nav container '$NAV_CONTAINER' not found)"
    fi

    if $follow; then
        echo -e "\n${CYAN}═══ Following logs (Ctrl+C to stop) ═══${NC}"
        local tail_pid=""
        if [[ -f "$DIMOS_LOG" ]]; then
            tail -f "$DIMOS_LOG" | sed "s/^/[host] /" &
            tail_pid=$!
        fi

        if docker ps --format '{{.Names}}' 2>/dev/null | grep -q "^${NAV_CONTAINER}$"; then
            docker logs -f "$NAV_CONTAINER" 2>&1 | sed "s/^/[nav]  /"
        else
            wait "$tail_pid" 2>/dev/null || true
        fi

        if [[ -n "$tail_pid" ]]; then
            kill "$tail_pid" 2>/dev/null || true
        fi
    fi
}

# =====================================================================
# usage + main dispatch
# =====================================================================
usage() {
    cat <<EOF
Usage: $(basename "$0") <subcommand> [options]

M20 ROSNav deployment manager for NOS host.

Subcommands:
  setup    One-time NOS host provisioning (uv, Python 3.10, dimos, drdds)
  start    Launch dimos on NOS host (venv, drdds check, nohup)
  stop     Stop host dimos process and nav container
  status   Show health of host, container, and ROS topics
  dev      Sync source changes to NOS venv
  logs     Show aggregated host + container logs [-f] [-n N]

EOF
}

case "${1:-}" in
    setup)       shift; cmd_setup "$@" ;;
    start)       shift; cmd_start "$@" ;;
    stop)        shift; cmd_stop "$@" ;;
    status)      shift; cmd_status "$@" ;;
    dev)         shift; cmd_dev "$@" ;;
    logs)        shift; cmd_logs "$@" ;;
    -h|--help|help) usage ;;
    *)
        if [ -n "${1:-}" ]; then
            error "Unknown subcommand: $1"
        fi
        usage
        exit 1
        ;;
esac

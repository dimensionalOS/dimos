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
#   stop    — Stop host dimos process and nav container
#   status  — Show health of host process, nav container, and ROS topics
#   dev     — Sync source changes to NOS venv for rapid iteration
#   logs    — Aggregate logs from host process and nav container
#

set -euo pipefail

# --- Colors ---
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

# --- Constants ---
DIMOS_VENV="/opt/dimos/venv"
DIMOS_PID_FILE="/var/run/dimos-nos.pid"
NAV_CONTAINER="dimos-nav"
LAUNCH_SCRIPT="launch_nos.py"
DIMOS_LOG="/var/log/dimos-nos.log"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# --- Helpers ---

info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; }

# Get the PID of the running dimos host process.
# Checks PID file first, then falls back to pgrep.
get_dimos_pid() {
    local pid=""

    # Try PID file first
    if [[ -f "$DIMOS_PID_FILE" ]]; then
        pid=$(<"$DIMOS_PID_FILE")
        # Verify PID is alive and matches launch_nos.py
        if kill -0 "$pid" 2>/dev/null && \
           grep -q "$LAUNCH_SCRIPT" "/proc/$pid/cmdline" 2>/dev/null; then
            echo "$pid"
            return 0
        fi
    fi

    # Fallback to pgrep
    pid=$(pgrep -f "$LAUNCH_SCRIPT" 2>/dev/null | head -1) || true
    if [[ -n "$pid" ]]; then
        echo "$pid"
        return 0
    fi

    return 1
}

# Check if the nav container is running.
nav_container_running() {
    docker ps --format '{{.Names}}' 2>/dev/null | grep -q "^${NAV_CONTAINER}$"
}

# --- Subcommands ---

cmd_stop() {
    echo -e "${BOLD}Stopping DimOS (host + nav container)...${NC}"
    local exit_code=0

    # 1. Stop host dimos process
    local pid
    if pid=$(get_dimos_pid); then
        info "Sending SIGTERM to dimos host process (PID $pid)..."
        kill -TERM "$pid" 2>/dev/null || true

        # Wait for graceful shutdown (up to 10 seconds)
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

        # Clean up PID file
        rm -f "$DIMOS_PID_FILE" 2>/dev/null || true
    else
        info "No dimos host process running."
    fi

    # 2. Stop nav container
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

    # 3. Verification
    echo ""
    if (( exit_code == 0 )); then
        info "All services stopped."
    else
        error "Some services failed to stop (see above)."
    fi

    return $exit_code
}

cmd_status() {
    echo -e "${BOLD}DimOS ROSNav Status${NC}"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    # 1. Host dimos process
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

    # 2. Nav container
    echo -e "\n${CYAN}Nav Container${NC}"
    if nav_container_running; then
        local container_status
        container_status=$(docker inspect --format '{{.State.Status}} (up {{.State.StartedAt}})' \
            "$NAV_CONTAINER" 2>/dev/null) || container_status="running"
        echo -e "  Status:  ${GREEN}● running${NC}"
        echo -e "  Detail:  $container_status"
    else
        # Check if container exists but is stopped
        if docker ps -a --format '{{.Names}}' 2>/dev/null | grep -q "^${NAV_CONTAINER}$"; then
            local exit_info
            exit_info=$(docker inspect --format 'exited (code {{.State.ExitCode}})' \
                "$NAV_CONTAINER" 2>/dev/null) || exit_info="exited"
            echo -e "  Status:  ${YELLOW}● $exit_info${NC}"
        else
            echo -e "  Status:  ${RED}● not found${NC}"
        fi
    fi

    # 3. ROS driver health (ros2 topic hz for key topics)
    echo -e "\n${CYAN}ROS Topics${NC}"
    if command -v ros2 &>/dev/null || [[ -f "$DIMOS_VENV/bin/activate" ]]; then
        # Source venv if needed for ros2 command
        if ! command -v ros2 &>/dev/null && [[ -f "$DIMOS_VENV/bin/activate" ]]; then
            # shellcheck disable=SC1091
            source "$DIMOS_VENV/bin/activate" 2>/dev/null || true
        fi

        if command -v ros2 &>/dev/null; then
            local topics=("/IMU" "/lidar_points" "/cmd_vel")
            for topic in "${topics[@]}"; do
                # Use timeout to avoid hanging on inactive topics
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

cmd_dev() {
    echo -e "${BOLD}Syncing source to NOS...${NC}"

    local src_dir="$SCRIPT_DIR"
    local dst_dir="$DIMOS_VENV/lib/python3.10/site-packages/dimos"

    # Validate source exists
    if [[ ! -d "$src_dir/dimos" ]]; then
        error "Source directory '$src_dir/dimos' not found."
        error "Run this command from the dimos repo root."
        return 1
    fi

    # Validate destination exists
    if [[ ! -d "$dst_dir" ]]; then
        # Try editable install path as fallback
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

cmd_logs() {
    local follow=false
    local lines=50

    while [[ $# -gt 0 ]]; do
        case $1 in
            -f|--follow)
                follow=true
                shift
                ;;
            -n|--lines)
                lines="$2"
                shift 2
                ;;
            *)
                warn "Unknown option: $1"
                shift
                ;;
        esac
    done

    echo -e "${BOLD}DimOS Logs${NC}"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    # Host logs
    echo -e "\n${CYAN}═══ Host Process Logs ═══${NC}"
    if [[ -f "$DIMOS_LOG" ]]; then
        tail -n "$lines" "$DIMOS_LOG"
    else
        # Try journalctl as fallback
        if command -v journalctl &>/dev/null; then
            journalctl -u dimos-nos --no-pager -n "$lines" 2>/dev/null || \
                echo "  (no host logs found at $DIMOS_LOG or in journalctl)"
        else
            echo "  (no host logs found at $DIMOS_LOG)"
        fi
    fi

    # Container logs
    echo -e "\n${CYAN}═══ Nav Container Logs ═══${NC}"
    if docker ps -a --format '{{.Names}}' 2>/dev/null | grep -q "^${NAV_CONTAINER}$"; then
        docker logs "$NAV_CONTAINER" --tail "$lines" 2>&1
    else
        echo "  (nav container '$NAV_CONTAINER' not found)"
    fi

    # Follow mode: tail both simultaneously
    if $follow; then
        echo -e "\n${CYAN}═══ Following logs (Ctrl+C to stop) ═══${NC}"
        # Use tail + docker logs in follow mode
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

        # Cleanup tail on exit
        if [[ -n "$tail_pid" ]]; then
            kill "$tail_pid" 2>/dev/null || true
        fi
    fi
}

# --- Main ---

usage() {
    echo "Usage: $0 <subcommand> [options]"
    echo ""
    echo "Subcommands:"
    echo "  stop      Stop host dimos process and nav container"
    echo "  status    Show health of host, container, and ROS topics"
    echo "  dev       Sync source changes to NOS venv"
    echo "  logs      Show aggregated host + container logs"
    echo ""
    echo "Log options:"
    echo "  -f, --follow    Follow log output"
    echo "  -n, --lines N   Number of lines to show (default: 50)"
}

if [[ $# -lt 1 ]]; then
    usage
    exit 1
fi

subcommand="$1"
shift

case "$subcommand" in
    stop)   cmd_stop "$@" ;;
    status) cmd_status "$@" ;;
    dev)    cmd_dev "$@" ;;
    logs)   cmd_logs "$@" ;;
    -h|--help|help)
        usage
        ;;
    *)
        error "Unknown subcommand: $subcommand"
        usage
        exit 1
        ;;
esac

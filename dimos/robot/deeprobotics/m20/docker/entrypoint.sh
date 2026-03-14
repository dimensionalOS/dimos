#!/usr/bin/env bash
# M20 ROSNav container entrypoint
# Runs inside the nav container launched by DockerModule.
# Waits for required ROS topics (/IMU, /LIDAR/POINTS) before starting
# the navigation stack based on LOCALIZATION_METHOD.
#
# LOCALIZATION_METHOD values:
#   fastlio  — Launch FAST_LIO with RoboSense config (Phase 1)
#   <other>  — Launch system_real_robot.launch.py (arise_slam)

set -euo pipefail

TOPIC_TIMEOUT="${TOPIC_TIMEOUT:-60}"
POLL_INTERVAL="${POLL_INTERVAL:-2}"
LOCALIZATION_METHOD="${LOCALIZATION_METHOD:-fastlio}"
FASTLIO_CONFIG="${FASTLIO_CONFIG:-/ros2_ws/src/fast_lio/config/robosense.yaml}"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

log()  { echo -e "${GREEN}[entrypoint]${NC} $*"; }
warn() { echo -e "${YELLOW}[entrypoint]${NC} $*"; }
err()  { echo -e "${RED}[entrypoint]${NC} $*" >&2; }

# --- Source ROS environment ---
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
if [ -d "/ros2_ws/install" ]; then
    source /ros2_ws/install/setup.bash
fi

# --- Wait for a ROS topic to appear ---
wait_for_topic() {
    local topic="$1"
    local elapsed=0

    log "Waiting for topic ${topic} (timeout: ${TOPIC_TIMEOUT}s)..."
    while [ "$elapsed" -lt "$TOPIC_TIMEOUT" ]; do
        if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
            log "Topic ${topic} available."
            return 0
        fi
        sleep "$POLL_INTERVAL"
        elapsed=$((elapsed + POLL_INTERVAL))
    done

    err "Topic ${topic} not available after ${TOPIC_TIMEOUT}s."
    return 1
}

# --- Cleanup on exit ---
PIDS=()
cleanup() {
    log "Shutting down..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -TERM "$pid" 2>/dev/null || true
        fi
    done
    sleep 2
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -KILL "$pid" 2>/dev/null || true
        fi
    done
}
trap cleanup EXIT INT TERM

# --- Wait for required topics ---
log "M20 ROSNav entrypoint starting (LOCALIZATION_METHOD=${LOCALIZATION_METHOD})..."

if ! wait_for_topic "/IMU"; then
    err "Required topic /IMU unavailable. Exiting."
    exit 1
fi

if ! wait_for_topic "/LIDAR/POINTS"; then
    err "Required topic /LIDAR/POINTS unavailable. Exiting."
    exit 1
fi

log "All required topics available."

# --- If explicit command provided, run it instead of auto-launch ---
if [ $# -gt 0 ]; then
    log "Executing provided command: $*"
    exec "$@"
fi

# --- Launch navigation stack based on LOCALIZATION_METHOD ---
case "${LOCALIZATION_METHOD}" in
    fastlio)
        log "Launching FAST_LIO with config: ${FASTLIO_CONFIG}"

        if [ ! -f "${FASTLIO_CONFIG}" ]; then
            err "FAST_LIO config not found: ${FASTLIO_CONFIG}"
            exit 1
        fi

        # Launch FAST_LIO node with topic remap:
        #   /cloud_registered -> /registered_scan (what ROSNav bridge subscribes to)
        ros2 run fast_lio fastlio_mapping \
            --ros-args \
            --params-file "${FASTLIO_CONFIG}" \
            -r /cloud_registered:=/registered_scan &
        PIDS+=($!)
        log "FAST_LIO started (PID ${PIDS[-1]})"

        # Wait for FAST_LIO to produce output
        if ! wait_for_topic "/registered_scan"; then
            err "FAST_LIO failed to produce /registered_scan. Check logs above."
            exit 1
        fi
        log "FAST-LIO is activated — /registered_scan publishing."

        # Launch base_autonomy + FAR planner (system_real_robot.launch.py starts
        # the full autonomy stack including arise_slam; we kill arise_slam after
        # since FAST_LIO already provides SLAM).
        log "Launching autonomy stack (base_autonomy + FAR planner)..."
        ros2 launch vehicle_simulator system_real_robot.launch.py &
        PIDS+=($!)

        # Give arise_slam time to start, then stop it to save memory — FAST_LIO
        # is already handling SLAM. The autonomy stack's other nodes (localPlanner,
        # terrainAnalysis, pathFollower, etc.) continue running.
        sleep 10
        if pkill -f "arise_slam" 2>/dev/null; then
            log "Stopped arise_slam (FAST_LIO handles SLAM instead)."
        fi
        ;;

    *)
        log "Launching default autonomy stack (arise_slam)..."
        ros2 launch vehicle_simulator system_real_robot.launch.py &
        PIDS+=($!)
        ;;
esac

log "Navigation stack launched. Waiting for processes..."
wait "${PIDS[@]}" 2>/dev/null || true

#!/usr/bin/env bash
# M20 ROSNav container entrypoint
# Runs inside the nav container launched by DockerModule.
# Waits for required ROS topics (/IMU, /lidar_points) before starting
# the CMU navigation stack (FASTLIO2 + base_autonomy + FAR planner).

set -euo pipefail

TOPIC_TIMEOUT="${TOPIC_TIMEOUT:-60}"
POLL_INTERVAL="${POLL_INTERVAL:-2}"

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

# --- Wait for required topics ---
log "M20 ROSNav entrypoint starting..."

if ! wait_for_topic "/IMU"; then
    err "Required topic /IMU unavailable. Exiting."
    exit 1
fi

if ! wait_for_topic "/LIDAR/POINTS"; then
    err "Required topic /LIDAR/POINTS unavailable. Exiting."
    exit 1
fi

log "All required topics available. Launching navigation stack."

# --- Execute the provided command (nav stack launch) or default ---
exec "$@"

#!/bin/bash
set -e

# Source ROS2 Humble
source /opt/ros/humble/setup.bash

# Activate dimos venv
source /opt/dimos-venv/bin/activate

# DDS configuration
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/dimos/docker/fastdds.xml

# drdds libs rebuilt from .msg files during docker build (ABI-compatible)
export LD_LIBRARY_PATH=/opt/drdds/lib:/opt/ros/humble/lib:${LD_LIBRARY_PATH}

# drdds Python 3.10 bindings (built during docker build)
export PYTHONPATH="/opt/drdds/lib/python3/site-packages:${PYTHONPATH}"

# LCM multicast configuration (required by dimos service layer)
ip route add 224.0.0.0/4 dev lo 2>/dev/null || true
sysctl -w net.core.rmem_max=67108864 2>/dev/null || true
sysctl -w net.core.rmem_default=67108864 2>/dev/null || true

# Prevent ros2 CLI from spawning a daemon on the host (--network host shares
# the filesystem namespace). The Foxy daemon on NOS thrashes at 78% CPU trying
# to reconcile Humble DDS discovery traffic.
export ROS2_DAEMON_TIMEOUT=0
ros2 daemon stop >/dev/null 2>&1 || true

# Wait for ROS2 topics before launching dimos (data flow verification).
# Checks run in parallel — 30s total timeout instead of 30s per topic.
echo "Waiting for ROS2 topics (30s timeout)..."
wait_topic() {
    local topic=$1
    timeout 30 bash -c "until ros2 topic echo $topic --once >/dev/null 2>&1; do sleep 1; done" \
        && echo "  $topic: OK" || echo "  $topic: TIMEOUT (continuing anyway)"
}
wait_topic /ODOM &
wait_topic /IMU &
wait

# Verify drdds bindings (non-fatal — falls back to UDP if unavailable)
python3 -c "from drdds.msg import NavCmd; print('drdds available — /NAV_CMD publisher enabled')" 2>/dev/null \
    || echo "WARNING: drdds Python bindings not available — using UDP fallback (no obstacle avoidance)"

exec "$@"

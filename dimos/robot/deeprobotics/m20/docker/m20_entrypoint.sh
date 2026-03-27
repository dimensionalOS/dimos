#!/bin/bash
#
# M20 wrapper entrypoint — starts the drdds bridge (ros2_pub) before
# handing off to the base nav entrypoint.
#
# ros2_pub must start from the entrypoint context (not docker exec) because
# DDS discovery via docker exec is broken with --ipc host (Finding #10).

set -e

# Start ros2_pub if available and we're in hardware + arise_slam mode
if [ "${MODE}" = "hardware" ] && [ "${LOCALIZATION_METHOD}" = "arise_slam" ]; then
    ROS2_PUB="/ros2_ws/install/drdds_bridge/lib/drdds_bridge/ros2_pub"
    if [ -x "$ROS2_PUB" ]; then
        echo "[m20] Starting ros2_pub (drdds SHM → ROS2 bridge)..."
        source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
        source /ros2_ws/install/setup.bash
        "$ROS2_PUB" &
        ROS2_PUB_PID=$!
        echo "[m20] ros2_pub PID: $ROS2_PUB_PID"

        # Wait for bridge to connect to SHM (created by drdds_recv on host)
        for i in $(seq 1 30); do
            if [ -f /dev/shm/drdds_bridge_lidar ]; then
                echo "[m20] SHM connected (${i}s)"
                break
            fi
            sleep 1
        done
    else
        echo "[m20] WARNING: ros2_pub not found at $ROS2_PUB"
    fi
fi

# Hand off to base nav entrypoint
exec /usr/local/bin/entrypoint.sh "$@"

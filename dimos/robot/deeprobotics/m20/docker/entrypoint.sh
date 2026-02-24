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

# Ensure LD_LIBRARY_PATH includes ROS and drdds
export LD_LIBRARY_PATH=/opt/ros/humble/lib:/opt/drdds/lib:${LD_LIBRARY_PATH}

# Wait for ROS2 topics before launching dimos (data flow verification)
echo "Waiting for ROS2 topics..."
for topic in /ODOM /IMU; do
    echo -n "  $topic: "
    timeout 30 bash -c "until ros2 topic echo $topic --once >/dev/null 2>&1; do sleep 1; done" \
        && echo "OK" || echo "TIMEOUT (continuing anyway)"
done

exec "$@"

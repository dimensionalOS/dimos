#!/bin/bash
set -e

# Create supervisor log directory
mkdir -p /app/assets/output

# Delete old logs
echo "Cleaning up old Supervisor logs..."
rm -f /app/assets/output/*.log

# Source ROS2 environment
source /opt/ros/${ROS_DISTRO}/setup.bash
source /ros2_ws/install/setup.bash

# Execute the command passed to docker run
exec "$@"
# python3 -m tests.test_unitree_agent

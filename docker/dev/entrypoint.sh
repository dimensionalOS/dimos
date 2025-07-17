#!/usr/bin/env bash
if [ -d "/opt/ros/${ROS_DISTRO}" ]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
else
    echo "ROS is not available in this env"
fi

exec "$@"

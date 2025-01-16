#!/bin/bash
export PYTHONPATH="${PYTHONPATH}:/app"
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
exec "$@" 
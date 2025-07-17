#!/bin/bash
export PYTHONPATH="${PYTHONPATH}:/app"
source /opt/ros/humble/setup.bash
#source /home/ros/dev_ws/install/setup.bash
exec "$@" 
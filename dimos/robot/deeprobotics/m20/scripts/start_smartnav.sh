#!/bin/bash
# Launch M20 smartnav (FAST-LIO2 + Airy IMU + Full nav stack).
# Runs on NOS. Writes pid to /tmp/smartnav.pid for stop scripts.
#
# Usage (on NOS):
#   ./start_smartnav.sh
#
# Or via deploy.sh start --host m20-770-gogo (from Mac).

: > /tmp/smartnav_native.log
cd /var/opt/robot/data/dimos
export M20_SLAM_BACKEND=fastlio2
export M20_FASTLIO2_IMU=airy
nohup /home/user/dimos-venv/bin/python -m dimos.robot.deeprobotics.m20.blueprints.nav.m20_smartnav_native \
    > /tmp/smartnav_native.log 2>&1 &
echo $! > /tmp/smartnav.pid

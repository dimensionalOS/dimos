#!/bin/bash
# Launch M20 nav stack (FAST-LIO2 + Airy IMU).
# Runs on NOS. Writes pid to /tmp/m20_nav_stack.pid for stop scripts.
#
# Usage (on NOS):
#   ./start_nav_stack.sh
#
# Or via deploy.sh start --host m20-770-gogo (from Mac).

: > /tmp/m20_nav_stack_native.log
cd /var/opt/robot/data/dimos
export M20_SLAM_BACKEND=fastlio2
export M20_FASTLIO2_IMU=airy
nohup /home/user/dimos-venv/bin/python -m dimos.robot.deeprobotics.m20.blueprints.nav.m20_nav_stack_native \
    > /tmp/m20_nav_stack_native.log 2>&1 &
echo $! > /tmp/m20_nav_stack.pid

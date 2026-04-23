#!/bin/bash
# Launch M20 smartnav WITHOUT the autonomous planner stack.
# SLAM + IMU + teleop-friendly only — use when testing SLAM in isolation
# without simple_planner injecting conflicting cmd_vel (e.g. when chasing
# a phantom goal from an earlier session).

: > /tmp/smartnav_native.log
cd /var/opt/robot/data/dimos
export M20_SLAM_BACKEND=fastlio2
export M20_FASTLIO2_IMU=airy
export M20_NAV_ENABLED=0
nohup /home/user/dimos-venv/bin/python -m dimos.robot.deeprobotics.m20.blueprints.nav.m20_smartnav_native \
    > /tmp/smartnav_native.log 2>&1 &
echo $! > /tmp/smartnav.pid

#!/bin/bash
# Launch M20 nav stack WITHOUT the autonomous planner stack.
# SLAM + IMU + teleop-friendly only — use when testing SLAM in isolation
# without simple_planner injecting conflicting cmd_vel (e.g. when chasing
# a phantom goal from an earlier session).

: > /tmp/m20_nav_stack_native.log
cd /var/opt/robot/data/dimos
export M20_SLAM_BACKEND=fastlio2
export M20_FASTLIO2_IMU=airy
export M20_NAV_ENABLED=0
nohup /home/user/dimos-venv/bin/python -m dimos.robot.deeprobotics.m20.blueprints.nav.m20_nav_stack_native \
    > /tmp/m20_nav_stack_native.log 2>&1 &
echo $! > /tmp/m20_nav_stack.pid

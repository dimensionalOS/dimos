#!/bin/bash
# Boot the Galaxea R1 Lite ROS stack — run ON THE ROBOT.
#
#   ./scripts/r1lite_test/roslaunch.sh          # boot the stack (no-op if already up)
#   ./scripts/r1lite_test/roslaunch.sh stop     # shut the stack down
#
# The whole onboard flow for a fresh robot:
#   1. ./scripts/r1lite_test/roslaunch.sh                  # stack up
#   2. bash scripts/r1lite_test/r1lite_dimos_install.sh    # first time only
#   3. ./scripts/r1lite_test/run_r1lite.sh                 # every session
# (run_r1lite.sh also calls this script itself, so forgetting step 1 is fine.)
#
# SAFETY: booting the stack makes the arms and grippers twitch (~30s HDAS
# init). Keep the robot clear and the e-stop within reach.

set -e

STARTUP_DIR="$HOME/galaxea/install/startup_config/share/startup_config/script"
SESSION_CFG="../sessions.d/ATCStandard/R1LITEBody.d"

if [ ! -d "$STARTUP_DIR" ]; then
    echo "[roslaunch] ERROR: $STARTUP_DIR not found."
    echo "[roslaunch] This script runs ON the R1 Lite onboard PC (ssh r1lite), not the laptop."
    exit 1
fi

if [ "$1" = "stop" ]; then
    ( cd "$STARTUP_DIR" && ./robot_startup.sh kill )
    echo "[roslaunch] stack stopped"
    exit 0
fi

if tmux ls 2>/dev/null | grep -q hdas; then
    echo "[roslaunch] Galaxea stack already running:"
    tmux ls
    exit 0
fi

echo "[roslaunch] Booting Galaxea stack (~30s). ARMS AND GRIPPERS WILL TWITCH —"
echo "[roslaunch] make sure the robot is clear and the e-stop is in reach."
( cd "$STARTUP_DIR" && ./robot_startup.sh boot "$SESSION_CFG" )
sleep 30
# The factory GELLO teleop session grabs the arms — keep it off.
tmux kill-session -t r1lite_teleop 2>/dev/null || true

echo "[roslaunch] stack up:"
tmux ls
echo "[roslaunch] next: ./scripts/r1lite_test/run_r1lite.sh   (or the installer on first setup)"

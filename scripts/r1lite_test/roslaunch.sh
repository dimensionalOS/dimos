#!/bin/bash
# Boot or stop the Galaxea vendor stack. Usage: roslaunch.sh [stop]

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

# The factory GELLO teleop session grabs the arms: while it lives, our arm
# commands are silently overridden. Kill it on BOTH paths, it used to sit
# after the early-exit below, so a stack that was already up (autostart, or a
# previous boot) kept GELLO on the arms and the script still reported success.
kill_gello() {
    if tmux has-session -t r1lite_teleop 2>/dev/null; then
        tmux kill-session -t r1lite_teleop 2>/dev/null || true
        echo "[roslaunch] killed the factory GELLO teleop session (it holds the arms)"
    fi
}

if tmux ls 2>/dev/null | grep -q hdas; then
    echo "[roslaunch] Galaxea stack already running:"
    kill_gello
    tmux ls
    exit 0
fi

echo "[roslaunch] Booting Galaxea stack (~30s). ARMS AND GRIPPERS WILL TWITCH, "
echo "[roslaunch] make sure the robot is clear and the e-stop is in reach."
( cd "$STARTUP_DIR" && ./robot_startup.sh boot "$SESSION_CFG" )
sleep 30
kill_gello

echo "[roslaunch] stack up:"
tmux ls
echo "[roslaunch] next: ./scripts/r1lite_test/run_r1lite.sh   (or the installer on first setup)"

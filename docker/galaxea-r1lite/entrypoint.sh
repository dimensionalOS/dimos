#!/bin/bash
# Source ROS, then run the dimos CLI. For a blueprint launch, wait for the
# vendor stack's /hdas/* topics first. DIMOS_NO_WAIT=1 skips the wait.
set -e

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-2}"

if [ "$1" = "run" ] && [ -z "$DIMOS_NO_WAIT" ]; then
    for i in $(seq 1 60); do
        if ros2 topic list 2>/dev/null | grep -q '^/hdas/'; then
            break
        fi
        [ "$i" = 1 ] && echo "[entrypoint] waiting for Galaxea stack (/hdas/* on domain $ROS_DOMAIN_ID)..."
        sleep 2
    done
    if ! ros2 topic list 2>/dev/null | grep -q '^/hdas/'; then
        echo "[entrypoint] no /hdas topics after 120s; launching anyway"
    fi
fi

exec dimos "$@"

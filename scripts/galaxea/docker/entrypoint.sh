#!/bin/bash
# Entrypoint for the dimos-r1lite runtime image: source ROS, then hand every
# argument to the dimos CLI (`docker run <image> list` == `dimos list`).
#
# When launching a blueprint (`run ...`), first wait for the Galaxea vendor
# stack's /hdas/* topics so container start order vs. the robot stack never
# matters (compose `restart: unless-stopped` + this wait = boot-order-proof).
# Set DIMOS_NO_WAIT=1 to skip.
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
        echo "[entrypoint] WARNING: no /hdas topics after 120s (vendor stack down?) — launching anyway"
    fi
fi

exec dimos "$@"

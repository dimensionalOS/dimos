#!/usr/bin/env bash
# Entrypoint for the on-robot dimos container.
set -e

source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"

# LCM's udpm transport needs a multicast-capable route for its group. All
# dimos traffic is same-host on the robot, so pin JUST the LCM group to
# loopback. Deliberately NOT 224.0.0.0/4 — a blanket multicast route via lo
# would also capture DDS discovery multicast and break robot<->PC ROS
# debugging while tethered.
#
# With --network=host this edits the host netns → needs --cap-add NET_ADMIN
# (run.sh passes it). `ip route replace` makes re-runs idempotent.
LCM_GROUP="${LCM_GROUP:-239.255.76.67}"
ip link set lo multicast on 2>/dev/null || true
ip route replace "${LCM_GROUP}/32" dev lo 2>/dev/null || true
if ip route get "${LCM_GROUP}" 2>/dev/null | grep -q "dev lo"; then
    echo "[entrypoint] LCM multicast ${LCM_GROUP} routed via lo"
else
    echo "[entrypoint] WARN: LCM multicast route not on lo. Either run with"
    echo "[entrypoint]   --cap-add NET_ADMIN (see run.sh), or once on the host:"
    echo "[entrypoint]   sudo ip link set lo multicast on && sudo ip route add ${LCM_GROUP}/32 dev lo"
    echo "[entrypoint] LCM pubsub (all inter-module traffic) will not flow without it."
fi

exec "$@"

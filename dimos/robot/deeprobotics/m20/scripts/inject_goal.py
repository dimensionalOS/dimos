#!/usr/bin/env python3
"""Inject a single click-to-goal point at <distance> meters in front of the
robot's current pose.

Bypasses the dimos-viewer click path entirely so the only thing that
publishes /clicked_point is this script. Lets us validate click-to-goal
without exercising the viewer's WebSocket → CmdVelMux → ClickToGoal
phantom-goal chain (see FASTLIO2_LOG re: 2026-04-27 incident).

Usage on NOS:
    /home/user/dimos-venv/bin/python \\
        /var/opt/robot/data/dimos/dimos/robot/deeprobotics/m20/scripts/inject_goal.py [distance_m]

distance_m defaults to 1.0. Positive = forward in robot's heading.
"""

import math
import sys
import time

import lcm

# Use dimos.msgs.* types (same as ClickToGoal + SimplePlanner) instead of
# dimos_lcm.* types directly. The dimos.msgs.PointStamped.lcm_encode
# wraps the LCM PointStamped and sets header.stamp.sec/nsec from
# self.ts, which the rest of the dimos pipeline expects.
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.nav_msgs.Odometry import Odometry


ODOM_TOPIC = "/odometry#nav_msgs.Odometry"
GOAL_TOPIC = "/clicked_point#geometry_msgs.PointStamped"


_pose: dict = {"x": None, "y": None, "z": None,
               "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}


def _on_odom(channel: str, data: bytes) -> None:
    # dimos.msgs.Odometry inherits a flat .position / .orientation surface
    # (same shape as ClickToGoal._on_odom uses).
    msg = Odometry.lcm_decode(data)
    _pose["x"] = msg.position.x
    _pose["y"] = msg.position.y
    _pose["z"] = msg.position.z
    _pose["qx"] = msg.orientation.x
    _pose["qy"] = msg.orientation.y
    _pose["qz"] = msg.orientation.z
    _pose["qw"] = msg.orientation.w


def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def main() -> int:
    distance = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0

    lc = lcm.LCM()
    sub = lc.subscribe(ODOM_TOPIC, _on_odom)

    print(f"[inject_goal] waiting for odometry on {ODOM_TOPIC} ...", flush=True)
    t0 = time.time()
    while _pose["x"] is None and time.time() - t0 < 5.0:
        lc.handle_timeout(100)

    if _pose["x"] is None:
        print("[inject_goal] ERROR: no /odometry message received in 5s. "
              "Is FAST-LIO running and out of preroll?", file=sys.stderr)
        return 1

    yaw = _quat_to_yaw(
        _pose["qx"], _pose["qy"], _pose["qz"], _pose["qw"]
    )
    rx, ry, rz = _pose["x"], _pose["y"], _pose["z"]
    gx = rx + distance * math.cos(yaw)
    gy = ry + distance * math.sin(yaw)
    gz = rz

    print(
        f"[inject_goal] robot pose = ({rx:+.3f}, {ry:+.3f}, "
        f"yaw={math.degrees(yaw):+.1f}°)",
        flush=True,
    )
    print(
        f"[inject_goal] goal       = ({gx:+.3f}, {gy:+.3f}, {gz:+.3f}) "
        f"— {distance:+.2f}m forward",
        flush=True,
    )
    print(
        "[inject_goal] publishing once to /clicked_point. "
        "ClickToGoal will forward to /goal + /way_point.",
        flush=True,
    )

    # dimos.msgs.PointStamped accepts x/y/z/ts/frame_id directly. Same
    # construction ClickToGoal._on_stop_movement uses, so the encode path
    # is identical and SimplePlanner's _on_goal will deserialize cleanly.
    msg = PointStamped(x=gx, y=gy, z=gz, ts=time.time(), frame_id="map")
    lc.publish(GOAL_TOPIC, msg.lcm_encode())

    # Brief drain so the publish actually leaves the socket buffer before exit.
    time.sleep(0.3)
    lc.unsubscribe(sub)

    print("[inject_goal] done.", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())

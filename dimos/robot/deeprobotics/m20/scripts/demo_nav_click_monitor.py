#!/usr/bin/env python3
"""Monitor the M20 click-to-goal LCM chain during live bring-up."""

from __future__ import annotations

import argparse
import json
import math
import time
from typing import Any

import lcm
from dimos_lcm.geometry_msgs.Twist import Twist
from dimos_lcm.std_msgs.Bool import Bool

from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path as NavPath


CLICK_TOPIC = "/clicked_point#geometry_msgs.PointStamped"
WAYPOINT_TOPIC = "/way_point#geometry_msgs.PointStamped"
PATH_TOPIC = "/path#nav_msgs.Path"
ODOM_TOPIC = "/odometry#nav_msgs.Odometry"
CORRECTED_ODOM_TOPIC = "/corrected_odometry#nav_msgs.Odometry"
STOP_TOPIC = "/stop_movement#std_msgs.Bool"
CMD_TOPICS = {
    "cmd": "/cmd_vel#geometry_msgs.Twist",
    "nav": "/nav_cmd_vel#geometry_msgs.Twist",
    "tele": "/tele_cmd_vel#geometry_msgs.Twist",
}


def _yaw_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(b - a), math.cos(b - a))


def _point_summary(point: PointStamped | None) -> str:
    if point is None:
        return "none"
    return f"({point.x:+.2f},{point.y:+.2f},{point.z:+.2f})[{point.frame_id}]"


def _is_zero_twist(msg: Twist, eps: float = 1e-6) -> bool:
    return (
        abs(msg.linear.x) < eps
        and abs(msg.linear.y) < eps
        and abs(msg.linear.z) < eps
        and abs(msg.angular.x) < eps
        and abs(msg.angular.y) < eps
        and abs(msg.angular.z) < eps
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=45.0)
    parser.add_argument("--print-period", type=float, default=1.0)
    parser.add_argument("--json-out", default="")
    args = parser.parse_args()

    lc = lcm.LCM()
    start = time.monotonic()
    next_print = start + args.print_period

    counts: dict[str, int] = {
        "click": 0,
        "waypoint": 0,
        "path": 0,
        "odom": 0,
        "corrected_odom": 0,
        "stop": 0,
        "path_nonempty": 0,
    }
    cmd_counts = {name: 0 for name in CMD_TOPICS}
    nonzero_cmd_counts = {name: 0 for name in CMD_TOPICS}
    max_cmd = {name: {"vxy": 0.0, "wz": 0.0} for name in CMD_TOPICS}

    last_click: PointStamped | None = None
    last_waypoint: PointStamped | None = None
    last_path_len = 0
    max_path_len = 0
    first_odom: tuple[float, float, float] | None = None
    last_odom: tuple[float, float, float] | None = None
    first_corrected: tuple[float, float, float] | None = None
    last_corrected: tuple[float, float, float] | None = None

    def on_click(_channel: str, data: bytes) -> None:
        nonlocal last_click
        last_click = PointStamped.lcm_decode(data)
        counts["click"] += 1

    def on_waypoint(_channel: str, data: bytes) -> None:
        nonlocal last_waypoint
        last_waypoint = PointStamped.lcm_decode(data)
        counts["waypoint"] += 1

    def on_path(_channel: str, data: bytes) -> None:
        nonlocal last_path_len, max_path_len
        msg = NavPath.lcm_decode(data)
        last_path_len = len(msg.poses)
        max_path_len = max(max_path_len, last_path_len)
        counts["path"] += 1
        if last_path_len > 0:
            counts["path_nonempty"] += 1

    def on_odom(kind: str):
        def _cb(_channel: str, data: bytes) -> None:
            nonlocal first_odom, last_odom, first_corrected, last_corrected
            msg = Odometry.lcm_decode(data)
            pose = (msg.x, msg.y, msg.yaw)
            if kind == "odom":
                counts["odom"] += 1
                if first_odom is None:
                    first_odom = pose
                last_odom = pose
            else:
                counts["corrected_odom"] += 1
                if first_corrected is None:
                    first_corrected = pose
                last_corrected = pose

        return _cb

    def on_stop(_channel: str, data: bytes) -> None:
        msg = Bool.lcm_decode(data)
        if msg.data:
            counts["stop"] += 1

    def on_cmd(name: str):
        def _cb(_channel: str, data: bytes) -> None:
            msg = Twist.lcm_decode(data)
            cmd_counts[name] += 1
            vxy = math.hypot(msg.linear.x, msg.linear.y)
            max_cmd[name]["vxy"] = max(max_cmd[name]["vxy"], vxy)
            max_cmd[name]["wz"] = max(max_cmd[name]["wz"], abs(msg.angular.z))
            if not _is_zero_twist(msg):
                nonzero_cmd_counts[name] += 1

        return _cb

    lc.subscribe(CLICK_TOPIC, on_click)
    lc.subscribe(WAYPOINT_TOPIC, on_waypoint)
    lc.subscribe(PATH_TOPIC, on_path)
    lc.subscribe(ODOM_TOPIC, on_odom("odom"))
    lc.subscribe(CORRECTED_ODOM_TOPIC, on_odom("corrected"))
    lc.subscribe(STOP_TOPIC, on_stop)
    for name, topic in CMD_TOPICS.items():
        lc.subscribe(topic, on_cmd(name))

    def odom_delta(
        first: tuple[float, float, float] | None,
        last: tuple[float, float, float] | None,
    ) -> dict[str, float] | None:
        if first is None or last is None:
            return None
        return {
            "dx": last[0] - first[0],
            "dy": last[1] - first[1],
            "dyaw_deg": math.degrees(_yaw_delta(first[2], last[2])),
        }

    while time.monotonic() - start < args.duration:
        lc.handle_timeout(100)
        now = time.monotonic()
        if now >= next_print:
            elapsed = now - start
            od = odom_delta(first_odom, last_odom)
            odom_text = "none" if od is None else (
                f"dx={od['dx']:+.2f} dy={od['dy']:+.2f} dyaw={od['dyaw_deg']:+.1f}deg"
            )
            print(
                f"{elapsed:5.1f}s click={counts['click']} wp={counts['waypoint']} "
                f"path={counts['path']}/{counts['path_nonempty']} "
                f"last_path={last_path_len} max_path={max_path_len} "
                f"stop={counts['stop']} "
                f"nonzero nav/cmd/tele="
                f"{nonzero_cmd_counts['nav']}/{nonzero_cmd_counts['cmd']}/{nonzero_cmd_counts['tele']} "
                f"max_cmd_vxy={max_cmd['cmd']['vxy']:.3f} max_cmd_wz={max_cmd['cmd']['wz']:.3f} "
                f"click={_point_summary(last_click)} wp={_point_summary(last_waypoint)} "
                f"odom={odom_text}",
                flush=True,
            )
            next_print += args.print_period

    summary: dict[str, Any] = {
        "duration": args.duration,
        "counts": counts,
        "cmd_counts": cmd_counts,
        "nonzero_cmd_counts": nonzero_cmd_counts,
        "max_cmd": max_cmd,
        "last_click": None
        if last_click is None
        else {
            "x": last_click.x,
            "y": last_click.y,
            "z": last_click.z,
            "frame_id": last_click.frame_id,
        },
        "last_waypoint": None
        if last_waypoint is None
        else {
            "x": last_waypoint.x,
            "y": last_waypoint.y,
            "z": last_waypoint.z,
            "frame_id": last_waypoint.frame_id,
        },
        "last_path_len": last_path_len,
        "max_path_len": max_path_len,
        "odom_delta": odom_delta(first_odom, last_odom),
        "corrected_odom_delta": odom_delta(first_corrected, last_corrected),
    }
    print("--- summary ---")
    print(json.dumps(summary, indent=2, sort_keys=True))
    if args.json_out:
        with open(args.json_out, "w") as f:
            json.dump(summary, f, indent=2, sort_keys=True)
            f.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

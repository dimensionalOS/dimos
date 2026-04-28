#!/usr/bin/env python3
"""Segmented live monitor for repeated M20 click-to-goal tests."""

from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import asdict, dataclass, field
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


def _is_zero_twist(msg: Twist, eps: float = 1e-6) -> bool:
    return (
        abs(msg.linear.x) < eps
        and abs(msg.linear.y) < eps
        and abs(msg.linear.z) < eps
        and abs(msg.angular.x) < eps
        and abs(msg.angular.y) < eps
        and abs(msg.angular.z) < eps
    )


def _point_dict(point: PointStamped | None) -> dict[str, Any] | None:
    if point is None:
        return None
    return {
        "x": point.x,
        "y": point.y,
        "z": point.z,
        "frame_id": point.frame_id,
    }


def _odom_dict(odom: Odometry | None) -> dict[str, float] | None:
    if odom is None:
        return None
    return {
        "x": odom.x,
        "y": odom.y,
        "z": odom.z,
        "yaw_deg": math.degrees(odom.yaw),
    }


def _odom_delta(
    first: Odometry | None,
    last: Odometry | None,
) -> dict[str, float] | None:
    if first is None or last is None:
        return None
    return {
        "dx": last.x - first.x,
        "dy": last.y - first.y,
        "dz": last.z - first.z,
        "dyaw_deg": math.degrees(_yaw_delta(first.yaw, last.yaw)),
    }


def _distance_to_click(odom: Odometry | None, click: PointStamped | None) -> float | None:
    if odom is None or click is None:
        return None
    return math.hypot(click.x - odom.x, click.y - odom.y)


@dataclass
class Segment:
    index: int
    start_s: float
    click: dict[str, Any]
    odom_start: dict[str, float] | None
    corrected_odom_start: dict[str, float] | None
    waypoint: dict[str, Any] | None = None
    waypoint_count: int = 0
    path_count: int = 0
    path_nonempty_count: int = 0
    max_path_len: int = 0
    last_path_len: int = 0
    cmd_counts: dict[str, int] = field(default_factory=lambda: {k: 0 for k in CMD_TOPICS})
    nonzero_cmd_counts: dict[str, int] = field(
        default_factory=lambda: {k: 0 for k in CMD_TOPICS}
    )
    max_cmd: dict[str, dict[str, float]] = field(
        default_factory=lambda: {k: {"vxy": 0.0, "wz": 0.0} for k in CMD_TOPICS}
    )
    stop_count: int = 0
    first_nonzero_cmd_s: float | None = None
    last_nonzero_cmd_s: float | None = None
    settled_s: float | None = None
    end_reason: str | None = None
    odom_end: dict[str, float] | None = None
    corrected_odom_end: dict[str, float] | None = None
    odom_delta: dict[str, float] | None = None
    corrected_odom_delta: dict[str, float] | None = None
    distance_to_click_start_m: float | None = None
    distance_to_click_end_m: float | None = None
    min_distance_to_click_m: float | None = None
    min_distance_s: float | None = None
    odom_at_min_distance: dict[str, float] | None = None
    yaw_after_min_distance_deg: float | None = None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=240.0)
    parser.add_argument("--print-period", type=float, default=1.0)
    parser.add_argument("--settle-quiet", type=float, default=2.0)
    parser.add_argument("--json-out", default="")
    args = parser.parse_args()

    lc = lcm.LCM()
    start = time.monotonic()
    next_print = start + args.print_period

    last_odom: Odometry | None = None
    last_corrected_odom: Odometry | None = None
    active: Segment | None = None
    segments: list[Segment] = []

    def finalize(seg: Segment, reason: str) -> None:
        nonlocal active
        if seg.end_reason is not None:
            return
        seg.end_reason = reason
        seg.settled_s = time.monotonic() - start
        seg.odom_end = _odom_dict(last_odom)
        seg.corrected_odom_end = _odom_dict(last_corrected_odom)
        click = PointStamped(
            x=seg.click["x"],
            y=seg.click["y"],
            z=seg.click["z"],
            frame_id=seg.click.get("frame_id", ""),
        )
        seg.distance_to_click_end_m = _distance_to_click(last_odom, click)
        if seg.odom_at_min_distance is not None and last_odom is not None:
            seg.yaw_after_min_distance_deg = math.degrees(
                _yaw_delta(
                    math.radians(seg.odom_at_min_distance["yaw_deg"]),
                    last_odom.yaw,
                )
            )
        # Reconstruct minimal odometry-like deltas from stored dictionaries.
        if seg.odom_start is not None and seg.odom_end is not None:
            seg.odom_delta = {
                "dx": seg.odom_end["x"] - seg.odom_start["x"],
                "dy": seg.odom_end["y"] - seg.odom_start["y"],
                "dz": seg.odom_end["z"] - seg.odom_start["z"],
                "dyaw_deg": math.degrees(
                    _yaw_delta(
                        math.radians(seg.odom_start["yaw_deg"]),
                        math.radians(seg.odom_end["yaw_deg"]),
                    )
                ),
            }
        if seg.corrected_odom_start is not None and seg.corrected_odom_end is not None:
            seg.corrected_odom_delta = {
                "dx": seg.corrected_odom_end["x"] - seg.corrected_odom_start["x"],
                "dy": seg.corrected_odom_end["y"] - seg.corrected_odom_start["y"],
                "dz": seg.corrected_odom_end["z"] - seg.corrected_odom_start["z"],
                "dyaw_deg": math.degrees(
                    _yaw_delta(
                        math.radians(seg.corrected_odom_start["yaw_deg"]),
                        math.radians(seg.corrected_odom_end["yaw_deg"]),
                    )
                ),
            }
        if active is seg:
            active = None

    def on_click(_channel: str, data: bytes) -> None:
        nonlocal active
        if active is not None and active.end_reason is None:
            finalize(active, "superseded_by_new_click")
        point = PointStamped.lcm_decode(data)
        seg = Segment(
            index=len(segments) + 1,
            start_s=time.monotonic() - start,
            click=_point_dict(point) or {},
            odom_start=_odom_dict(last_odom),
            corrected_odom_start=_odom_dict(last_corrected_odom),
            distance_to_click_start_m=_distance_to_click(last_odom, point),
        )
        segments.append(seg)
        active = seg
        print(
            f"CLICK #{seg.index}: ({point.x:+.2f},{point.y:+.2f},{point.z:+.2f}) "
            f"frame={point.frame_id!r}",
            flush=True,
        )

    def on_waypoint(_channel: str, data: bytes) -> None:
        if active is None:
            return
        point = PointStamped.lcm_decode(data)
        active.waypoint = _point_dict(point)
        active.waypoint_count += 1

    def on_path(_channel: str, data: bytes) -> None:
        if active is None:
            return
        msg = NavPath.lcm_decode(data)
        path_len = len(msg.poses)
        active.path_count += 1
        active.last_path_len = path_len
        active.max_path_len = max(active.max_path_len, path_len)
        if path_len > 0:
            active.path_nonempty_count += 1

    def on_odom(kind: str):
        def _cb(_channel: str, data: bytes) -> None:
            nonlocal last_odom, last_corrected_odom
            msg = Odometry.lcm_decode(data)
            if kind == "odom":
                last_odom = msg
                if active is not None and active.end_reason is None:
                    click = PointStamped(
                        x=active.click["x"],
                        y=active.click["y"],
                        z=active.click["z"],
                        frame_id=active.click.get("frame_id", ""),
                    )
                    distance = _distance_to_click(msg, click)
                    if distance is not None and (
                        active.min_distance_to_click_m is None
                        or distance < active.min_distance_to_click_m
                    ):
                        active.min_distance_to_click_m = distance
                        active.min_distance_s = time.monotonic() - start
                        active.odom_at_min_distance = _odom_dict(msg)
            else:
                last_corrected_odom = msg

        return _cb

    def on_stop(_channel: str, data: bytes) -> None:
        if active is None:
            return
        msg = Bool.lcm_decode(data)
        if msg.data:
            active.stop_count += 1

    def on_cmd(name: str):
        def _cb(_channel: str, data: bytes) -> None:
            if active is None:
                return
            msg = Twist.lcm_decode(data)
            active.cmd_counts[name] += 1
            vxy = math.hypot(msg.linear.x, msg.linear.y)
            active.max_cmd[name]["vxy"] = max(active.max_cmd[name]["vxy"], vxy)
            active.max_cmd[name]["wz"] = max(active.max_cmd[name]["wz"], abs(msg.angular.z))
            if not _is_zero_twist(msg):
                now_s = time.monotonic() - start
                active.nonzero_cmd_counts[name] += 1
                if active.first_nonzero_cmd_s is None:
                    active.first_nonzero_cmd_s = now_s
                active.last_nonzero_cmd_s = now_s

        return _cb

    lc.subscribe(CLICK_TOPIC, on_click)
    lc.subscribe(WAYPOINT_TOPIC, on_waypoint)
    lc.subscribe(PATH_TOPIC, on_path)
    lc.subscribe(ODOM_TOPIC, on_odom("odom"))
    lc.subscribe(CORRECTED_ODOM_TOPIC, on_odom("corrected"))
    lc.subscribe(STOP_TOPIC, on_stop)
    for name, topic in CMD_TOPICS.items():
        lc.subscribe(topic, on_cmd(name))

    def maybe_settle() -> None:
        if active is None or active.end_reason is not None:
            return
        if active.last_nonzero_cmd_s is None:
            return
        now_s = time.monotonic() - start
        quiet = now_s - active.last_nonzero_cmd_s >= args.settle_quiet
        path_consumed = active.max_path_len > 1 and active.last_path_len <= 1
        if quiet and path_consumed:
            finalize(active, "settled_path_consumed")

    def status_line() -> str:
        if active is None:
            return f"segments={len(segments)} active=none"
        return (
            f"active=#{active.index} wp={active.waypoint_count} "
            f"path={active.path_count}/{active.path_nonempty_count} "
            f"last_path={active.last_path_len} max_path={active.max_path_len} "
            f"stop={active.stop_count} "
            f"nonzero nav/cmd/tele="
            f"{active.nonzero_cmd_counts['nav']}/"
            f"{active.nonzero_cmd_counts['cmd']}/"
            f"{active.nonzero_cmd_counts['tele']} "
            f"max_vxy={active.max_cmd['cmd']['vxy']:.3f} "
            f"max_wz={active.max_cmd['cmd']['wz']:.3f}"
        )

    while time.monotonic() - start < args.duration:
        lc.handle_timeout(100)
        maybe_settle()
        now = time.monotonic()
        if now >= next_print:
            print(f"{now - start:6.1f}s {status_line()}", flush=True)
            next_print += args.print_period

    if active is not None and active.end_reason is None:
        finalize(active, "monitor_timeout")

    summary = {
        "duration": args.duration,
        "segments": [asdict(seg) for seg in segments],
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

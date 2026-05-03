#!/usr/bin/env python3
"""Live M20 nav convergence monitor.

This subscribes directly on NOS LCM topics so we can separate robot-side
odometry/planner behavior from dimos-viewer display lag.
"""

from __future__ import annotations

import argparse
from collections import deque
from collections.abc import Callable
import csv
from dataclasses import dataclass, field
import math
from pathlib import Path
import time
from typing import Any

from dimos_lcm.geometry_msgs.Twist import Twist
import lcm

from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.Imu import Imu

ODOM_TOPIC = "/odometry#nav_msgs.Odometry"
CORRECTED_ODOM_TOPIC = "/corrected_odometry#nav_msgs.Odometry"
IMU_TOPIC = "/airy_imu_front#sensor_msgs.Imu"
CLICK_TOPIC = "/clicked_point#geometry_msgs.PointStamped"
GOAL_TOPIC = "/goal#geometry_msgs.PointStamped"
WAYPOINT_TOPIC = "/way_point#geometry_msgs.PointStamped"
PATH_TOPIC = "/path#nav_msgs.Path"
GOAL_PATH_TOPIC = "/goal_path#nav_msgs.Path"
CMD_TOPICS = {
    "cmd": "/cmd_vel#geometry_msgs.Twist",
    "nav": "/nav_cmd_vel#geometry_msgs.Twist",
    "tele": "/tele_cmd_vel#geometry_msgs.Twist",
}


def _msg_age_s(msg_ts: float | None) -> float | None:
    if msg_ts is None or msg_ts <= 1_000_000_000:
        return None
    return time.time() - msg_ts


def _yaw_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(b - a), math.cos(b - a))


def _distance_xy(a: Any | None, b: Any | None) -> float | None:
    if a is None or b is None:
        return None
    return math.hypot(float(a.x) - float(b.x), float(a.y) - float(b.y))


def _fmt(value: float | None, digits: int = 2) -> str:
    if value is None or not math.isfinite(value):
        return "n/a"
    return f"{value:.{digits}f}"


def _sign(value: float, eps: float) -> int:
    if value > eps:
        return 1
    if value < -eps:
        return -1
    return 0


@dataclass
class TopicRate:
    count: int = 0
    last_recv_wall: float | None = None
    last_msg_ts: float | None = None
    max_age_s: float | None = None
    recv_times: deque[float] = field(default_factory=lambda: deque(maxlen=200))

    def mark(self, msg_ts: float | None = None) -> None:
        now = time.monotonic()
        self.count += 1
        self.last_recv_wall = now
        self.last_msg_ts = msg_ts
        self.recv_times.append(now)
        age = _msg_age_s(msg_ts)
        if age is not None:
            self.max_age_s = age if self.max_age_s is None else max(self.max_age_s, age)

    def hz(self, window_s: float = 3.0) -> float:
        now = time.monotonic()
        recent = [t for t in self.recv_times if now - t <= window_s]
        if len(recent) < 2:
            return 0.0
        return (len(recent) - 1) / max(recent[-1] - recent[0], 1e-6)

    def stale_s(self) -> float | None:
        if self.last_recv_wall is None:
            return None
        return time.monotonic() - self.last_recv_wall

    def age_s(self) -> float | None:
        return _msg_age_s(self.last_msg_ts)


@dataclass
class CommandStats:
    count: int = 0
    nonzero_count: int = 0
    vx_flips: int = 0
    vy_flips: int = 0
    wz_flips: int = 0
    last_vx_sign: int = 0
    last_vy_sign: int = 0
    last_wz_sign: int = 0

    def mark(self, msg: Twist, eps: float) -> None:
        self.count += 1
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        wz = float(msg.angular.z)
        if abs(vx) > eps or abs(vy) > eps or abs(wz) > eps:
            self.nonzero_count += 1
        self.vx_flips += self._flip("x", _sign(vx, eps))
        self.vy_flips += self._flip("y", _sign(vy, eps))
        self.wz_flips += self._flip("z", _sign(wz, eps))

    def _flip(self, axis: str, current: int) -> int:
        if current == 0:
            return 0
        attr = {
            "x": "last_vx_sign",
            "y": "last_vy_sign",
            "z": "last_wz_sign",
        }[axis]
        previous = getattr(self, attr)
        setattr(self, attr, current)
        return 1 if previous != 0 and previous != current else 0


@dataclass
class State:
    odom: Odometry | None = None
    corrected_odom: Odometry | None = None
    imu: Imu | None = None
    click: PointStamped | None = None
    goal: PointStamped | None = None
    waypoint: PointStamped | None = None
    path: NavPath | None = None
    goal_path: NavPath | None = None
    cmd: dict[str, Twist | None] = field(default_factory=lambda: {k: None for k in CMD_TOPICS})
    rates: dict[str, TopicRate] = field(
        default_factory=lambda: {
            "odom": TopicRate(),
            "corrected_odom": TopicRate(),
            "imu": TopicRate(),
            "click": TopicRate(),
            "goal": TopicRate(),
            "waypoint": TopicRate(),
            "path": TopicRate(),
            "goal_path": TopicRate(),
            **{f"{name}_cmd": TopicRate() for name in CMD_TOPICS},
        }
    )
    cmd_stats: dict[str, CommandStats] = field(
        default_factory=lambda: {name: CommandStats() for name in CMD_TOPICS}
    )
    first_corrected_odom: Odometry | None = None
    last_goal_distance: float | None = None
    min_goal_distance: float | None = None
    min_goal_elapsed_s: float | None = None


def _path_len(path: NavPath | None) -> int:
    return 0 if path is None else len(path.poses)


def _path_last(path: NavPath | None) -> Any | None:
    if path is None or not path.poses:
        return None
    return path.poses[-1]


def _point_row(prefix: str, point: Any | None) -> dict[str, float | str]:
    return {
        f"{prefix}_x": "" if point is None else float(point.x),
        f"{prefix}_y": "" if point is None else float(point.y),
        f"{prefix}_z": "" if point is None else float(point.z),
    }


def _twist_row(prefix: str, twist: Twist | None) -> dict[str, float | str]:
    return {
        f"{prefix}_vx": "" if twist is None else float(twist.linear.x),
        f"{prefix}_vy": "" if twist is None else float(twist.linear.y),
        f"{prefix}_wz": "" if twist is None else float(twist.angular.z),
    }


def _make_row(state: State, start: float) -> dict[str, float | int | str]:
    elapsed = time.monotonic() - start
    active_goal = state.goal or state.click
    path_last = _path_last(state.path)
    goal_path_last = _path_last(state.goal_path)
    odom_ref = state.corrected_odom or state.odom
    goal_dist = _distance_xy(odom_ref, active_goal)
    waypoint_dist = _distance_xy(odom_ref, state.waypoint)
    path_end_dist = _distance_xy(odom_ref, path_last)

    row: dict[str, float | int | str] = {
        "wall_time": time.time(),
        "elapsed_s": elapsed,
        "odom_hz": state.rates["odom"].hz(),
        "corrected_odom_hz": state.rates["corrected_odom"].hz(),
        "imu_hz": state.rates["imu"].hz(),
        "odom_age_s": state.rates["odom"].age_s() or "",
        "corrected_odom_age_s": state.rates["corrected_odom"].age_s() or "",
        "imu_age_s": state.rates["imu"].age_s() or "",
        "odom_stale_s": state.rates["odom"].stale_s() or "",
        "corrected_odom_stale_s": state.rates["corrected_odom"].stale_s() or "",
        "path_len": _path_len(state.path),
        "goal_path_len": _path_len(state.goal_path),
        "dist_goal_m": "" if goal_dist is None else goal_dist,
        "dist_waypoint_m": "" if waypoint_dist is None else waypoint_dist,
        "dist_path_end_m": "" if path_end_dist is None else path_end_dist,
        "min_goal_dist_m": "" if state.min_goal_distance is None else state.min_goal_distance,
        "cmd_wz_flips": state.cmd_stats["cmd"].wz_flips,
        "nav_wz_flips": state.cmd_stats["nav"].wz_flips,
        "tele_nonzero_count": state.cmd_stats["tele"].nonzero_count,
    }

    for name, odom in (("odom", state.odom), ("corrected", state.corrected_odom)):
        row.update(_point_row(name, odom))
        row[f"{name}_yaw_deg"] = "" if odom is None else math.degrees(odom.yaw)
    for name, point in (
        ("click", state.click),
        ("goal", state.goal),
        ("waypoint", state.waypoint),
        ("path_end", path_last),
        ("goal_path_end", goal_path_last),
    ):
        row.update(_point_row(name, point))
    for name, twist in state.cmd.items():
        row.update(_twist_row(name, twist))
    return row


def _print_summary(state: State, start: float) -> None:
    elapsed = time.monotonic() - start
    active_goal = state.goal or state.click
    odom_ref = state.corrected_odom or state.odom
    path_last = _path_last(state.path)
    goal_dist = _distance_xy(odom_ref, active_goal)
    waypoint_dist = _distance_xy(odom_ref, state.waypoint)
    path_end_dist = _distance_xy(odom_ref, path_last)
    cmd = state.cmd.get("cmd")
    nav = state.cmd.get("nav")
    cmd_text = (
        "cmd=n/a"
        if cmd is None
        else f"cmd=({cmd.linear.x:+.2f},{cmd.linear.y:+.2f},{cmd.angular.z:+.2f})"
    )
    nav_text = (
        "nav=n/a"
        if nav is None
        else f"nav=({nav.linear.x:+.2f},{nav.linear.y:+.2f},{nav.angular.z:+.2f})"
    )
    print(
        f"{elapsed:6.1f}s "
        f"odom_hz={state.rates['odom'].hz():4.1f} "
        f"corr_hz={state.rates['corrected_odom'].hz():4.1f} "
        f"imu_hz={state.rates['imu'].hz():5.1f} "
        f"odom_age={_fmt(state.rates['odom'].age_s(), 3)}s "
        f"corr_age={_fmt(state.rates['corrected_odom'].age_s(), 3)}s "
        f"imu_age={_fmt(state.rates['imu'].age_s(), 3)}s "
        f"goal={_fmt(goal_dist)}m min={_fmt(state.min_goal_distance)}m "
        f"wp={_fmt(waypoint_dist)}m path_end={_fmt(path_end_dist)}m "
        f"path={_path_len(state.path)} goal_path={_path_len(state.goal_path)} "
        f"{cmd_text} {nav_text} "
        f"flips(cmd/nav wz)={state.cmd_stats['cmd'].wz_flips}/"
        f"{state.cmd_stats['nav'].wz_flips} "
        f"tele_nonzero={state.cmd_stats['tele'].nonzero_count}",
        flush=True,
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--print-period", type=float, default=1.0)
    parser.add_argument("--sample-period", type=float, default=0.2)
    parser.add_argument("--csv-out", default="/tmp/m20_nav_convergence.csv")
    parser.add_argument("--cmd-eps", type=float, default=1e-4)
    parser.add_argument("--lcm-url", default="")
    parser.add_argument(
        "--imu-topic",
        default=IMU_TOPIC,
        help="IMU topic to monitor for sensor-delivery age/rate.",
    )
    args = parser.parse_args()

    lc = lcm.LCM(args.lcm_url) if args.lcm_url else lcm.LCM()
    state = State()
    start = time.monotonic()
    next_print = start + args.print_period
    next_sample = start

    def on_odom(kind: str) -> Callable[[str, bytes], None]:
        def _cb(_channel: str, data: bytes) -> None:
            msg = Odometry.lcm_decode(data)
            if kind == "odom":
                state.odom = msg
                state.rates["odom"].mark(msg.ts)
            else:
                state.corrected_odom = msg
                if state.first_corrected_odom is None:
                    state.first_corrected_odom = msg
                state.rates["corrected_odom"].mark(msg.ts)
            active_goal = state.goal or state.click
            odom_ref = state.corrected_odom or state.odom
            distance = _distance_xy(odom_ref, active_goal)
            if distance is not None:
                state.last_goal_distance = distance
                if state.min_goal_distance is None or distance < state.min_goal_distance:
                    state.min_goal_distance = distance
                    state.min_goal_elapsed_s = time.monotonic() - start

        return _cb

    def on_imu(_channel: str, data: bytes) -> None:
        msg = Imu.lcm_decode(data)
        state.imu = msg
        state.rates["imu"].mark(msg.ts)

    def on_point(name: str) -> Callable[[str, bytes], None]:
        def _cb(_channel: str, data: bytes) -> None:
            msg = PointStamped.lcm_decode(data)
            setattr(state, name, msg)
            state.rates[name].mark(msg.ts)
            if name in {"click", "goal"}:
                state.min_goal_distance = None
                state.min_goal_elapsed_s = None

        return _cb

    def on_path(name: str) -> Callable[[str, bytes], None]:
        def _cb(_channel: str, data: bytes) -> None:
            msg = NavPath.lcm_decode(data)
            setattr(state, name, msg)
            state.rates[name].mark(msg.ts)

        return _cb

    def on_cmd(name: str) -> Callable[[str, bytes], None]:
        def _cb(_channel: str, data: bytes) -> None:
            msg = Twist.lcm_decode(data)
            state.cmd[name] = msg
            state.rates[f"{name}_cmd"].mark(None)
            state.cmd_stats[name].mark(msg, args.cmd_eps)

        return _cb

    lc.subscribe(ODOM_TOPIC, on_odom("odom"))
    lc.subscribe(CORRECTED_ODOM_TOPIC, on_odom("corrected_odom"))
    lc.subscribe(args.imu_topic, on_imu)
    lc.subscribe(CLICK_TOPIC, on_point("click"))
    lc.subscribe(GOAL_TOPIC, on_point("goal"))
    lc.subscribe(WAYPOINT_TOPIC, on_point("waypoint"))
    lc.subscribe(PATH_TOPIC, on_path("path"))
    lc.subscribe(GOAL_PATH_TOPIC, on_path("goal_path"))
    for name, topic in CMD_TOPICS.items():
        lc.subscribe(topic, on_cmd(name))

    csv_path = Path(args.csv_out)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames: list[str] | None = None
    with csv_path.open("w", newline="") as f:
        writer: csv.DictWriter[str] | None = None
        while time.monotonic() - start < args.duration:
            lc.handle_timeout(50)
            now = time.monotonic()
            if now >= next_sample:
                row = _make_row(state, start)
                if writer is None:
                    fieldnames = list(row.keys())
                    writer = csv.DictWriter(f, fieldnames=fieldnames)
                    writer.writeheader()
                writer.writerow(row)
                f.flush()
                next_sample += args.sample_period
            if now >= next_print:
                _print_summary(state, start)
                next_print += args.print_period

    print("--- summary ---", flush=True)
    print(f"csv={csv_path}", flush=True)
    print(
        "counts " + " ".join(f"{name}={rate.count}" for name, rate in sorted(state.rates.items())),
        flush=True,
    )
    print(
        "max_age_s "
        + " ".join(
            f"{name}={_fmt(rate.max_age_s, 3)}"
            for name, rate in sorted(state.rates.items())
            if rate.max_age_s is not None
        ),
        flush=True,
    )
    if state.first_corrected_odom is not None and state.corrected_odom is not None:
        print(
            "corrected_delta "
            f"dx={state.corrected_odom.x - state.first_corrected_odom.x:+.2f} "
            f"dy={state.corrected_odom.y - state.first_corrected_odom.y:+.2f} "
            f"dyaw={math.degrees(_yaw_delta(state.first_corrected_odom.yaw, state.corrected_odom.yaw)):+.1f}deg",
            flush=True,
        )
    print(
        "cmd_flips "
        + " ".join(
            f"{name}:vx={stats.vx_flips},vy={stats.vy_flips},wz={stats.wz_flips},nonzero={stats.nonzero_count}"
            for name, stats in sorted(state.cmd_stats.items())
        ),
        flush=True,
    )
    if fieldnames is None:
        print("no rows written", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

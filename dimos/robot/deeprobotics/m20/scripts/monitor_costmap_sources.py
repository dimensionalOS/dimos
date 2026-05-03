#!/usr/bin/env python3
"""Attribute near-robot M20 costmap obstacles to upstream source topics."""

from __future__ import annotations

import argparse
from collections import deque
from collections.abc import Callable
import csv
from dataclasses import dataclass, field
import math
from pathlib import Path
import time

from dimos_lcm.sensor_msgs.PointCloud2 import PointCloud2 as LCMPointCloud2
import lcm
import numpy as np

from dimos.msgs.nav_msgs.Odometry import Odometry

ODOM_TOPIC = "/odometry#nav_msgs.Odometry"


@dataclass(frozen=True)
class CloudTopic:
    name: str
    channel: str
    world_frame: bool
    intensity_is_height: bool = False
    all_points_are_obstacles: bool = False


CLOUD_TOPICS: dict[str, CloudTopic] = {
    "registered_scan": CloudTopic(
        "registered_scan",
        "/registered_scan#sensor_msgs.PointCloud2",
        world_frame=True,
    ),
    "terrain_map": CloudTopic(
        "terrain_map",
        "/terrain_map#sensor_msgs.PointCloud2",
        world_frame=True,
        intensity_is_height=True,
    ),
    "terrain_map_ext": CloudTopic(
        "terrain_map_ext",
        "/terrain_map_ext#sensor_msgs.PointCloud2",
        world_frame=True,
        intensity_is_height=True,
    ),
    "costmap_cloud": CloudTopic(
        "costmap_cloud",
        "/costmap_cloud#sensor_msgs.PointCloud2",
        world_frame=True,
        all_points_are_obstacles=True,
    ),
    "obstacle_cloud": CloudTopic(
        "obstacle_cloud",
        "/obstacle_cloud#sensor_msgs.PointCloud2",
        world_frame=False,
        intensity_is_height=True,
    ),
    "raw_points": CloudTopic(
        "raw_points",
        "/raw_points#sensor_msgs.PointCloud2",
        world_frame=False,
    ),
}

DEFAULT_TOPICS = (
    "registered_scan",
    "terrain_map",
    "terrain_map_ext",
    "costmap_cloud",
    "obstacle_cloud",
)


@dataclass
class TopicRate:
    received: int = 0
    processed: int = 0
    recv_times: deque[float] = field(default_factory=lambda: deque(maxlen=200))
    last_processed_mono: float = 0.0

    def mark_received(self) -> None:
        self.received += 1
        self.recv_times.append(time.monotonic())

    def mark_processed(self) -> None:
        self.processed += 1
        self.last_processed_mono = time.monotonic()

    def hz(self, window_s: float = 3.0) -> float:
        now = time.monotonic()
        recent = [t for t in self.recv_times if now - t <= window_s]
        if len(recent) < 2:
            return 0.0
        return (len(recent) - 1) / max(recent[-1] - recent[0], 1e-6)


@dataclass
class RobotPose:
    x: float
    y: float
    z: float
    yaw: float
    msg_ts: float
    recv_mono: float


@dataclass
class State:
    odom: RobotPose | None = None
    latest: dict[str, dict[str, float | int | str]] = field(default_factory=dict)
    rates: dict[str, TopicRate] = field(
        default_factory=lambda: {name: TopicRate() for name in CLOUD_TOPICS}
    )


def _msg_ts(msg: LCMPointCloud2) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nsec) / 1_000_000_000


def _age_s(msg_ts: float) -> float | None:
    if msg_ts <= 1_000_000_000:
        return None
    return time.time() - msg_ts


def _fmt(value: float | int | str | None, digits: int = 2) -> str:
    if value is None or value == "":
        return "n/a"
    if isinstance(value, str):
        return value
    if not math.isfinite(float(value)):
        return "n/a"
    return f"{float(value):.{digits}f}"


def _radius_key(radius: float) -> str:
    return str(radius).replace(".", "p")


def _odom_fields(odom: RobotPose | None) -> dict[str, float | str]:
    if odom is None:
        return {
            "odom_x": "",
            "odom_y": "",
            "odom_z": "",
            "odom_yaw_rad": "",
            "odom_age_s": "",
        }
    return {
        "odom_x": odom.x,
        "odom_y": odom.y,
        "odom_z": odom.z,
        "odom_yaw_rad": odom.yaw,
        "odom_age_s": time.monotonic() - odom.recv_mono,
    }


def _field_offset(msg: LCMPointCloud2, name: str) -> int | None:
    for field_msg in msg.fields:
        if field_msg.name == name:
            return int(field_msg.offset)
    return None


def _extract_float32_field(
    raw_data: bytes,
    *,
    num_points: int,
    point_step: int,
    offset: int,
    bigendian: bool,
) -> np.ndarray:
    if offset < 0 or offset + 4 > point_step:
        raise ValueError(f"invalid float32 field offset {offset} for point_step {point_step}")
    dtype = ">f4" if bigendian else "<f4"
    raw = np.frombuffer(raw_data, dtype=np.uint8, count=num_points * point_step)
    field_bytes = raw.reshape(num_points, point_step)[:, offset : offset + 4]
    return np.ascontiguousarray(field_bytes).view(dtype).reshape(num_points).astype(np.float32)


def _decode_cloud(data: bytes) -> tuple[str, float, np.ndarray, np.ndarray | None]:
    msg = LCMPointCloud2.lcm_decode(data)
    num_points = int(msg.width) * int(msg.height)
    frame_id = msg.header.frame_id
    msg_ts = _msg_ts(msg)
    if num_points == 0:
        return frame_id, msg_ts, np.zeros((0, 3), dtype=np.float32), None

    x_offset = _field_offset(msg, "x")
    y_offset = _field_offset(msg, "y")
    z_offset = _field_offset(msg, "z")
    if x_offset is None or y_offset is None or z_offset is None:
        raise ValueError("PointCloud2 missing x/y/z fields")

    raw_data = msg.data if isinstance(msg.data, bytes) else bytes(msg.data)
    point_step = int(msg.point_step)
    bigendian = bool(msg.is_bigendian)
    points = np.column_stack(
        [
            _extract_float32_field(
                raw_data,
                num_points=num_points,
                point_step=point_step,
                offset=x_offset,
                bigendian=bigendian,
            ),
            _extract_float32_field(
                raw_data,
                num_points=num_points,
                point_step=point_step,
                offset=y_offset,
                bigendian=bigendian,
            ),
            _extract_float32_field(
                raw_data,
                num_points=num_points,
                point_step=point_step,
                offset=z_offset,
                bigendian=bigendian,
            ),
        ]
    ).astype(np.float32, copy=False)

    intensity: np.ndarray | None = None
    intensity_offset = _field_offset(msg, "intensity")
    if intensity_offset is not None:
        intensity = _extract_float32_field(
            raw_data,
            num_points=num_points,
            point_step=point_step,
            offset=intensity_offset,
            bigendian=bigendian,
        )
    return frame_id, msg_ts, points, intensity


def _robot_frame_xy(
    points: np.ndarray,
    *,
    spec: CloudTopic,
    odom: RobotPose | None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, str]:
    if spec.world_frame and odom is not None:
        dx = points[:, 0] - odom.x
        dy = points[:, 1] - odom.y
        cos_yaw = math.cos(odom.yaw)
        sin_yaw = math.sin(odom.yaw)
        body_x = cos_yaw * dx + sin_yaw * dy
        body_y = -sin_yaw * dx + cos_yaw * dy
        z_rel = points[:, 2] - odom.z
        return body_x, body_y, z_rel, "odom"
    return points[:, 0], points[:, 1], points[:, 2], "origin"


def _nearest_row(
    points: np.ndarray,
    intensity: np.ndarray | None,
    mask: np.ndarray,
    dist: np.ndarray,
) -> dict[str, float | str]:
    if not np.any(mask):
        return {
            "nearest_obs_dist_m": "",
            "nearest_obs_x": "",
            "nearest_obs_y": "",
            "nearest_obs_z": "",
            "nearest_obs_intensity": "",
        }
    masked_indices = np.flatnonzero(mask)
    idx = int(masked_indices[int(np.argmin(dist[masked_indices]))])
    return {
        "nearest_obs_dist_m": float(dist[idx]),
        "nearest_obs_x": float(points[idx, 0]),
        "nearest_obs_y": float(points[idx, 1]),
        "nearest_obs_z": float(points[idx, 2]),
        "nearest_obs_intensity": ""
        if intensity is None or len(intensity) != len(points)
        else float(intensity[idx]),
    }


def _cloud_row(
    *,
    spec: CloudTopic,
    frame_id: str,
    msg_ts: float,
    points: np.ndarray,
    intensity: np.ndarray | None,
    odom: RobotPose | None,
    radii: list[float],
    start: float,
    obstacle_intensity_threshold: float,
    relative_z_threshold: float,
) -> dict[str, float | int | str]:
    finite = np.isfinite(points).all(axis=1)
    if intensity is not None and len(intensity) == len(points):
        finite &= np.isfinite(intensity)
    points = points[finite]
    if intensity is not None and len(intensity) == len(finite):
        intensity = intensity[finite]

    if len(points) == 0:
        row: dict[str, float | int | str] = {
            "elapsed_s": time.monotonic() - start,
            "wall_time": time.time(),
            "topic": spec.name,
            "frame_id": frame_id,
            "frame_mode": "empty",
            "msg_ts": msg_ts,
            "age_s": _age_s(msg_ts) or "",
            **_odom_fields(odom),
            "point_count": 0,
            "min_dist_m": "",
            "max_intensity": "",
            "mean_intensity": "",
            "max_z_rel": "",
        }
        for radius in radii:
            key = _radius_key(radius)
            row[f"near_{key}m"] = 0
            row[f"obs_{key}m"] = 0
            row[f"high_z_{key}m"] = 0
        row.update(_nearest_row(points, intensity, np.zeros(0, dtype=bool), np.zeros(0)))
        return row

    body_x, body_y, z_rel, frame_mode = _robot_frame_xy(points, spec=spec, odom=odom)
    dist = np.hypot(body_x, body_y)
    high_z = z_rel >= relative_z_threshold

    if spec.all_points_are_obstacles:
        obstacle = np.ones(len(points), dtype=bool)
    elif spec.intensity_is_height and intensity is not None and len(intensity) == len(points):
        obstacle = intensity >= obstacle_intensity_threshold
    else:
        obstacle = high_z

    row = {
        "elapsed_s": time.monotonic() - start,
        "wall_time": time.time(),
        "topic": spec.name,
        "frame_id": frame_id,
        "frame_mode": frame_mode,
        "msg_ts": msg_ts,
        "age_s": _age_s(msg_ts) or "",
        **_odom_fields(odom),
        "point_count": len(points),
        "min_dist_m": float(np.min(dist)),
        "max_intensity": ""
        if intensity is None or len(intensity) != len(points)
        else float(np.max(intensity)),
        "mean_intensity": ""
        if intensity is None or len(intensity) != len(points)
        else float(np.mean(intensity)),
        "max_z_rel": float(np.max(z_rel)),
    }
    for radius in radii:
        key = _radius_key(radius)
        near = dist <= radius
        row[f"near_{key}m"] = int(np.count_nonzero(near))
        row[f"obs_{key}m"] = int(np.count_nonzero(near & obstacle))
        row[f"high_z_{key}m"] = int(np.count_nonzero(near & high_z))
    row.update(_nearest_row(points, intensity, obstacle, dist))
    return row


def _fieldnames(radii: list[float]) -> list[str]:
    names = [
        "elapsed_s",
        "wall_time",
        "topic",
        "frame_id",
        "frame_mode",
        "msg_ts",
        "age_s",
        "odom_x",
        "odom_y",
        "odom_z",
        "odom_yaw_rad",
        "odom_age_s",
        "point_count",
        "min_dist_m",
        "max_intensity",
        "mean_intensity",
        "max_z_rel",
        "nearest_obs_dist_m",
        "nearest_obs_x",
        "nearest_obs_y",
        "nearest_obs_z",
        "nearest_obs_intensity",
    ]
    for radius in radii:
        key = _radius_key(radius)
        names.extend([f"near_{key}m", f"obs_{key}m", f"high_z_{key}m"])
    return names


def _print_summary(
    *,
    state: State,
    topics: list[CloudTopic],
    radii: list[float],
    start: float,
) -> None:
    elapsed = time.monotonic() - start
    if state.odom is None:
        odom_text = "odom=n/a"
    else:
        odom_age = time.monotonic() - state.odom.recv_mono
        odom_text = (
            f"odom=({state.odom.x:+.2f},{state.odom.y:+.2f},"
            f"yaw={math.degrees(state.odom.yaw):+.1f}deg age={odom_age:.2f}s)"
        )
    print(f"{elapsed:6.1f}s {odom_text}", flush=True)

    far_key = _radius_key(max(radii))
    for spec in topics:
        row = state.latest.get(spec.name)
        rate = state.rates[spec.name]
        if row is None:
            print(f"  {spec.name:16} hz={rate.hz():4.1f} processed=0", flush=True)
            continue
        print(
            f"  {spec.name:16} hz={rate.hz():4.1f} "
            f"age={_fmt(row.get('age_s'), 3)}s frame={row.get('frame_id', '')!s:>6} "
            f"mode={row.get('frame_mode', '')!s:>6} n={row['point_count']} "
            f"near{max(radii):.1f}={row[f'near_{far_key}m']} "
            f"obs{max(radii):.1f}={row[f'obs_{far_key}m']} "
            f"min_d={_fmt(row.get('min_dist_m'))} "
            f"max_i={_fmt(row.get('max_intensity'), 3)} "
            f"nearest_obs={_fmt(row.get('nearest_obs_dist_m'))}m",
            flush=True,
        )


def _parse_topics(names: list[str]) -> list[CloudTopic]:
    unknown = sorted(set(names) - set(CLOUD_TOPICS))
    if unknown:
        valid = ", ".join(sorted(CLOUD_TOPICS))
        raise SystemExit(f"unknown topic name(s): {', '.join(unknown)}; valid: {valid}")
    return [CLOUD_TOPICS[name] for name in names]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--print-period", type=float, default=1.0)
    parser.add_argument("--min-cloud-period", type=float, default=0.5)
    parser.add_argument("--csv-out", default="/tmp/m20_costmap_sources.csv")
    parser.add_argument("--lcm-url", default="")
    parser.add_argument("--radii", type=float, nargs="+", default=[0.5, 0.8, 1.2])
    parser.add_argument(
        "--topics",
        nargs="+",
        default=list(DEFAULT_TOPICS),
        help=f"Topic names to monitor. Valid: {', '.join(sorted(CLOUD_TOPICS))}",
    )
    parser.add_argument(
        "--obstacle-intensity-threshold",
        type=float,
        default=0.2,
        help="TerrainAnalysis intensity at/above this value counts as obstacle-like.",
    )
    parser.add_argument(
        "--relative-z-threshold",
        type=float,
        default=-0.20,
        help="Non-intensity clouds count as obstacle-like at/above this robot-relative z.",
    )
    args = parser.parse_args()

    radii = sorted(float(radius) for radius in args.radii)
    topics = _parse_topics(args.topics)
    lc = lcm.LCM(args.lcm_url) if args.lcm_url else lcm.LCM()
    state = State()
    start = time.monotonic()
    next_print = start + args.print_period

    csv_path = Path(args.csv_out)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    csv_file = csv_path.open("w", newline="")
    writer = csv.DictWriter(csv_file, fieldnames=_fieldnames(radii))
    writer.writeheader()

    def on_odom(_channel: str, data: bytes) -> None:
        msg = Odometry.lcm_decode(data)
        state.odom = RobotPose(
            x=float(msg.x),
            y=float(msg.y),
            z=float(msg.z),
            yaw=float(msg.yaw),
            msg_ts=float(msg.ts),
            recv_mono=time.monotonic(),
        )

    def on_cloud(spec: CloudTopic) -> Callable[[str, bytes], None]:
        def _cb(_channel: str, data: bytes) -> None:
            rate = state.rates[spec.name]
            rate.mark_received()
            now = time.monotonic()
            if now - rate.last_processed_mono < args.min_cloud_period:
                return
            rate.mark_processed()
            try:
                frame_id, msg_ts, points, intensity = _decode_cloud(data)
                row = _cloud_row(
                    spec=spec,
                    frame_id=frame_id,
                    msg_ts=msg_ts,
                    points=points,
                    intensity=intensity,
                    odom=state.odom,
                    radii=radii,
                    start=start,
                    obstacle_intensity_threshold=args.obstacle_intensity_threshold,
                    relative_z_threshold=args.relative_z_threshold,
                )
            except Exception as exc:
                row = {
                    "elapsed_s": time.monotonic() - start,
                    "wall_time": time.time(),
                    "topic": spec.name,
                    "frame_id": "decode_error",
                    "frame_mode": str(exc),
                    "msg_ts": "",
                    "age_s": "",
                    **_odom_fields(state.odom),
                    "point_count": 0,
                    "min_dist_m": "",
                    "max_intensity": "",
                    "mean_intensity": "",
                    "max_z_rel": "",
                    "nearest_obs_dist_m": "",
                    "nearest_obs_x": "",
                    "nearest_obs_y": "",
                    "nearest_obs_z": "",
                    "nearest_obs_intensity": "",
                }
                for radius in radii:
                    key = _radius_key(radius)
                    row[f"near_{key}m"] = 0
                    row[f"obs_{key}m"] = 0
                    row[f"high_z_{key}m"] = 0
            state.latest[spec.name] = row
            writer.writerow(row)
            csv_file.flush()

        return _cb

    lc.subscribe(ODOM_TOPIC, on_odom)
    for spec in topics:
        lc.subscribe(spec.channel, on_cloud(spec))

    try:
        while time.monotonic() - start < args.duration:
            lc.handle_timeout(50)
            now = time.monotonic()
            if now >= next_print:
                _print_summary(state=state, topics=topics, radii=radii, start=start)
                next_print += args.print_period
    finally:
        csv_file.close()

    print("--- summary ---", flush=True)
    print(f"csv={csv_path}", flush=True)
    for spec in topics:
        rate = state.rates[spec.name]
        print(
            f"{spec.name}: received={rate.received} processed={rate.processed} hz={rate.hz():.1f}",
            flush=True,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

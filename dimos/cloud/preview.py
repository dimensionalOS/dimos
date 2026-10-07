# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Spatial preview of a recording for the console's Data inspector.

Built on the uploading machine, which already has the recording open, so the preview
is ready when the upload completes. One JSON document: the odometry trajectory, a
voxel-downsampled LiDAR map, timed LiDAR scans and timed camera thumbnails. Points are
base64 little-endian int16 triplets in units of ``scale`` metres around ``origin``.
"""

from __future__ import annotations

import base64
from typing import TYPE_CHECKING, Any

import numpy as np

if TYPE_CHECKING:
    from dimos.memory.store.base import Store

FORMAT = "dimos-spatial-preview-v2"
SCALE = 0.02  # metres per int16 step: +-655 m around the origin
WORLD_FRAMES = {"world", "map", "odom"}


def _kind(stream: Any) -> str | None:
    """lidar / camera / pose, from the payload type of the stream's first item."""
    try:
        first = stream.first()
    except LookupError:  # empty stream
        return None
    name = type(first.data).__name__
    return {
        "PointCloud2": "lidar",
        "Image": "camera",
        "PoseStamped": "pose",
        "Odometry": "pose",
    }.get(name)


def _position_yaw(data: Any) -> tuple[np.ndarray, float]:
    pose = data.pose.pose if hasattr(data, "pose") and hasattr(data.pose, "pose") else data
    p, q = pose.position, pose.orientation
    yaw = float(np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    return np.array([p.x, p.y, p.z], dtype=np.float64), yaw


def _rotation(q: Any) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _voxel(points: np.ndarray, size: float) -> np.ndarray:
    if len(points) == 0:
        return points
    keys = np.floor(points / size).astype(np.int64)
    _, keep = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(keep)]


def _cap(points: np.ndarray, n: int, rng: np.random.Generator) -> np.ndarray:
    return points if len(points) <= n else points[rng.choice(len(points), n, replace=False)]


def _pack(points: np.ndarray, origin: np.ndarray) -> str:
    q = np.clip(np.round((points - origin) / SCALE), -32767, 32767).astype("<i2")
    return base64.b64encode(q.tobytes()).decode()


def _pick(n_items: int, n: int) -> list[int]:
    return (
        sorted({round(i) for i in np.linspace(0, n_items - 1, min(n, n_items))}) if n_items else []
    )


def build(
    store: Store,
    *,
    frames: int = 24,
    map_scans: int = 300,
    scan_points: int = 4000,
    map_points: int = 40000,
    map_voxel: float = 0.1,
    thumb_width: int = 320,
) -> dict[str, Any] | None:
    """Preview of a recording, or None when it has no LiDAR, camera or pose stream."""
    rng = np.random.default_rng(0)
    streams = {}
    for name in store.list_streams():
        kind = _kind(store.streams[name])
        if kind and kind not in streams:
            streams[kind] = store.streams[name]
    if not streams:
        return None

    def sample(
        kind: str, picks: set[int]
    ) -> tuple[list[Any], list[tuple[float, Any]], float, float]:
        """One pass over a stream keeping only the picked observations (payloads are
        lazy; nothing unused is retained), every observation's attached pose, and the
        first/last timestamps."""
        kept: list[Any] = []
        attached: list[tuple[float, Any]] = []
        first = last = float("nan")
        if kind not in streams:
            return kept, attached, first, last
        for i, obs in enumerate(streams[kind]):
            first = obs.ts if i == 0 else first
            last = obs.ts
            if getattr(obs, "pose", None) is not None:
                attached.append((obs.ts, obs.pose))
            if i in picks:
                kept.append((i, obs))
        return kept, attached, first, last

    n_lidar = streams["lidar"].count() if "lidar" in streams else 0
    shown = set(_pick(n_lidar, frames))
    lidar, lidar_poses, l0, l1 = sample("lidar", shown | set(_pick(n_lidar, map_scans)))
    n_cam = streams["camera"].count() if "camera" in streams else 0
    camera, _, c0, c1 = sample("camera", set(_pick(n_cam, frames)))

    # Sensor-frame clouds (e.g. a mid360 on its mount) carry the sensor's pose per
    # observation: then each scan is placed with its own pose and the trajectory comes
    # from those poses, so the map and the path share one frame even when the recording
    # has other odometry streams. Scans that cannot be placed in that frame are skipped.
    sensor_frame = bool(lidar_poses) and any(
        getattr(o.data, "frame_id", "world") not in WORLD_FRAMES for _, o in lidar
    )
    odom: list[tuple[float, Any]] = []
    if sensor_frame:
        t_pose = np.array([t for t, _ in lidar_poses])
        pos_yaw = [_position_yaw(p) for _, p in lidar_poses]
    else:
        odom = [(o.ts, o.data) for o in streams["pose"]] if "pose" in streams else []
        t_pose = np.array([t for t, _ in odom])
        pos_yaw = [_position_yaw(d) for _, d in odom]
    p0 = [float(t_pose[0])] if len(t_pose) else []
    p1 = [float(t_pose[-1])] if len(t_pose) else []
    t0 = float(np.nanmin([l0, c0, *p0, np.inf]))
    t1 = float(np.nanmax([l1, c1, *p1, t0]))

    def pose_at(t: float) -> Any:
        return odom[int(np.clip(np.searchsorted(t_pose, t), 0, len(odom) - 1))][1]

    def world_points(obs: Any) -> np.ndarray | None:
        pts = np.asarray(obs.data.points_f32(), dtype=np.float64).reshape(-1, 3)
        pts = pts[np.isfinite(pts).all(axis=1)]
        if getattr(obs.data, "frame_id", "world") in WORLD_FRAMES:
            return None if sensor_frame else pts
        if sensor_frame:
            pose = getattr(obs, "pose", None)
        else:
            pose = pose_at(obs.ts) if len(t_pose) else None
        if pose is None:
            return None
        pose = pose.pose.pose if hasattr(pose, "pose") and hasattr(pose.pose, "pose") else pose
        p = pose.position
        return np.asarray(pts @ _rotation(pose.orientation).T + np.array([p.x, p.y, p.z]))

    # ceilings hide the floor plan from above: keep -0.5 m .. +2 m around the robot's height
    z_robot = float(np.median([p[2] for p, _ in pos_yaw])) if pos_yaw else None

    def band(pts: np.ndarray) -> np.ndarray:
        if z_robot is None:
            return pts
        return pts[(pts[:, 2] > z_robot - 0.5) & (pts[:, 2] < z_robot + 2.0)]

    # map: up to map_scans evenly spaced scans, voxel-merged (time stays flat on long
    # recordings); scans: `frames` evenly spaced, each downsampled
    acc, scans = [], []
    for i, obs in lidar:
        placed = world_points(obs)
        if placed is None:
            continue
        pts = _voxel(band(placed), map_voxel)
        acc.append(pts)
        if i in shown:
            scans.append((obs.ts, _cap(pts, scan_points, rng)))
        if len(acc) > 16:
            acc = [_voxel(np.concatenate(acc), map_voxel)]
    map_pts = _voxel(np.concatenate(acc), map_voxel) if acc else np.zeros((0, 3))
    if len(map_pts) > map_points:  # coarser voxels keep the shape better than sampling
        map_pts = _cap(
            _voxel(map_pts, map_voxel * np.sqrt(len(map_pts) / map_points)), map_points, rng
        )

    traj = (
        np.array([[t - t0, *p, yaw] for t, (p, yaw) in zip(t_pose, pos_yaw, strict=False)])
        if len(t_pose)
        else np.zeros((0, 5))
    )
    if len(traj) > 3000:
        traj = traj[np.linspace(0, len(traj) - 1, 3000).astype(int)]
    every = np.concatenate([map_pts, traj[:, 1:4]]) if len(traj) else map_pts
    origin = np.round(every.mean(axis=0) if len(every) else np.zeros(3), 2)
    lo, hi = (every.min(axis=0), every.max(axis=0)) if len(every) else (np.zeros(3), np.zeros(3))

    shots, light = [], []
    for _, shot in camera:
        small = shot.data.resize_to_fit(thumb_width, thumb_width)[0]
        light.append(float(np.asarray(small.as_numpy()).mean()))
        shots.append(
            {
                "t": round(shot.ts - t0, 3),
                "jpeg": base64.b64encode(small.to_jpeg_bytes(quality=70)).decode(),
            }
        )

    return {
        "format": FORMAT,
        "duration_s": round(t1 - t0, 3),
        "origin": origin.tolist(),
        "scale": SCALE,
        "bounds": [np.round(lo, 2).tolist(), np.round(hi, 2).tolist()],
        "streams": {k: {"name": s.name, "count": s.count()} for k, s in streams.items()},
        "trajectory": np.round(traj, 3).tolist(),
        "map": _pack(map_pts, origin),
        "scans": [{"t": round(t - t0, 3), "points": _pack(p, origin)} for t, p in scans],
        "camera": shots,
        "thumb": int(np.argmax(light)) if light else None,  # brightest frame: the card image
    }

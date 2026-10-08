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

"""Console preview of a recording, built by `dimos data upload` while the recording is at hand:
LiDAR map and timed scans with the robot's path (from the poses the store keeps with every
LiDAR observation), timed camera thumbnails, and a WebM timelapse."""

from __future__ import annotations

import base64
from pathlib import Path
from typing import TYPE_CHECKING, Any

import cv2
import numpy as np

from dimos.cloud.constants import (
    PREVIEW_BAND,
    PREVIEW_FORMAT,
    PREVIEW_FRAMES,
    PREVIEW_MAP_POINTS,
    PREVIEW_MAP_SCANS,
    PREVIEW_MAP_VOXEL,
    PREVIEW_SCALE,
    PREVIEW_SCAN_POINTS,
    PREVIEW_THUMB_PX,
    TIMELAPSE_FPS,
    TIMELAPSE_HEIGHT,
    TIMELAPSE_MAX_S,
    WORLD_FRAMES,
)
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

if TYPE_CHECKING:
    from dimos.memory.store.base import Store


def _stream(store: Store, payload: type) -> Any:
    """The first stream carrying this dimos type."""
    for name in store.list_streams():
        try:
            if isinstance(store.streams[name].first().data, payload):
                return store.streams[name]
        except LookupError:  # empty stream
            continue
    return None


def _evenly(n: int, k: int) -> set[int]:
    return set(np.linspace(0, n - 1, min(n, k)).astype(int).tolist()) if n else set()


def _fit(pc: PointCloud2, points: int) -> PointCloud2:
    """Coarser voxels until it fits: keeps the shape better than random sampling."""
    n = len(pc.points_f32())
    return (
        pc if n <= points else pc.voxel_downsample(PREVIEW_MAP_VOXEL * float(np.sqrt(n / points)))
    )


def _pack(pc: PointCloud2, origin: np.ndarray) -> str:
    """The browser's point format: little-endian int16 triplets of PREVIEW_SCALE around origin."""
    q = np.clip(np.round((pc.points_f32() - origin) / PREVIEW_SCALE), -32767, 32767).astype("<i2")
    return base64.b64encode(q.tobytes()).decode()


def build(store: Store) -> dict[str, Any] | None:
    """`dimos-spatial-preview-v2`, or None for a recording without LiDAR."""
    lidar, camera = _stream(store, PointCloud2), _stream(store, Image)
    if lidar is None:
        return None
    shown = _evenly(lidar.count(), PREVIEW_FRAMES)
    merged = shown | _evenly(lidar.count(), PREVIEW_MAP_SCANS)
    poses, scans, world, last = [], [], None, 0.0
    for i, obs in enumerate(lidar):  # payloads are lazy: only merged scans are decoded
        last = obs.ts
        if obs.pose is not None:
            poses.append((obs.ts, obs.pose))
        if i not in merged:
            continue
        pc = obs.data
        if pc.frame_id not in WORLD_FRAMES:  # sensor frame: place it with its own pose
            if obs.pose is None:
                continue
            pc = pc.transform(Transform.from_pose(WORLD_FRAMES[0], obs.pose))
        pc = pc.voxel_downsample(PREVIEW_MAP_VOXEL)
        if i in shown:
            scans.append((obs.ts, pc))
        world = pc if world is None else (world + pc).voxel_downsample(PREVIEW_MAP_VOXEL)
    if world is None:
        return None

    z = float(np.median([p.position.z for _, p in poses])) if poses else 0.0
    lo, hi = z + PREVIEW_BAND[0], z + PREVIEW_BAND[1]
    world = _fit(world.filter_by_height(lo, hi), PREVIEW_MAP_POINTS)
    scans = [(t, _fit(pc.filter_by_height(lo, hi), PREVIEW_SCAN_POINTS)) for t, pc in scans]

    shots = []
    if camera is not None:
        picked = _evenly(camera.count(), PREVIEW_FRAMES)
        shots = [o for i, o in enumerate(camera) if i in picked]
    t0 = min([lidar.first().ts] + [o.ts for o in shots[:1]])
    t1 = max([last] + [o.ts for o in shots[-1:]])
    traj = [[t - t0, p.position.x, p.position.y, p.position.z, p.yaw] for t, p in poses]
    traj = [traj[i] for i in sorted(_evenly(len(traj), 3000))]
    pts = world.points_f32().astype(np.float64)
    origin = np.round(pts.mean(axis=0), 2)
    streams = {"lidar": lidar, **({"camera": camera} if camera is not None else {})}
    return {
        "format": PREVIEW_FORMAT,
        "duration_s": round(t1 - t0, 3),
        "origin": origin.tolist(),
        "scale": PREVIEW_SCALE,
        "bounds": [np.round(pts.min(axis=0), 2).tolist(), np.round(pts.max(axis=0), 2).tolist()],
        "streams": {k: {"name": s.name, "count": s.count()} for k, s in streams.items()},
        "trajectory": np.round(traj, 3).tolist(),
        "map": _pack(world, origin),
        "scans": [{"t": round(t - t0, 3), "points": _pack(pc, origin)} for t, pc in scans],
        "camera": [
            {
                "t": round(o.ts - t0, 3),
                "jpeg": o.data.to_base64(
                    70, max_width=PREVIEW_THUMB_PX, max_height=PREVIEW_THUMB_PX
                ),
            }
            for o in shots
        ],
        "thumb": int(np.argmax([o.data.brightness for o in shots])) if shots else None,
    }


def timelapse(store: Store, out: Path) -> dict[str, Any] | None:
    """VP8 WebM of the first camera stream: real time up to TIMELAPSE_MAX_S, sped up to fit
    beyond. Returns {duration_s, speed, bytes}, or None without a camera."""
    camera = _stream(store, Image)
    if camera is None:
        return None
    ts = np.array([o.ts for o in camera])
    span = float(ts[-1] - ts[0])
    speed = max(1.0, span / TIMELAPSE_MAX_S)
    n = max(1, int(min(span, TIMELAPSE_MAX_S) * TIMELAPSE_FPS))
    # video frames each recorded image covers (repeats when the camera is slower than the video)
    at = np.searchsorted(ts, ts[0] + np.arange(n) * speed / TIMELAPSE_FPS, "right") - 1
    repeats = np.bincount(at, minlength=len(ts))
    writer = None
    for i, obs in enumerate(camera):
        if not repeats[i]:
            continue
        img = obs.data
        size = (round(img.width * TIMELAPSE_HEIGHT / img.height / 2) * 2, TIMELAPSE_HEIGHT)
        if writer is None:
            writer = cv2.VideoWriter(str(out), cv2.VideoWriter.fourcc(*"VP80"), TIMELAPSE_FPS, size)
        frame = np.ascontiguousarray(img.resize(*size).to_bgr().as_numpy()[:, :, :3])
        for _ in range(repeats[i]):
            writer.write(frame)
    if writer is not None:
        writer.release()
    return {
        "duration_s": round(n / TIMELAPSE_FPS, 3),
        "speed": round(speed, 3),
        "bytes": out.stat().st_size,
    }

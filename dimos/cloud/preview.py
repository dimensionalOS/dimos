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
LiDAR observation), timed camera thumbnails, and an H.264 timelapse."""

from __future__ import annotations

import base64
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np

try:  # PyAV comes with dimos[unitree] / dimos[webrtc] (aiortc), not the bare base install,
    import av  # where the preview simply has no timelapse

    HAS_AV = True
except ImportError:  # pragma: no cover
    HAS_AV = False

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
    TIMELAPSE_CRF,
    TIMELAPSE_FPS,
    TIMELAPSE_HEIGHT,
    TIMELAPSE_MAX_S,
    WORLD_FRAMES,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

if TYPE_CHECKING:
    from dimos.memory.store.base import Store


def _stream(store: Store, payload: type, prefer: str) -> Any:
    """The first stream carrying this dimos type, named like *prefer* first: a mapper's
    `global_map` is a PointCloud2 too, and `goal_request` a Pose."""
    for name in sorted(store.list_streams(), key=lambda n: prefer not in n):
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
    """`dimos-spatial-preview-v2`, or None for a recording without LiDAR, camera or poses."""
    lidar, camera = _stream(store, PointCloud2, "lidar"), _stream(store, Image, "image")
    odom = _stream(store, Pose, "odom")
    poses, scans, world, ends = [], [], None, []
    if lidar is not None:
        shown = _evenly(lidar.count(), PREVIEW_FRAMES)
        merged = shown | _evenly(lidar.count(), PREVIEW_MAP_SCANS)
        for i, obs in enumerate(lidar):  # payloads are lazy: only merged scans are decoded
            ends.append(obs.ts)
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
        ends = ends[:1] + ends[-1:]
    if not poses and odom is not None:  # no LiDAR (or unposed LiDAR): the path from odometry
        poses = [(o.ts, o.data) for o in odom]
    shots = []
    if camera is not None:
        picked = _evenly(camera.count(), PREVIEW_FRAMES)
        shots = [o for i, o in enumerate(camera) if i in picked]
    ends += [t for t, _ in poses[:1] + poses[-1:]] + [o.ts for o in shots[:1] + shots[-1:]]
    if not ends:
        return None

    z = float(np.median([p.position.z for _, p in poses])) if poses else 0.0
    lo, hi = z + PREVIEW_BAND[0], z + PREVIEW_BAND[1]
    if world is not None:
        world = _fit(world.filter_by_height(lo, hi), PREVIEW_MAP_POINTS)
    scans = [(t, _fit(pc.filter_by_height(lo, hi), PREVIEW_SCAN_POINTS)) for t, pc in scans]

    t0, t1 = min(ends), max(ends)
    traj = [[t - t0, p.position.x, p.position.y, p.position.z, p.yaw] for t, p in poses]
    traj = [traj[i] for i in sorted(_evenly(len(traj), 3000))]
    pts = world.points_f32().astype(np.float64) if world is not None else np.zeros((0, 3))
    if not len(pts):  # no map: frame the path
        pts = np.array([r[1:4] for r in traj] or [[0.0, 0.0, z]])
    origin = np.round(pts.mean(axis=0), 2)
    streams = {"lidar": lidar, "camera": camera, "odom": odom if lidar is None else None}
    return {
        "format": PREVIEW_FORMAT,
        "duration_s": round(t1 - t0, 3),
        "origin": origin.tolist(),
        "scale": PREVIEW_SCALE,
        "bounds": [np.round(pts.min(axis=0), 2).tolist(), np.round(pts.max(axis=0), 2).tolist()],
        "streams": {
            k: {"name": s.name, "count": s.count()} for k, s in streams.items() if s is not None
        },
        "trajectory": np.round(traj, 3).tolist(),
        "map": _pack(world, origin) if world is not None else "",
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
    """H.264 MP4 of the first camera stream: real time up to TIMELAPSE_MAX_S, sped up to fit
    beyond. Returns {duration_s, speed, bytes, type}, or None without a camera or PyAV."""
    camera = _stream(store, Image, "image") if HAS_AV else None
    if camera is None:
        return None
    ts = np.array([o.ts for o in camera])
    span = float(ts[-1] - ts[0])
    speed = max(1.0, span / TIMELAPSE_MAX_S)
    n = max(1, int(min(span, TIMELAPSE_MAX_S) * TIMELAPSE_FPS))
    # video frames each recorded image covers (repeats when the camera is slower than the video)
    at = np.searchsorted(ts, ts[0] + np.arange(n) * speed / TIMELAPSE_FPS, "right") - 1
    repeats = np.bincount(at, minlength=len(ts))
    container = av.open(str(out), "w", options={"movflags": "faststart"})  # plays while downloading
    video: Any = None
    for i, obs in enumerate(camera):
        if not repeats[i]:
            continue
        img = obs.data
        if video is None:
            video = container.add_stream("libx264", rate=TIMELAPSE_FPS)
            video.width = round(img.width * TIMELAPSE_HEIGHT / img.height / 2) * 2
            video.height, video.pix_fmt = TIMELAPSE_HEIGHT, "yuv420p"
            video.options = {"crf": str(TIMELAPSE_CRF), "preset": "veryfast"}
        frame = av.VideoFrame.from_ndarray(
            np.ascontiguousarray(
                img.resize(video.width, video.height).to_bgr().as_numpy()[:, :, :3]
            ),
            format="bgr24",
        )
        for _ in range(repeats[i]):
            container.mux(video.encode(frame))
    container.mux(video.encode())
    container.close()
    return {
        "duration_s": round(n / TIMELAPSE_FPS, 3),
        "speed": round(speed, 3),
        "bytes": out.stat().st_size,
        "type": "video/mp4",
    }

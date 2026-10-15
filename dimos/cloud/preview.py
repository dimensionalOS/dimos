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
LiDAR observation), timed camera thumbnails, joystick input, and an H.264 timelapse."""

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

import cv2
from dimos_generated.geometry_msgs.msg import (
    PoseStamped,
    Transform,
    TransformStamped,
    Vector3,
)
from dimos_generated.sensor_msgs.msg import Image, Joy, PointCloud2
from dimos_generated.std_msgs.msg import Header

from dimos.cloud.constants import (
    PREVIEW_BAND,
    PREVIEW_FORMAT,
    PREVIEW_FRAMES,
    PREVIEW_JOY_SAMPLES,
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
from dimos.msgs.geometry import yaw
from dimos.msgs.image import image_brightness, image_resize_to_fit, image_to_bgr, image_to_jpeg
from dimos.msgs.pointcloud import (
    concatenate_clouds,
    pointcloud_xyz,
    select_points,
    transform_cloud,
    voxel_downsample_cloud,
)
from dimos.msgs.time import time_from_seconds

if TYPE_CHECKING:
    from dimos.memory.store.base import Store


def _stream(store: Store, payload: type, prefer: str) -> Any:
    """The first stream carrying this dimos type, named like *prefer* first: a mapper's
    `global_map` is a PointCloud2 too, `goal_request` a Pose and `depth_image` an Image."""
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
    n = len(pointcloud_xyz(pc))
    return (
        pc
        if n <= points
        else voxel_downsample_cloud(pc, PREVIEW_MAP_VOXEL * float(np.sqrt(n / points)))
    )


def _pack(pc: PointCloud2, origin: np.ndarray) -> str:
    """The browser's point format: little-endian int16 triplets of PREVIEW_SCALE around origin."""
    q = np.clip(np.round((pointcloud_xyz(pc) - origin) / PREVIEW_SCALE), -32767, 32767).astype(
        "<i2"
    )
    return base64.b64encode(q.tobytes()).decode()


def build(store: Store) -> dict[str, Any] | None:
    """`dimos-spatial-preview-v2`, or None for a recording without LiDAR, camera or poses."""
    lidar, camera = _stream(store, PointCloud2, "lidar"), _stream(store, Image, "color")
    odom = _stream(store, PoseStamped, "odom")
    joy = _stream(store, Joy, "joy")
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
            if pc.header.frame_id not in WORLD_FRAMES:  # sensor frame: place it with its own pose
                if obs.pose is None:
                    continue
                pc = transform_cloud(
                    pc,
                    TransformStamped(
                        header=Header(stamp=time_from_seconds(obs.ts), frame_id=WORLD_FRAMES[0]),
                        child_frame_id=pc.header.frame_id,
                        transform=Transform(
                            translation=Vector3(
                                x=obs.pose.position.x, y=obs.pose.position.y, z=obs.pose.position.z
                            ),
                            rotation=obs.pose.orientation,
                        ),
                    ),
                )
            pc = voxel_downsample_cloud(pc, PREVIEW_MAP_VOXEL)
            if i in shown:
                scans.append((obs.ts, pc))
            world = (
                pc
                if world is None
                else voxel_downsample_cloud(concatenate_clouds(world, pc), PREVIEW_MAP_VOXEL)
            )
        ends = ends[:1] + ends[-1:]
    if odom is not None and odom.count() > len(
        poses
    ):  # denser than the LiDAR poses, or the only path
        poses = [(o.ts, o.data.pose) for o in odom]
    sticks = []
    if joy is not None:
        picked = _evenly(joy.count(), PREVIEW_JOY_SAMPLES)
        sticks = [o for i, o in enumerate(joy) if i in picked]
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
        world = _fit(
            select_points(
                world, (pointcloud_xyz(world)[:, 2] > lo) & (pointcloud_xyz(world)[:, 2] < hi)
            ),
            PREVIEW_MAP_POINTS,
        )
    scans = [
        (
            t,
            _fit(
                select_points(
                    pc, (pointcloud_xyz(pc)[:, 2] > lo) & (pointcloud_xyz(pc)[:, 2] < hi)
                ),
                PREVIEW_SCAN_POINTS,
            ),
        )
        for t, pc in scans
    ]

    t0, t1 = min(ends), max(ends)
    traj = [
        [t - t0, p.position.x, p.position.y, p.position.z, yaw(p.orientation)] for t, p in poses
    ]
    traj = [traj[i] for i in sorted(_evenly(len(traj), 3000))]
    pts = pointcloud_xyz(world).astype(np.float64) if world is not None else np.zeros((0, 3))
    if not len(pts):  # no map: frame the path
        pts = np.array([r[1:4] for r in traj] or [[0.0, 0.0, z]])
    origin = np.round(pts.mean(axis=0), 2)
    streams = {
        "lidar": lidar,
        "camera": camera,
        "odom": odom if lidar is None else None,
        "joystick": joy,
    }
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
                "jpeg": base64.b64encode(
                    image_to_jpeg(
                        image_resize_to_fit(o.data, PREVIEW_THUMB_PX, PREVIEW_THUMB_PX)[0], 70
                    )
                ).decode(),
            }
            for o in shots
        ],
        "thumb": int(np.argmax([image_brightness(o.data) for o in shots])) if shots else None,
        "joy": [
            [round(o.ts - t0, 3), [round(a, 2) for a in o.data.axes], list(o.data.buttons)]
            for o in sticks
        ],
    }


def timelapse(store: Store, out: Path) -> dict[str, Any] | None:
    """H.264 MP4 of the first camera stream: real time up to TIMELAPSE_MAX_S, sped up to fit
    beyond. Returns {duration_s, speed, bytes, type}, or None without a camera or PyAV."""
    camera = _stream(store, Image, "color") if HAS_AV else None
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
                cv2.resize(image_to_bgr(img), (video.width, video.height)), dtype=np.uint8
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

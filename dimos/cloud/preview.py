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
from contextlib import closing
import json
import math
from pathlib import Path
import sqlite3
from typing import TYPE_CHECKING, Any, cast

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
    PREVIEW_GAP_THRESHOLD_S,
    PREVIEW_JOY_SAMPLES,
    PREVIEW_MAP_POINTS,
    PREVIEW_MAP_SCANS,
    PREVIEW_MAP_VOXEL,
    PREVIEW_MAX_BYTES,
    PREVIEW_SCALE,
    PREVIEW_SCAN_POINTS,
    PREVIEW_THUMB_PX,
    PREVIEW_TIMING_GAPS,
    TIMELAPSE_CRF,
    TIMELAPSE_FPS,
    TIMELAPSE_HEIGHT,
    TIMELAPSE_MAX_S,
    WORLD_FRAMES,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.memory.store.base import Store
    from dimos.memory.store.sqlite import SqliteStore


logger = setup_logger()


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
    n = len(pc.points_f32())
    return (
        pc if n <= points else pc.voxel_downsample(PREVIEW_MAP_VOXEL * float(np.sqrt(n / points)))
    )


def _pack(pc: PointCloud2, origin: np.ndarray) -> str:
    """The browser's point format: little-endian int16 triplets of PREVIEW_SCALE around origin."""
    q = np.clip(np.round((pc.points_f32() - origin) / PREVIEW_SCALE), -32767, 32767).astype("<i2")
    return base64.b64encode(q.tobytes()).decode()


def _timing(path: Path, streams: dict[str, Any], t0: float) -> dict[str, Any]:
    """Stored SQLite timestamp intervals, not acquisition timing or dropped frames.

    SELECT only ts: stream iteration may decode even without .data access when
    eager_blobs is configured. Use a read-only connection to the staged recording,
    explicit timestamp ordering, and the existing preview origin. Names/counts
    live in doc.streams. This first slice does not support other store formats.
    """
    report = {}
    with closing(sqlite3.connect(f"{path.resolve().as_uri()}?mode=ro", uri=True)) as db:
        for kind, stream in streams.items():
            if stream is None or kind == "joystick":
                continue
            first = last = None
            max_gap = 0.0
            gaps: list[list[float]] = []
            gap_count = 0
            name = stream.name.replace('"', '""')
            for (ts,) in db.execute(f'SELECT ts FROM "{name}" ORDER BY ts'):
                if ts is None or not math.isfinite(ts - t0):
                    raise ValueError("Timing timestamps must be finite")
                t = float(ts - t0)
                if first is None:
                    first = t
                if last is not None:
                    gap = t - last
                    max_gap = max(max_gap, gap)
                    if gap > PREVIEW_GAP_THRESHOLD_S:
                        gap_count += 1
                        if len(gaps) < PREVIEW_TIMING_GAPS:
                            gaps.append([last, t])
                last = t
            if first is not None and last is not None:
                if not math.isfinite(last - first):
                    raise ValueError("Timing span must be finite")
                report[kind] = {
                    "first_s": first,
                    "last_s": last,
                    "span_s": last - first,
                    "max_gap_s": max_gap,
                    "gaps": gaps,
                    "gap_count": gap_count,
                    "truncated": gap_count > len(gaps),
                }
    return {"version": 1, "gap_threshold_s": PREVIEW_GAP_THRESHOLD_S, "streams": report}


def trim_timing(doc: dict[str, Any]) -> None:
    """Drop only optional timing if the full wire JSON exceeds the endpoint budget.

    Call again after adding video metadata. Existing previews keep their old behavior
    if they are already too large without timing. Encoding matches HttpCloudRequest.
    """
    if "timing" not in doc:
        return
    try:
        if len(json.dumps(doc, allow_nan=False).encode()) <= PREVIEW_MAX_BYTES:
            return
    except (TypeError, ValueError):
        pass  # invalid optional timing must not break an otherwise usable preview
    doc.pop("timing")
    if "lidar" in doc.get("streams", {}):
        doc["streams"].pop("odom", None)  # added only for timing when LiDAR is present
    logger.warning("Preview timing omitted: invalid JSON or preview exceeds size limit")


def build(store: Store) -> dict[str, Any] | None:
    """`dimos-spatial-preview-v2`, or None for a recording without LiDAR, camera or poses."""
    lidar, camera = _stream(store, PointCloud2, "lidar"), _stream(store, Image, "color")
    odom = _stream(store, Pose, "odom")
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
            if pc.frame_id not in WORLD_FRAMES:  # sensor frame: place it with its own pose
                if obs.pose is None:
                    continue
                pc = pc.transform(Transform.from_pose(WORLD_FRAMES[0], obs.pose))
            pc = pc.voxel_downsample(PREVIEW_MAP_VOXEL)
            if i in shown:
                scans.append((obs.ts, pc))
            world = pc if world is None else (world + pc).voxel_downsample(PREVIEW_MAP_VOXEL)
        ends = ends[:1] + ends[-1:]
    if odom is not None and odom.count() > len(
        poses
    ):  # denser than the LiDAR poses, or the only path
        poses = [(o.ts, o.data) for o in odom]
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
        world = _fit(world.filter_by_height(lo, hi), PREVIEW_MAP_POINTS)
    scans = [(t, _fit(pc.filter_by_height(lo, hi), PREVIEW_SCAN_POINTS)) for t, pc in scans]

    t0, t1 = min(ends), max(ends)
    traj = [[t - t0, p.position.x, p.position.y, p.position.z, p.yaw] for t, p in poses]
    traj = [traj[i] for i in sorted(_evenly(len(traj), 3000))]
    pts = world.points_f32().astype(np.float64) if world is not None else np.zeros((0, 3))
    if not len(pts):  # no map: frame the path
        pts = np.array([r[1:4] for r in traj] or [[0.0, 0.0, z]])
    origin = np.round(pts.mean(axis=0), 2)
    streams = {
        "lidar": lidar,
        "camera": camera,
        "odom": odom,
        "joystick": joy,
    }
    doc = {
        "format": PREVIEW_FORMAT,
        "duration_s": round(t1 - t0, 3),
        "origin": origin.tolist(),
        "scale": PREVIEW_SCALE,
        "bounds": [np.round(pts.min(axis=0), 2).tolist(), np.round(pts.max(axis=0), 2).tolist()],
        "streams": {
            k: {"name": s.name, "count": s.count()}
            for k, s in streams.items()
            if s is not None and (k != "odom" or lidar is None)
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
        "joy": [
            [round(o.ts - t0, 3), [round(a, 2) for a in o.data.axes], list(o.data.buttons)]
            for o in sticks
        ],
    }

    try:  # timing is optional; a metadata failure must not discard the spatial preview
        timing = _timing(Path(cast("SqliteStore", store).config.path), streams, t0)
        # Old previews omit the odom count when LiDAR is present; timing needs it.
        odom_meta = (
            {"name": odom.name, "count": odom.count()}
            if lidar is not None and odom is not None
            else None
        )
        doc["timing"] = timing
        if odom_meta is not None:
            doc["streams"]["odom"] = odom_meta
        trim_timing(doc)
    except Exception as exc:  # optional report must not break a preview
        logger.warning("Preview timing unavailable", error=str(exc))
    return doc


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

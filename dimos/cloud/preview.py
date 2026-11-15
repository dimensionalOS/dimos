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
LiDAR observation, else the recording's tf), timed camera thumbnails, joystick input, and an
H.264 timelapse."""

from __future__ import annotations

import base64
import itertools
from pathlib import Path
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
from dimos.memory.tf import StreamTF
from dimos.msgs.foxglove_msgs.CompressedVideo import VIDEO_DECODERS, CompressedVideo
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.go2.dds.msgs.CompressedVideo import CompressedVideo as Go2CompressedVideo

if TYPE_CHECKING:
    from collections.abc import Iterator

    from dimos.memory.store.base import Store
    from dimos.memory.type.observation import Observation


def _stream(store: Store, payload: type | tuple[type, ...], prefer: str) -> Any:
    """The first stream carrying this dimos type, named like *prefer* first: a mapper's
    `global_map` is a PointCloud2 too, `goal_request` a Pose and `depth_image` an Image."""
    for name in sorted(store.list_streams(), key=lambda n: prefer not in n):
        kind = getattr(store.streams[name], "data_type", None)
        if isinstance(kind, type) and not issubclass(kind, payload):
            continue  # typed by the store: no need to decode it (or fail to) to know
        try:
            if isinstance(store.streams[name].first().data, payload):
                return store.streams[name]
        except LookupError:  # empty stream
            continue
    return None


def _placed(
    pc: PointCloud2, pose: Pose | None, ts: float, tf: StreamTF | None, frames: tuple[str, ...]
) -> PointCloud2 | None:
    """A scan in the first of *frames* tf reaches. A sensor-frame scan the store posed (a .db
    keeps one) goes by that pose; a world-frame one tf can't relate stays as it is."""
    if pose is not None and pc.frame_id not in WORLD_FRAMES:
        return pc.transform(Transform.from_pose(WORLD_FRAMES[0], pose))
    if tf is not None:
        for frame in frames:
            if (t := tf.get(frame, pc.frame_id, ts, warn=False)) is not None:
                return pc.transform(t)
    return pc if pc.frame_id in WORLD_FRAMES else None


def _decoded(packets: Iterator[Observation[Any]]) -> Iterator[Observation[Image]]:
    """An ordered stream of encoded video packets as BGR ``Image`` observations.

    The decoder is picked from each packet's own ``format`` (``VIDEO_DECODERS``: h264, h265,
    AV1 through dav1d, vp8, vp9) and reopened when it changes. Each packet goes in with its
    timestamp as PTS, so a picture comes out with its own packet's time even when the codec
    reorders frames. Packets before the first keyframe yield nothing, as do unknown formats.
    """
    context: Any = None
    codec, sent = "", cast("dict[int, Observation[Any]]", {})
    ended: list[None] = [None]  # drains what the decoder still holds
    for obs in itertools.chain(packets, ended):
        packet = None if obs is None else obs.data
        if context is not None and (packet is None or packet.format != codec):
            yield from _pictures(context, None, sent)
            context = None
        if obs is None or packet is None:
            continue
        if context is None:
            name = VIDEO_DECODERS.get(packet.format.lower())
            if name is None:
                continue
            codec, context = packet.format, av.CodecContext.create(name, "r")
            if name == "libdav1d":  # else dav1d holds each frame back several packets
                context.options = {"max_frame_delay": "1"}
        encoded = av.Packet(packet.data.tobytes())
        encoded.pts = round(obs.ts * 1e9)
        sent[encoded.pts] = obs
        yield from _pictures(context, encoded, sent)


def _pictures(
    context: Any, encoded: Any, sent: dict[int, Observation[Any]]
) -> Iterator[Observation[Image]]:
    try:
        frames = context.decode(encoded)
    except av.error.FFmpegError:  # a P-frame with no keyframe before it yet
        return
    for frame in frames:
        obs = sent.get(frame.pts)
        if obs is None:
            continue
        for pts in [p for p in sent if p <= frame.pts]:  # what can't come out any more
            del sent[pts]
        bgr = frame.to_ndarray(format="bgr24")
        yield obs.derive(data=Image.from_numpy(bgr, ImageFormat.BGR, obs.data.frame_id, obs.ts))


def _camera(store: Store) -> tuple[Any, Any]:
    """The camera as (the stream to report, its pictures as ``Image`` observations).

    A camera that recorded single frames (jpeg/png, or a ROS 2 ``CompressedImage``) is already
    a stream of ``Image``. One that recorded video (a ``CompressedVideo``: h264 from the M20 and
    the Go2 front camera, or h265 / AV1) is a stream of encoded packets instead: each P-frame
    refers to earlier ones, so the packets only become pictures by running the ordered stream
    through a single decoder. That needs PyAV; without it such a recording has no camera.

    The two halves are kept apart because counts and timestamps must come from the packets:
    asking a decoded stream how long it is would decode the whole recording to find out.
    """
    direct = _stream(store, Image, "color")
    if direct is not None:
        return direct, direct
    packets = _stream(store, (CompressedVideo, Go2CompressedVideo), "front")
    if packets is None or not HAS_AV:
        return packets, None
    return packets, packets.transform(_decoded)


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
    lidar = _stream(store, PointCloud2, "lidar")
    camera, pictures = _camera(store)
    odom = _stream(store, (Pose, Odometry), "odom")
    tf = StreamTF.from_store(store)
    # scans go in the odometry's frame, so the path lines up with the map
    frame = getattr(odom.first().data, "frame_id", "") if odom is not None else ""
    frames = (frame,) if frame else WORLD_FRAMES
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
            if pc.frame_id not in frames:  # a sensor frame, or another world frame
                pc = _placed(pc, obs.pose, obs.ts, tf, frames)
                if pc is None:
                    continue
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
    if pictures is not None:
        picked = _evenly(camera.count(), PREVIEW_FRAMES)
        shots = [o for i, o in enumerate(pictures) if i in picked]
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


def timelapse(store: Store, out: Path) -> dict[str, Any] | None:
    """H.264 MP4 of the first camera stream: real time up to TIMELAPSE_MAX_S, sped up to fit
    beyond. Returns {duration_s, speed, bytes, type}, or None without a camera or PyAV."""
    camera, pictures = _camera(store)
    if pictures is None or not HAS_AV:
        return None
    # timed from the packets, pictures from the decode: see _camera
    ts = np.array([o.ts for o in camera])
    span = float(ts[-1] - ts[0])
    speed = max(1.0, span / TIMELAPSE_MAX_S)
    n = max(1, int(min(span, TIMELAPSE_MAX_S) * TIMELAPSE_FPS))
    # video frames each recorded image covers (repeats when the camera is slower than the video)
    at = np.searchsorted(ts, ts[0] + np.arange(n) * speed / TIMELAPSE_FPS, "right") - 1
    repeats = np.bincount(at, minlength=len(ts))
    container = av.open(str(out), "w", options={"movflags": "faststart"})  # plays while downloading
    video: Any = None
    for obs in pictures:
        # by timestamp, not by position: a decoder emits one picture per packet at best, and
        # drops the P-frames before the first keyframe, so the two are not the same count
        i = min(int(np.searchsorted(ts, obs.ts, "left")), len(ts) - 1)
        wanted = int(repeats[i])
        if not wanted:
            continue
        repeats[i] = 0  # reordered frames can share a timestamp; the slot is filled once
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
        for _ in range(wanted):
            container.mux(video.encode(frame))
    if video is None:  # a camera stream that yielded no decodable picture
        container.close()
        out.unlink(missing_ok=True)
        return None
    container.mux(video.encode())
    container.close()
    return {
        "duration_s": round(n / TIMELAPSE_FPS, 3),
        "speed": round(speed, 3),
        "bytes": out.stat().st_size,
        "type": "video/mp4",
    }

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

"""Open a recording for the memory world: a mem2 ``.db`` or a ROS 2 ``.mcap``.

A ``.db`` is a :class:`SqliteStore` and everything (the SigLIP index, the
replay keyframes and diffs) is written back into it. An ``.mcap`` is not opened for
writing HERE -- the format itself is editable and appendable, and dimos has a command
for it; it is `McapStore` that reads only --
so those derived streams go into ``<name>.derived.db`` beside it, and
:class:`RecordingWithDerivedStreams` presents both as one store: reads look in
the mcap first, new streams are created in the companion database.

The mcap channels are decoded by ROS 2 schema name (CDR): the point cloud,
odometry and IMU decoders come from the Go2 DDS layer; raw images, camera
info and tf messages are decoded here.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
import contextlib
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sqlite3
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.memory.store.base import Store, StreamAccessor
from dimos.memory.store.mcap import McapStore
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.stream import Stream
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.unitree.go2.dds import cdr, ros
from dimos.robot.unitree.go2.dds.codec import FnCodec
from dimos.teleop.memory_world.tf_tree import canonical_frame
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.teleop.memory_world.tf_tree import TfTree

logger = setup_logger()

DERIVED_SUFFIX = ".derived.db"

# ROS image encoding -> (dimos format, numpy dtype, channels); the inverse of
# the table ~/Commands/db_to_mcap writes with.
IMAGE_ENCODINGS: dict[str, tuple[ImageFormat, type, int]] = {
    "rgb8": (ImageFormat.RGB, np.uint8, 3),
    "bgr8": (ImageFormat.BGR, np.uint8, 3),
    "rgba8": (ImageFormat.RGBA, np.uint8, 4),
    "bgra8": (ImageFormat.BGRA, np.uint8, 4),
    "mono8": (ImageFormat.GRAY, np.uint8, 1),
    "mono16": (ImageFormat.GRAY16, np.uint16, 1),
    "16UC1": (ImageFormat.DEPTH16, np.uint16, 1),
    "32FC1": (ImageFormat.DEPTH, np.float32, 1),
}


# ---- CDR layouts of the ROS 2 messages the Go2 layer does not cover ----------


@dataclass
class _ImageWire:
    header: ros._Header
    height: int
    width: int
    encoding: str
    is_bigendian: int
    step: int
    data: np.ndarray

    __cdr_align__ = 1  # CDR aligns primitives, not structs
    __cdr_fields__ = [
        ("header", ros._Header),
        ("height", "u32"),
        ("width", "u32"),
        ("encoding", "string"),
        ("is_bigendian", "u8"),
        ("step", "u32"),
        ("data", ("seq", "u8")),
    ]


def decode_image(buf: bytes) -> Image:
    w: _ImageWire = cdr.decode(buf, _ImageWire)[0]
    if w.encoding not in IMAGE_ENCODINGS:
        raise ValueError(f"unsupported image encoding {w.encoding!r}")
    fmt, dtype, channels = IMAGE_ENCODINGS[w.encoding]
    itemsize = np.dtype(dtype).itemsize
    rows = np.frombuffer(w.data, dtype=np.uint8).reshape(w.height, w.step)  # step: padded rows
    pixels = np.ascontiguousarray(rows[:, : w.width * channels * itemsize]).view(dtype)
    if w.is_bigendian and itemsize > 1:
        pixels = pixels.byteswap().view(pixels.dtype.newbyteorder("="))
    shape = (w.height, w.width, channels) if channels > 1 else (w.height, w.width)
    return Image.from_numpy(
        pixels.reshape(shape), format=fmt, frame_id=w.header.frame_id, ts=ros._ts(w.header)
    )


@dataclass
class _CompressedImageWire:
    header: ros._Header
    format: str
    data: np.ndarray

    __cdr_align__ = 1  # CDR aligns primitives, not structs
    __cdr_fields__ = [("header", ros._Header), ("format", "string"), ("data", ("seq", "u8"))]


def decode_compressed_image(buf: bytes) -> Image:
    """A ROS 2 ``sensor_msgs/CompressedImage`` decoded to pixels."""
    w: _CompressedImageWire = cdr.decode(buf, _CompressedImageWire)[0]
    return image_from_encoded(bytes(w.data), w.format, w.header.frame_id, ros._ts(w.header))


def image_from_encoded(data: bytes, fmt: ImageFormat | str, frame_id: str, ts: float) -> Image:
    """jpeg, png or webp bytes decoded to pixels.

    The Pi recorder stores colour and infrared this way; depth stays raw. The
    format string reads like ``"rgb8; jpeg compressed bgr8"`` or ``"webp"``;
    the bytes decide, and cv2 hands back BGR for colour and the stored depth
    for 16-bit png."""
    import cv2

    pixels = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    if pixels is None:
        raise ValueError(f"undecodable compressed image ({fmt!r}, {len(data)} bytes)")
    if pixels.ndim == 3 and pixels.shape[2] == 3:
        fmt = IMAGE_ENCODINGS["bgr8"][0]
    elif pixels.ndim == 3 and pixels.shape[2] == 4:
        pixels = cv2.cvtColor(pixels, cv2.COLOR_BGRA2BGR)
        fmt = IMAGE_ENCODINGS["bgr8"][0]
    elif pixels.dtype == np.uint16:
        fmt = IMAGE_ENCODINGS["16UC1"][0]
    else:
        fmt = IMAGE_ENCODINGS["mono8"][0]
    return Image.from_numpy(pixels, format=fmt, frame_id=frame_id, ts=ts)


class _DecodedImages:
    """A codec whose decoded ``CompressedImage`` comes out as an ``Image``."""

    def __init__(self, inner: Any) -> None:
        self.inner = inner

    def encode(self, value: Any) -> bytes:
        return self.inner.encode(value)

    def decode(self, data: bytes) -> Image:
        message = self.inner.decode(data)
        return image_from_encoded(bytes(message.data), message.format, message.frame_id, message.ts)


class RecordingDb(SqliteStore):
    """A mem2 database read as a recording: a ``CompressedImage`` stream (the
    stitched Pi recordings store webp) reads as ``Image``, like an mcap's."""

    def _assemble_backend(self, name: str, stored: dict[str, Any]) -> Any:
        backend = super()._assemble_backend(name, stored)
        if stored["payload_module"].endswith(".CompressedImage"):
            codec = _DecodedImages(backend.codec)
            backend.codec = backend.metadata_store._codec = codec
            backend.data_type = Image
        return backend


@dataclass
class _RoiWire:
    x_offset: int
    y_offset: int
    height: int
    width: int
    do_rectify: int

    __cdr_align__ = 1  # CDR aligns primitives, not structs
    __cdr_fields__ = [
        ("x_offset", "u32"),
        ("y_offset", "u32"),
        ("height", "u32"),
        ("width", "u32"),
        ("do_rectify", "u8"),
    ]


@dataclass
class _CameraInfoWire:
    header: ros._Header
    height: int
    width: int
    distortion_model: str
    d: np.ndarray
    k: np.ndarray
    r: np.ndarray
    p: np.ndarray
    binning_x: int
    binning_y: int
    roi: _RoiWire

    __cdr_align__ = 1  # CDR aligns primitives, not structs
    __cdr_fields__ = [
        ("header", ros._Header),
        ("height", "u32"),
        ("width", "u32"),
        ("distortion_model", "string"),
        ("d", ("seq", "f64")),
        ("k", ("array", "f64", 9)),
        ("r", ("array", "f64", 9)),
        ("p", ("array", "f64", 12)),
        ("binning_x", "u32"),
        ("binning_y", "u32"),
        ("roi", _RoiWire),
    ]


def decode_camera_info(buf: bytes) -> CameraInfo:
    w: _CameraInfoWire = cdr.decode(buf, _CameraInfoWire)[0]
    info = CameraInfo(
        height=w.height,
        width=w.width,
        distortion_model=w.distortion_model,
        D=w.d.tolist(),
        K=w.k.tolist(),
        R=w.r.tolist(),
        P=w.p.tolist(),
        binning_x=w.binning_x,
        binning_y=w.binning_y,
        frame_id=w.header.frame_id,
        ts=ros._ts(w.header),
    )
    # The roi is decoded off the wire and then has to be carried across by hand, because
    # CameraInfo.__init__ takes no roi and zeroes the five fields. Dropping it here made
    # every camera_info read from an mcap report "no roi", which is exactly the field
    # `sensor_intrinsics` reads to tell a CROP from a resize -- so a cropped rig had its
    # patches placed 0.84 m out laterally, with the sign of x flipped, while the same
    # recording read from a .db was correct because `lcm_decode` copies these over.
    # Geometry must not depend on which container the recording is in.
    info.roi_x_offset = int(w.roi.x_offset)
    info.roi_y_offset = int(w.roi.y_offset)
    info.roi_height = int(w.roi.height)
    info.roi_width = int(w.roi.width)
    info.roi_do_rectify = bool(w.roi.do_rectify)
    return info


@dataclass
class _TransformWire:
    translation: np.ndarray
    rotation: np.ndarray

    __cdr_align__ = 1  # CDR aligns primitives, not structs
    __cdr_fields__ = [("translation", ("array", "f64", 3)), ("rotation", ("array", "f64", 4))]


@dataclass
class _TransformStampedWire:
    header: ros._Header
    child_frame_id: str
    transform: _TransformWire

    __cdr_align__ = 1  # CDR aligns primitives, not structs
    __cdr_fields__ = [
        ("header", ros._Header),
        ("child_frame_id", "string"),
        ("transform", _TransformWire),
    ]


@dataclass
class _TfMessageWire:
    transforms: list[_TransformStampedWire]

    __cdr_align__ = 1  # CDR aligns primitives, not structs
    __cdr_fields__ = [("transforms", ("seq", _TransformStampedWire))]


def decode_tf_message(buf: bytes) -> TFMessage:
    w: _TfMessageWire = cdr.decode(buf, _TfMessageWire)[0]
    return TFMessage(
        *(
            Transform(
                translation=Vector3(*entry.transform.translation.tolist()),
                rotation=Quaternion(*entry.transform.rotation.tolist()),
                frame_id=entry.header.frame_id,
                child_frame_id=entry.child_frame_id,
                ts=ros._ts(entry.header),
            )
            for entry in w.transforms
        )
    )


@dataclass
class _DimensionWire:
    label: str
    size: int
    stride: int

    __cdr_align__ = 1  # CDR aligns primitives, not structs
    __cdr_fields__ = [("label", "string"), ("size", "u32"), ("stride", "u32")]


@dataclass
class _Float32MultiArrayWire:
    dim: list[_DimensionWire]
    data_offset: int
    data: np.ndarray

    __cdr_align__ = 1  # CDR aligns primitives, not structs
    __cdr_fields__ = [
        ("dim", ("seq", _DimensionWire)),
        ("data_offset", "u32"),
        ("data", ("seq", "f32")),
    ]


@dataclass(frozen=True)
class MultiArray:
    """``std_msgs/msg/Float32MultiArray``: the shape siglipify stores embeddings in.

    One row per frame, laid out ``[patch, dim]`` for per-patch vectors and
    ``[dim]`` for a pooled one. A plain ROS message so anything that reads
    ROS data can read the vectors, not only this module.
    """

    sizes: tuple[int, ...]
    data: np.ndarray  # float32, flat, row-major over ``sizes``

    def vectors(self) -> np.ndarray:
        """The row as ``(count, dims)``: one vector per patch, or a single pooled one."""
        dims = self.sizes[-1] if self.sizes else self.data.size
        return self.data.reshape(-1, dims) if dims else self.data.reshape(0, 0)


def decode_multiarray(buf: bytes) -> MultiArray:
    w: _Float32MultiArrayWire = cdr.decode(buf, _Float32MultiArrayWire)[0]
    return MultiArray(
        sizes=tuple(int(d.size) for d in w.dim),
        data=np.asarray(w.data, dtype=np.float32)[int(w.data_offset) :],
    )


# schema name -> codec, for every channel the memory world can use
ROS2_CODECS: dict[str, FnCodec] = {
    "sensor_msgs/msg/PointCloud2": FnCodec(PointCloud2, ros.decode_pointcloud2),
    "sensor_msgs/msg/Image": FnCodec(Image, decode_image),
    "sensor_msgs/msg/CompressedImage": FnCodec(Image, decode_compressed_image),
    "sensor_msgs/msg/CameraInfo": FnCodec(CameraInfo, decode_camera_info),
    "sensor_msgs/msg/Imu": FnCodec(Imu, ros.decode_imu),
    "nav_msgs/msg/Odometry": FnCodec(Odometry, ros.decode_odometry),
    "tf2_msgs/msg/TFMessage": FnCodec(TFMessage, decode_tf_message),
    "std_msgs/msg/Float32MultiArray": FnCodec(MultiArray, decode_multiarray),
}


def stream_name_of(topic: str) -> str:
    """``/realsense/color`` -> ``realsense_color``: the name a mem2 db would use."""
    return topic.strip("/").removeprefix("rt/").replace("/", "_")


def open_ros2_mcap(path: str | Path) -> McapStore:
    """An mcap opened with codecs chosen by ROS 2 schema name."""
    from mcap.reader import make_reader

    with open(path, "rb") as handle:
        summary = make_reader(handle).get_summary()
    codecs: dict[str, Any] = {}
    names: dict[str, str] = {}
    if summary is not None:
        for channel in summary.channels.values():
            schema = summary.schemas.get(channel.schema_id)
            if schema is not None and schema.name in ROS2_CODECS:
                codecs[channel.topic] = ROS2_CODECS[schema.name]
            names[stream_name_of(channel.topic)] = channel.topic
    return McapStore(path=str(path), codecs=codecs, streams=names)


class RecordingWithDerivedStreams(Store):
    """A read-only recording plus a writable database for the streams built from it.

    ``stream(name)`` returns the recording's stream when it has one and
    otherwise a stream of the companion database, so code that reads a
    recording and appends its own streams works unchanged on an mcap.
    """

    def __init__(self, recording: Store, derived: SqliteStore) -> None:
        super().__init__()
        self.recording = recording
        self.derived = derived

    @property
    def streams(self) -> StreamAccessor[Stream[Any]]:
        return StreamAccessor(self)

    def stream(self, name: str, payload_type: type | None = None, **overrides: Any) -> Stream[Any]:
        if name in self.recording.list_streams():
            return self.recording.stream(name)
        return self.derived.stream(name, payload_type, **overrides)

    def list_streams(self) -> list[str]:
        return sorted(set(self.recording.list_streams()) | set(self.derived.list_streams()))

    def delete_stream(self, name: str) -> None:
        if name in self.recording.list_streams():
            raise ValueError(f"{name!r} is part of the recording and cannot be deleted")
        self.derived.delete_stream(name)

    def summary(self) -> str:
        return f"{self.recording.summary()}\n{self.derived.summary()}".strip()

    def stop(self) -> None:
        self.recording.stop()
        self.derived.stop()
        super().stop()


def derived_db_path(mcap_path: str | Path) -> Path:
    path = Path(mcap_path)
    return path.with_name(path.stem + DERIVED_SUFFIX)


def open_recording(path: str | Path) -> Store:
    """Open a ``.db`` directly, or an ``.mcap`` with its companion derived database."""
    text = str(path)
    if text.endswith(".mcap"):
        return RecordingWithDerivedStreams(
            open_ros2_mcap(text), SqliteStore(path=str(derived_db_path(text)))
        )
    return RecordingDb(path=text, must_exist=True)


# ---- naming a recording's streams -------------------------------------------


def usable_streams(store: Any) -> set[str]:
    """The recording's streams that actually hold something this build can read.

    The same test :func:`detect_streams` applies to the candidates it picks from, lifted
    out so a caller checking a name the OPERATOR gave applies it too. A name is not its
    contents: a killed ingest leaves the name behind, and an empty `tf` accepted from the
    raw list builds a non-None, zero-frame tree that defeats every `tree is None` fallback.
    """
    usable = set()
    for name in store.list_streams():
        try:
            if any(True for _ in store.streams[name]):
                usable.add(name)
        except Exception:  # a stream this build cannot open is not usable either
            continue
    return usable


def name_streams(store: Any, config: Any, tf_tree: Callable[[], Any]) -> None:
    """Name the streams (and the world frame) the config left empty or the recording lacks.

    Roles are filled from the recording's message types, the lidar from whichever
    point-cloud stream agrees with tf, the world frame from the tf root; a name given on
    the command line is kept when the recording has it AND it holds something.

    Lives here rather than on the module because everything it decides with --
    `detect_streams`, `usable_streams`, `pick_lidar`, `tf_root` -- is here, and module.py
    is at the repository's file-size limit. *tf_tree* is a callable because the tree can
    only be read after tf itself has been named, which happens inside the loop.
    """
    # A configured colour stream keeps its own camera_info paired to it.
    detected = detect_streams(store, image=config.image_stream_name or None)
    usable = usable_streams(store)  # named is not enough; see usable_streams above
    # tf first: naming the lidar needs the tree.
    for role, setting in (
        ("tf", "tf_stream_name"),
        ("image", "image_stream_name"),
        ("depth", "depth_stream_name"),
        ("camera_info", "camera_info_stream_name"),
        ("lidar", "lidar_stream_name"),
    ):
        configured = getattr(config, setting)
        if configured and configured not in usable:
            # Said HERE because clearing the name makes this the only place it can be
            # said, and absent and empty are different things for an operator to hear.
            why = "empty" if configured in store.list_streams() else "not in the recording"
            logger.warning("%s: %r is %s; ignoring it", setting, configured, why)
            setattr(config, setting, "")
            configured = ""
        if configured or detected[role] is None:
            continue
        chosen = detected[role]
        if role == "lidar" and len(detected["lidar_candidates"]) > 1:
            tree = tf_tree()  # tf is named first, so the tree can be read now
            if tree is not None:
                world = config.world_frame
                if not tree.has_frame(world):
                    world = tf_root(tree) or world
                chosen = pick_lidar(store, detected["lidar_candidates"], tree, world) or chosen
        setattr(config, setting, chosen)
        logger.info("%s: using %r (detected)", setting, chosen)
    tree = tf_tree()
    if tree is not None and not tree.has_frame(config.world_frame):
        root = tf_root(tree)
        if root:
            logger.info("world_frame: using %r (the tf root)", root)
            config.world_frame = root


def depth_info_stream_for(store: Any, depth_stream: str, camera_info: str) -> str:
    """The depth camera's own ``camera_info`` when the recording has a USABLE one.

    Takes the store, not a set of names, because a name is not intrinsics. This is the
    third place in this package to learn that: `detect_streams` skips empty streams
    because a killed ingest leaves the name behind, and `precomputed_stream_name` because
    an empty embeddings stream is adopted as an index that can never be built. Here an
    empty `<depth>_camera_info` outranks a populated `<depth>` one on name alone, and the
    ingest then dies at "stream ... is empty" -- AFTER it has deleted the index it was
    about to replace.
    """
    present = set(store.list_streams())
    # Split on `_image` rather than stripping a trailing one: a ROS depth topic is
    # `camera_depth_image_rect_raw`, which does not END with `_image`, so removesuffix
    # was a no-op there and the real `camera_depth_camera_info` was never tried. The
    # fallback was then the COLOUR info, used as the depth camera's K -- so the patch
    # correction mapped colour to colour and indexed a 1280-wide raster into an 848-wide
    # one: everything past uv 0.66 dropped, the rest sampled half a frame to the right.
    base = depth_stream.split("_image")[0] or depth_stream
    for candidate in (
        f"{depth_stream}_camera_info",
        f"{base}_camera_info",
    ):
        if candidate in present and next(iter(store.streams[candidate].order_by("ts")), None):
            return candidate
    return camera_info


# Streams this module writes itself; never candidates for the recording's own.
# The SigLIP index needs no entry — its payload type matches no sensor role.
DERIVED_STREAMS = frozenset({"voxel_diff", "voxel_keyframe"})
# Words that rank a candidate up or out, for each role.
_STREAM_HINTS: dict[str, tuple[tuple[str, ...], tuple[str, ...]]] = {
    #  role: (preferred words, disqualifying words)
    "image": (("color", "rgb", "camera"), ("depth", "infra", "ir_", "_ir", "mask")),
    # No "color"/"rgb" disqualifier: `pick("depth", ..., depth_like=True)` has already
    # restricted the candidates to depth-named streams, so those words can only ever
    # SUBTRACT a legitimate one -- and they did. The standard RealSense topic set with
    # `align_depth.enable:=true` names its depth stream
    # `camera_aligned_depth_to_color_image_raw`, which contains "color", so a rig with
    # aligned depth was detected as having no depth at all and the ingest refused a
    # recording that plainly has one.
    "depth": (("depth",), ("infra",)),
    "camera_info": (("color", "rgb"), ("depth", "infra")),
    # An icp-stitched recording carries `<lidar>_corrected` beside the raw scans. The raw
    # scans win: the stitch's loop closure moves the clouds out from under Hyperspace's
    # keyframe poses, so its answers land somewhere the map no longer agrees with.
    "lidar": (
        ("lidar", "cloud", "points", "scan"),
        ("costmap", "map", "accumulated", "corrected"),
    ),
    "tf": (("tf",), ("static",)),
    "tf_static": (("static",), ()),
}


STAGED_SUFFIX = "__rebuilt"


def _holds_anything(store: Store, name: str) -> bool:
    """Whether *name* has a single observation. A stream can be present and empty."""
    try:
        return any(True for _ in store.streams[name])
    except Exception:
        return False


# How many samples of a stream are tried before giving up on reading its shape.
CHANNEL_TRIES = 8


def _channels(store: Store, name: str) -> int | None:
    """How many channels *name*'s images have: the count they AGREE on, else None.

    The first sample is not the stream, twice over. A writer killed mid-frame leaves one
    blob that will not decode and the rest intact, so one failure must not condemn the
    stream: judging on that alone put a colourised image ahead of real DEPTH16, throwing
    away nineteen good frames of depth for one bad one.

    And a stream whose shape CHANGES is not depth at all, so reading on until something
    decodes is not enough either: eight mono frames in front of ninety-two RGB ones read
    as one channel, beat a real DEPTH16 stream, and `patch_world_position` raised "too
    many values to unpack" on the frame it was handed. The last sample is asked as well,
    and a disagreement is "cannot tell", which sorts last.
    """
    counts: set[int] = set()
    try:
        stream = store.streams[name]
        for index, obs in enumerate(stream):
            if index >= CHANNEL_TRIES:
                break
            with contextlib.suppress(Exception):
                counts.add(int(obs.data.channels))
        with contextlib.suppress(Exception):
            counts.add(int(stream.last().data.channels))
    except Exception:
        return None
    return counts.pop() if len(counts) == 1 else None


def refuse_if_a_rebuild_is_half_done(store: Store, name: str) -> None:
    """Stop before touching a stream an earlier rebuild died half way through.

    Checked before the stream is even read, because the run that died may have got as far
    as dropping it: reading it would then be a KeyError, which says nothing about the copy
    sitting right there under another name.
    """
    staged = name + STAGED_SUFFIX
    if staged not in store.list_streams():
        return
    held = sum(1 for _ in store.streams[staged])
    there = sum(1 for _ in store.streams[name]) if name in store.list_streams() else 0
    raise SystemExit(
        f"an earlier rebuild of {name!r} died part way and left {staged!r} behind."
        f" {name!r} has {there} samples and {staged!r} has {held}. If {name!r} is the"
        f" short one it is the half-written copy: replace it with {staged!r}. Either way,"
        f" drop {staged!r} before running this again."
    )


def rebuild_stream(store: Store, name: str, rows: list[tuple[float, Any]], payload: Any) -> None:
    """Replace a stream's contents, with no moment where the data exists nowhere.

    A stream is rewritten by deleting it and writing it again, and between those two the
    recording holds nothing. Putting the old contents back only works while the disk still
    takes writes; when it does not -- a full disk, a read-only remount -- the restore fails
    too and the stream is gone for good. So the new contents go in under another name
    FIRST. The worst case is then a recording holding the data under the wrong name, which
    the error names and which is a minute's work to undo.
    """
    refuse_if_a_rebuild_is_half_done(store, name)
    staged = name + STAGED_SUFFIX
    try:
        written = store.stream(staged, payload)
        for ts, message in rows:
            written.append(message, ts=ts)
    except BaseException:  # nothing has been taken away yet
        if staged in store.list_streams():
            store.delete_stream(staged)
        raise
    store.delete_stream(name)
    try:
        written = store.stream(name, payload)
        for ts, message in rows:
            written.append(message, ts=ts)
    except BaseException:
        raise SystemExit(
            f"writing {name!r} failed after it was dropped. The new contents are in the"
            f" recording as {staged!r} and nothing is lost: copy them back to {name!r}."
        ) from None
    store.delete_stream(staged)


def _spelt_like(name: str, spelling: dict[str, str], slashed: bool) -> str:
    """*name* as the moving stream would write it.

    Its own spelling when the moving stream has used that frame, and otherwise its
    CONVENTION -- a frame the moving stream has never mentioned (a camera hanging off a
    mount, say) still has to match the rest of the stream it is being written into, or
    the mixed spellings are exactly the severed chain this exists to prevent.
    """
    canonical = canonical_frame(name)
    if canonical in spelling:
        return spelling[canonical]
    return f"/{canonical}" if slashed else canonical


def _restamped(
    transform: Any,
    ts: float,
    spelling: dict[str, str] | None = None,
    slashed: bool = False,
    attached: set[str] | None = None,
) -> Any:
    """The same joint, said at another moment, and spelled to join what it must join.

    A static edge carries the one stamp it was latched at. Folding has to restate it at
    the moment it is being put, because TfTree reads the transform's own stamp.

    *spelling* maps a canonical frame name to how the moving stream writes it. A reader
    outside this package need not canonicalise -- dimos' own `MultiTBuffer` keys the raw
    pair -- so a folded edge left in `tf_static`'s spelling beside a differently spelled
    moving stream leaves no chain through the tree at all.

    An edge whose frames the moving stream never mentions is left EXACTLY as it was. It
    has nothing to join: respelling it there only breaks what did work, and the vote that
    decided the convention was taken over frames that have nothing to do with it. Measured
    both ways -- a recording whose tf and tf_static both said `/base -> /cam` came out of
    the fold saying `base -> cam`, and the lookup that answered 3.0 before answered None
    after, on the only edge in the recording.

    *attached* is which frames the moving stream's spelling REACHES, which is not the same
    as which frames it mentions: it travels along the folded edges too. `base -> /mount`
    joins `odom -> base`, and `/mount -> /cam` joins that in turn, so respelling the first
    and leaving the second alone split the chain at `mount` -- 6.0 before the fold, None
    after. Without it, a frame the moving stream never says is the end of the line.
    """
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3

    p, q = transform.translation, transform.rotation
    parent, child = str(transform.frame_id), str(transform.child_frame_id)
    known = spelling or {}
    reaches = known.keys() if attached is None else attached
    joins = canonical_frame(parent) in reaches or canonical_frame(child) in reaches
    return Transform(
        translation=Vector3(float(p.x), float(p.y), float(p.z)),
        rotation=Quaternion(float(q.x), float(q.y), float(q.z), float(q.w)),
        frame_id=_spelt_like(parent, known, slashed) if joins else parent,
        child_frame_id=_spelt_like(child, known, slashed) if joins else child,
        ts=ts,
    )


def fold_static_tf(store: Store, tf_stream: str, static_stream: str) -> int:
    """Move a recording's static tf edges into its moving tf, and drop the static stream.

    One tf tree. :func:`build_tf_tree` lays the static edges over the moving stream, but
    Hyperspace reads the moving stream ALONE, so an edge that lives only in the static one
    is invisible to search while the map and the markers can see it -- and a stale copy in
    the moving stream quietly outvotes the static one. Either way the two disagree.

    Every static edge replaces whatever the moving stream said about it, wherever it said
    it, and is then stated once, at the earliest moment any tf message in the recording
    claims -- ``_Edge.at`` holds a single-sample series from there forward for ever.
    Nothing is compared first: an edge is static because it holds for all time, and a
    moving stream that happens to agree in the samples it carries does not say that
    anywhere. This runs once per recording, because afterwards there is no static stream
    left to fold. Returns the number of edges moved.

    What it cannot keep: a static edge was unbounded in BOTH directions, and a stream
    cannot say that. So a path made only of folded edges stops answering more than a
    tolerance before that earliest stamp. Any path through a moving edge is unaffected,
    because the moving edge ends there too, and on the cart recordings tf starts
    milliseconds before the first image.
    """
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage

    refuse_if_a_rebuild_is_half_done(store, static_stream)
    folded: dict[tuple[str, str], Any] = {}
    for obs in store.streams[static_stream]:
        for t in obs.data.transforms:
            # The FIRST sample of an edge, which is the one TfTree.add keeps for a static.
            # Keyed canonically, like the tree: one recording can spell the same edge
            # `/base -> /cam` on tf_static and `base -> cam` on tf, and keying them raw
            # made them two different edges. The stale MOVING copy then survived the
            # "kept" filter below, outvoted the folded static -- it is a later sample of
            # what the tree sees as the same edge -- and `tf_static`, the only record of
            # the right value, was deleted. Measured: `odom -> cam` at t=2 read 4.0
            # before the fold and 11.0 after, with the evidence gone.
            folded.setdefault(
                (canonical_frame(str(t.frame_id)), canonical_frame(str(t.child_frame_id))), t
            )
    if not folded:
        store.delete_stream(static_stream)
        return 0

    # Stale copies of a folded edge come out of every sample, because one left behind is a
    # later sample of the same edge and wins. The folded value goes back in ONCE, at the
    # earliest moment the stream can be asked about: _Edge.at holds a single-sample series
    # from that moment forward for ever, which is what a static edge means, and any more
    # copies would bound it to the last of them -- a mount that stops holding while the
    # single-sample odometry beside it still does.
    #
    # The stamps are the messages' own, not the stamps they were recorded at, and a
    # transform stamped 0 is read at its observation's: TfTree.from_stream does both, and
    # measuring it any other way puts the copy after the first frame the camera took.
    # How the MOVING stream spells each frame, counting only the transforms that SURVIVE
    # the fold. A stale copy of the folded edge is usually the one spelled the other way
    # -- that is why the recording has two spellings at all -- and taking its word for it
    # named the folded edge after something about to be deleted: measured, a recording
    # whose tf carried `/base -> /cam` beside `odom -> base` answered `odom -> cam` 4.0
    # before the fold and None after, the fold having written the static as `/base ->
    # /cam` to chain with an edge that was no longer there.
    #
    # The folded edge is written back in that spelling when the moving stream has an
    # opinion, and in the static's own when it has none, because a reader outside this
    # package need not canonicalise: dimos' own `MultiTBuffer` keys the raw pair, so a
    # tree left holding `odom -> base` and `/base -> /cam` has no chain through it at all.
    # Measured on a both-ways recording: `odom -> cam` read None after the fold where the
    # answer is 4.0. Taking the static's spelling unconditionally would break the
    # all-slashed recording instead, so it is the moving stream that decides.
    spelling: dict[str, str] = {}
    rows = []
    for obs in store.streams[tf_stream]:
        kept = []
        for t in obs.data.transforms:
            if (
                canonical_frame(str(t.frame_id)),
                canonical_frame(str(t.child_frame_id)),
            ) in folded:
                continue  # a stale copy, on its way out: its spelling leaves with it
            for name in (str(t.frame_id), str(t.child_frame_id)):
                spelling.setdefault(canonical_frame(name), name)
            kept.append(t)
        said = [float(t.ts) or float(obs.ts) for t in obs.data.transforms] + [float(obs.ts)]
        rows.append((float(obs.ts), kept, min(said)))
    if not rows:
        # Folding into nothing would write nothing and destroy the statics on the way.
        raise SystemExit(
            f"{tf_stream!r} is empty, so there is nowhere to fold {static_stream!r} into."
            " A recording with no moving tf has no tree to place anything in."
        )
    # The earliest moment the recording's tf data claims, not just the earliest the
    # MOVING stream does: a rig latches its statics before it starts moving, and a picture
    # taken in that gap has an all-static path to the camera that used to resolve.
    first = min(
        [low for _, _, low in rows]
        + [
            float(t.ts) or float(obs.ts)
            for obs in store.streams[static_stream]
            for t in obs.data.transforms
        ]
    )
    # `TfTree.from_stream` reads `transform.ts or obs.ts`, so a transform stamped exactly
    # 0.0 is read as UNSTAMPED and takes its row's stamp instead -- and 0.0 is what
    # `first` is on a recording whose clock starts at zero. The folded statics therefore
    # have to sit in a row that is ITSELF stamped at `first`, or they do not hold at the
    # moment they were just restated to hold from.
    #
    # In their OWN row, not merged into the first existing one with its stamp moved back.
    # Moving that row retimes every MOVING transform in it that is likewise unstamped:
    # measured on a two-row stream, `world -> base` read 0 m at t=2 before the fold and
    # 1 m after, with a pose appearing at t=1 where there had been none. One fix, one
    # new corruption, in the same function.
    slashed = sum(name.startswith("/") for name in spelling.values()) * 2 > len(spelling)
    # How far the moving stream's spelling reaches: its own frames, and then everything a
    # folded edge joins to them, edge by edge until nothing new is reached. A mount hangs
    # off a frame the moving stream says, and a camera hangs off the mount.
    attached = set(spelling)
    growing = True
    while growing:
        growing = False
        for pair in folded:
            if attached.isdisjoint(pair):
                continue
            for frame in pair:
                if frame not in attached:
                    attached.add(frame)
                    growing = True
    # A frame has ONE parent in a tree, and the fold can hand it a second: a recording that
    # keeps `cam` and `/cam` as different frames says its camera is in two places, and this
    # function DELETES `tf_static` when it is done -- so it would settle that by throwing
    # one of the two answers away. Measured: `odom -> /cam` read 6.0 before the fold and
    # None after, with the only record of it gone. Refuse instead. Nothing is lost by
    # stopping, and the operator can see both values while both are still there.
    #
    # Only a clash the FOLD creates counts. A child whose moving edges were dropped as
    # stale copies of the folded edge is not one, and a moving stream that already gives a
    # frame two parents is its own problem, not this function's.
    moving_parent = {
        canonical_frame(str(t.child_frame_id)): canonical_frame(str(t.frame_id))
        for _, kept, _ in rows
        for t in kept
    }
    clashes = sorted(
        f"{child!r} hangs off {moving_parent[child]!r} in {tf_stream!r}"
        f" and off {parent!r} in {static_stream!r}"
        for parent, child in folded
        if child in moving_parent and moving_parent[child] != parent
    )
    if clashes:
        raise SystemExit(
            f"folding {static_stream!r} into {tf_stream!r} would give a frame two parents: "
            + "; ".join(clashes)
            + ". The recording says the same frame is in two places, and folding would"
            " delete the evidence for one of them. Nothing was changed."
        )
    statics_row = [
        (
            first,
            TFMessage(
                *(_restamped(t, first, spelling, slashed, attached) for t in folded.values())
            ),
        )
    ]
    written = statics_row + [(ts, TFMessage(*kept)) for ts, kept, _ in rows]
    rebuild_stream(store, tf_stream, written, TFMessage)
    store.delete_stream(static_stream)
    return len(folded)


def build_tf_tree(store: Store, tf_stream: str) -> TfTree:
    """The recording's tf tree: the moving stream, with its static edges over the top.

    The recording is taken exactly as it is, with no world named and nothing substituted
    in. An icp stitch's loop-closed odometry used to be put in here so everything the
    tree placed landed in the world the stitched map was in; that is gone, because the
    stitch moves the clouds out from under Hyperspace's keyframe poses and its answers
    then disagree with the map.
    """
    from dimos.teleop.memory_world.tf_tree import TfTree

    # A rebuild that died leaves the real stream truncated and the whole copy beside it.
    # Nothing else would notice: a short tf still loads, still answers, and places the
    # second half of the recording nowhere.
    refuse_if_a_rebuild_is_half_done(store, tf_stream)
    tree = TfTree.from_stream(store.streams[tf_stream])
    static = detect_streams(store).get("tf_static")
    if static is not None:
        refuse_if_a_rebuild_is_half_done(store, static)
        for obs in store.streams[static]:
            for t in obs.data.transforms:
                p, q = t.translation, t.rotation
                tree.add(
                    str(t.frame_id),
                    str(t.child_frame_id),
                    float(obs.ts),
                    (float(p.x), float(p.y), float(p.z)),
                    (float(q.x), float(q.y), float(q.z), float(q.w)),
                    static=True,
                )
    return tree


def tf_root(tree: Any) -> str | None:
    """The frame nothing hangs under (``odom`` or ``world``, typically): where the map lives.
    None when the tree is empty or has several roots."""
    # A self-edge names its frame as both a parent and a child, and one is enough to
    # subtract the real root out of the set. A duplicate or misconfigured broadcaster
    # republishing a frame onto itself is a real ROS shape, and it made this return None
    # for a tree that has exactly one root -- so `world_frame` kept its default, the tree
    # did not have that frame, and every lookup afterwards returned None. A recording
    # whose tf is otherwise perfectly usable placed nothing at all.
    edges = [(parent, child) for parent, child in tree._edges if parent != child]
    parents = {parent for parent, _ in edges}
    children = {child for _, child in edges}
    roots = sorted(parents - children)
    return roots[0] if len(roots) == 1 else None


def detect_streams(store: Store, image: str | None = None) -> dict[str, Any]:
    """Name the stream to use for each role, from the payload types in *store*.

    *image* is the colour stream the caller chose, when it did: its camera_info
    is paired to that one (``<image>_camera_info``, else ``<image minus _image>_camera_info``).
    Depth is picked by name, preferring the streams whose images are single-channel, so a
    rig with two depth cameras needs the caller to name its depth stream
    (``depth_stream_name``).

    Recordings disagree about names — this rig calls its camera
    ``realsense_color_image`` where a Go2 recording says ``color_image`` — so
    each role is filled by payload type first and by the name only to break
    ties. Roles with no candidate come back as None.
    """
    present = set(store.list_streams())
    for name in present:
        # A rebuild that died between dropping the old stream and writing the new one
        # leaves only the staged copy. Nothing further down would say so: the role comes
        # back empty and the module goes on without a tf at all, complaining about a
        # stream named "". A staged copy whose original IS there needs no special case --
        # it holds the same payload, but it is the longer name of the two and the ranking
        # below never prefers it.
        if not name.endswith(STAGED_SUFFIX):
            continue
        original = name.removesuffix(STAGED_SUFFIX)
        # EMPTY counts as not there. `rebuild_stream` drops the original and then writes
        # it back, so a rebuild dying inside that window leaves the name present and
        # holding nothing -- and the old test, which asked only whether the name existed,
        # let it through. The empty original is then dropped from the ranking below as
        # having no payload type, the staged copy becomes the only candidate of its type,
        # and the module reads a half-written rebuild as the recording's own tf.
        if original not in present or not _holds_anything(store, original):
            raise SystemExit(
                f"{name!r} is in the recording and {original!r} is"
                " not: a rebuild died between dropping the old stream and writing the new"
                f" one. Rename {name!r} back and nothing is lost."
            )

    by_type: dict[str, list[str]] = {}
    for name in present:
        # A staged copy is never the recording's own stream, whatever it holds. The
        # ranking used to be trusted to pass it over on length alone, which is true only
        # while the original is a candidate too.
        if name in DERIVED_STREAMS or name.endswith(STAGED_SUFFIX):
            continue
        try:
            payload = store.stream(name).data_type
            # An empty stream cannot fill a role, and it can lose one: a `tf` left behind
            # empty by a killed ingest outranks the recording's own `robot_tf` on name
            # alone, and the world then has no transforms at all.
            if payload is None or not any(True for _ in store.streams[name]):
                continue
        except Exception:  # a stream this build cannot open is not a candidate
            continue
        by_type.setdefault(payload.__name__, []).append(name)

    def rank(role: str, type_name: str) -> list[str]:
        """Candidates for a role, best-named first."""
        preferred, disqualifying = _STREAM_HINTS[role]
        candidates = [
            name
            for name in by_type.get(type_name, [])
            if not any(word in name.lower() for word in disqualifying)
        ]

        def order(name: str) -> tuple[int, int, str]:
            # Earlier words in `preferred` win: a recording with both
            # `pointlio_lidar` and `rtab_cloud` should give the lidar, since
            # "cloud" also fits a cloud some other stage derived.
            hit = next(
                (i for i, word in enumerate(preferred) if word in name.lower()), len(preferred)
            )
            return (hit, len(name), name)

        return sorted(candidates, key=order)

    def pick(role: str, type_name: str, depth_like: bool | None = None) -> str | None:
        candidates = rank(role, type_name)
        if depth_like is not None:
            candidates = [name for name in candidates if ("depth" in name.lower()) == depth_like]
        if depth_like:
            # Named like depth is not the same as BEING depth. A colourised depth image --
            # what a `colorizer` node publishes for a person to look at -- is called
            # `depth_color` and is three channels of RGB, and being the shorter name it
            # outranked the real `camera_aligned_depth_to_color_image_raw` beside it;
            # `patch_world_position` then raised "too many values to unpack" on it, since
            # a metre is stored in one channel.
            #
            # It BREAKS THE TIE rather than disqualifying, because a stream says how many
            # channels it has only as clearly as its codec lets it: a db whose images were
            # written with the default jpeg codec hands back three channels of RGB
            # whatever went in, and a recording whose only depth is stored that way should
            # still be found. Real recordings decode depth as it was written -- the
            # grocery recording's `depth_image` reads (720, 1280) uint16 DEPTH16.
            #
            # A candidate whose sample will not decode at all goes LAST, below even the
            # colour one. Treating "cannot tell" as "no objection" let a stream with a
            # truncated blob win on its shorter name, and there is no route by which the
            # module could read a metre out of it -- measured, a corrupt `depth` beat a
            # `camera_depth_image` that reads as real DEPTH16.
            def depth_first(name: str) -> int:
                channels = _channels(store, name)
                return 0 if channels == 1 else (2 if channels is None else 1)

            candidates = sorted(candidates, key=depth_first)
        return candidates[0] if candidates else None

    image = image if image in by_type.get("Image", []) else pick("image", "Image", depth_like=False)
    detected = {
        "image": image,
        "depth": pick("depth", "Image", depth_like=True),
        "camera_info": pick("camera_info", "CameraInfo"),
        "lidar": pick("lidar", "PointCloud2"),
        # Every PointCloud2 stream, best-named first. A recording often holds
        # several lidars and several stages of registration, and the name says
        # nothing about which one agrees with the tf tree — the caller checks.
        "lidar_candidates": rank("lidar", "PointCloud2"),
        "tf": pick("tf", "TFMessage"),
        "tf_static": next(
            (n for n in rank("tf_static", "TFMessage") if "static" in n.lower()),
            None,
        ),
    }
    # Prefer the camera_info that belongs to the chosen image stream.
    if image is not None:
        infos = by_type.get("CameraInfo", [])
        # Split on `_image`, not removesuffix -- the same fix `depth_info_stream_for`
        # carries and for the same reason. `camera_color_image_raw` does not END with
        # `_image`, so removesuffix was a no-op, both candidates were the identical
        # string, neither existed, and the generic hint ranking picked whichever
        # CameraInfo it liked: on a two-camera recording, the OTHER camera's intrinsics.
        # That K reaches _camera_hfov, sensor_intrinsics and patch_world_position, which
        # is to say it reaches where the answers are placed.
        base = image.split("_image")[0] or image
        for paired in (f"{image}_camera_info", f"{base}_camera_info"):
            if paired in infos:
                detected["camera_info"] = paired
                break
    return detected


# ---- embeddings another tool wrote into the recording -----------------------

# The type siglipify registers its streams under. It is not a dimos class, so
# a SqliteStore cannot open such a stream itself; the rows are read directly.
EMBEDDING_PAYLOAD_MODULE = "std_msgs.msg.Float32MultiArray"


def embedding_stream_name(image_stream_name: str, model_name: str) -> str:
    """siglipify's name for a model's vectors of an image stream.

    ``color_image`` embedded with ``google/siglip2-giant-opt-patch16-384`` is
    ``color_image_siglip2_giant_opt_p16_384``: the source stream, then the
    model, so two checkpoints' vectors never share a stream.
    """
    from dimos.teleop.memory_world.visual_search import model_slug

    return f"{image_stream_name}_{model_slug(model_name)}"


@dataclass(frozen=True)
class StoredEmbedding:
    """One frame's vectors as another tool stored them, before any alignment."""

    ts: float
    vectors: np.ndarray  # (count, dims) float32: one row per patch, or one pooled row
    # A mem2 row names the frame it embeds; an mcap message only shares its stamp.
    source_id: int | None = None
    model: str | None = None


class StoredEmbeddings:
    """The rows of an embedding stream, from either container.

    In a mem2 database the stream is read straight from its tables, because
    its payload type is a ROS message dimos has no class for. In an mcap it is
    an ordinary channel decoded by schema name.
    """

    def __init__(self, store: Store, name: str) -> None:
        self.store = store
        self.name = name
        self._db_path = store.config.path if isinstance(store, SqliteStore) else None

    def _connect(self) -> sqlite3.Connection:
        return sqlite3.connect(f"file:{self._db_path}?mode=ro", uri=True)

    def text_aligned(self) -> bool | None:
        """Whether every vector went through the pooling head, so text can score it.

        siglipify marks its streams; one written before the mark existed holds
        raw tower tokens. None when the container carries no such mark (mcap).
        """
        if self._db_path is None:
            return None
        conn = self._connect()
        try:
            row = conn.execute(
                "SELECT json_extract(config, '$.text_aligned') FROM _streams WHERE name = ?",
                (self.name,),
            ).fetchone()
        finally:
            conn.close()
        return None if row is None else bool(row[0])

    def count(self) -> int:
        if self._db_path is None:
            return int(self.store.streams[self.name].count())
        conn = self._connect()
        try:
            return int(conn.execute(f'SELECT count(*) FROM "{self.name}"').fetchone()[0])
        finally:
            conn.close()

    def __iter__(self) -> Iterator[StoredEmbedding]:
        if self._db_path is None:
            for obs in self.store.streams[self.name].order_by("ts"):
                yield StoredEmbedding(ts=float(obs.ts), vectors=obs.data.vectors())
            return
        conn = self._connect()
        try:
            rows = conn.execute(
                f'SELECT s.ts, json(s.tags), b.data FROM "{self.name}" s '
                f'JOIN "{self.name}_blob" b ON b.id = s.id ORDER BY s.ts'
            )
            for ts, tags_json, blob in rows:
                tags = json.loads(tags_json) if tags_json else {}
                source_id = tags.get("source_id")
                yield StoredEmbedding(
                    ts=float(ts),
                    vectors=decode_multiarray(blob).vectors(),
                    source_id=None if source_id is None else int(source_id),
                    model=tags.get("model"),
                )
        finally:
            conn.close()


def grid_side(patch_count: int) -> int:
    """Rows (= columns) of a square patch grid; a pooled row is a 1x1 grid."""
    side = math.isqrt(patch_count)
    if side * side != patch_count:
        raise ValueError(f"{patch_count} patches do not form a square grid")
    return side


# ---- which lidar agrees with the tf tree ------------------------------------

# Scans sampled per candidate when measuring how far its poses sit from tf.
LIDAR_AGREEMENT_SAMPLES = 10


@dataclass(frozen=True)
class LidarAgreement:
    """How one point-cloud stream relates to the tf tree."""

    name: str
    frame_id: str
    # tf can put this stream's frame in the world at its scans' stamps.
    placeable: bool
    # Median distance from the pose stamped on a scan to the nearest tf frame
    # at that stamp, over sampled scans. None when the scans carry no pose.
    disagreement_m: float | None


def lidar_agreement(
    store: Store, name: str, tree: Any, world_frame: str, samples: int = LIDAR_AGREEMENT_SAMPLES
) -> LidarAgreement:
    """Measure *name* against *tree* (a ``TfTree``) on scans spread over the recording."""
    stream = store.streams[name]
    first, last = stream.first(), stream.last()
    stamps = np.linspace(float(first.ts), float(last.ts), samples + 2)[1:-1]
    frame_id = str(getattr(first.data, "frame_id", "") or "").lstrip("/")
    placeable = False
    distances: list[float] = []
    for ts in stamps:
        try:
            obs = stream.at(float(ts), tolerance=1.0).first()
        except LookupError:
            continue
        if tree.lookup(world_frame, frame_id, float(obs.ts)) is not None:
            placeable = True
        pose = obs.pose_tuple
        if pose is None:
            continue
        stamped = np.asarray(pose[:3], dtype=float)
        nearest = min(
            (
                float(np.linalg.norm(matrix[:3, 3] - stamped))
                for frame in tree.frames
                if (matrix := tree.lookup(world_frame, frame, float(obs.ts))) is not None
            ),
            default=None,
        )
        if nearest is not None:
            distances.append(nearest)
    return LidarAgreement(
        name=name,
        frame_id=frame_id,
        placeable=placeable,
        disagreement_m=float(np.median(distances)) if distances else None,
    )


def pick_lidar(store: Store, candidates: list[str], tree: Any, world_frame: str) -> str | None:
    """The point-cloud stream whose poses the tf tree agrees with.

    A recording often holds several lidars and several stages of registration
    — here ten — and the name says nothing about which one the tf tree was
    built from: on this stairwell the stream called plain ``lidar`` sits 12 m
    from where tf puts the robot (the robot's own SLAM, in its own world),
    ``fastlio_lidar`` 1.4 m, ``pointlio_lidar`` 1 cm. Every candidate is
    scored by the distance between the pose stamped on its scans and the
    nearest tf frame at the same stamp; the closest wins. A stream tf cannot
    place at all is out, and one with no stamped poses is a last resort, in
    the caller's order.
    """
    measured: list[LidarAgreement] = []
    for name in candidates:
        try:
            measured.append(lidar_agreement(store, name, tree, world_frame))
        except Exception as error:  # a stream this build cannot read is not a candidate
            logger.warning("lidar candidate %r unreadable: %s", name, error)
    for entry in measured:
        logger.info(
            "lidar candidate %r (frame %r): %s, %s from tf",
            entry.name,
            entry.frame_id,
            "placeable" if entry.placeable else "not placeable",
            "no stamped pose" if entry.disagreement_m is None else f"{entry.disagreement_m:.2f} m",
        )
    usable = [entry for entry in measured if entry.placeable]
    scored = [entry for entry in usable if entry.disagreement_m is not None]
    if scored:
        return min(scored, key=lambda entry: entry.disagreement_m or 0.0).name
    return usable[0].name if usable else None

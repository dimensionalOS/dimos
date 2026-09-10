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
replay keyframes and diffs) is written back into it. An ``.mcap`` is read-only,
so those derived streams go into ``<name>.derived.db`` beside it, and
:class:`RecordingWithDerivedStreams` presents both as one store: reads look in
the mcap first, new streams are created in the companion database.

The mcap channels are decoded by ROS 2 schema name (CDR): the point cloud,
odometry and IMU decoders come from the Go2 DDS layer; raw images, camera
info and tf messages are decoded here.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

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
    pixels = w.data.view(dtype)
    shape = (w.height, w.width, channels) if channels > 1 else (w.height, w.width)
    return Image.from_numpy(
        pixels.reshape(shape), format=fmt, frame_id=w.header.frame_id, ts=ros._ts(w.header)
    )


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
    return CameraInfo(
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


# schema name -> codec, for every channel the memory world can use
ROS2_CODECS: dict[str, FnCodec] = {
    "sensor_msgs/msg/PointCloud2": FnCodec(PointCloud2, ros.decode_pointcloud2),
    "sensor_msgs/msg/Image": FnCodec(Image, decode_image),
    "sensor_msgs/msg/CameraInfo": FnCodec(CameraInfo, decode_camera_info),
    "sensor_msgs/msg/Imu": FnCodec(Imu, ros.decode_imu),
    "nav_msgs/msg/Odometry": FnCodec(Odometry, ros.decode_odometry),
    "tf2_msgs/msg/TFMessage": FnCodec(TFMessage, decode_tf_message),
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
    return SqliteStore(path=text, must_exist=True)


# ---- naming a recording's streams -------------------------------------------

# Streams this module writes itself; never candidates for the recording's own.
DERIVED_STREAMS = frozenset({"voxel_diff", "voxel_keyframe", "image_siglip2_patches"})
# Words that rank a candidate up or out, for each role.
_STREAM_HINTS: dict[str, tuple[tuple[str, ...], tuple[str, ...]]] = {
    #  role: (preferred words, disqualifying words)
    "image": (("color", "rgb", "camera"), ("depth", "infra", "ir_", "_ir", "mask")),
    "depth": (("depth",), ("color", "rgb", "infra")),
    "camera_info": (("color", "rgb"), ("depth", "infra")),
    "lidar": (("lidar", "cloud", "points", "scan"), ("costmap", "map")),
    "tf": (("tf",), ()),
}


def detect_streams(store: Store) -> dict[str, str | None]:
    """Name the stream to use for each role, from the payload types in *store*.

    Recordings disagree about names — this rig calls its camera
    ``realsense_color_image`` where a Go2 recording says ``color_image`` — so
    each role is filled by payload type first and by the name only to break
    ties. Roles with no candidate come back as None.
    """
    by_type: dict[str, list[str]] = {}
    for name in store.list_streams():
        if name in DERIVED_STREAMS:
            continue
        try:
            payload = store.stream(name).data_type
        except Exception:  # a stream this build cannot open is not a candidate
            continue
        if payload is not None:
            by_type.setdefault(payload.__name__, []).append(name)

    def pick(role: str, type_name: str, depth_like: bool | None = None) -> str | None:
        preferred, disqualifying = _STREAM_HINTS[role]
        candidates = [
            name
            for name in by_type.get(type_name, [])
            if not any(word in name.lower() for word in disqualifying)
        ]
        if depth_like is not None:
            candidates = [name for name in candidates if ("depth" in name.lower()) == depth_like]
        if not candidates:
            return None

        def rank(name: str) -> tuple[int, int, str]:
            # Earlier words in `preferred` win: a recording with both
            # `pointlio_lidar` and `rtab_cloud` should give the lidar, since
            # "cloud" also fits a cloud some other stage derived.
            hit = next(
                (i for i, word in enumerate(preferred) if word in name.lower()), len(preferred)
            )
            return (hit, len(name), name)

        return min(candidates, key=rank)

    image = pick("image", "Image", depth_like=False)
    detected = {
        "image": image,
        "depth": pick("depth", "Image", depth_like=True),
        "camera_info": pick("camera_info", "CameraInfo"),
        "lidar": pick("lidar", "PointCloud2"),
        "tf": pick("tf", "TFMessage"),
    }
    # Prefer the camera_info that belongs to the chosen image stream.
    if image is not None:
        paired = f"{image}_camera_info"
        if paired in by_type.get("CameraInfo", []):
            detected["camera_info"] = paired
    return detected

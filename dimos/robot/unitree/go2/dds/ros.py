# Copyright 2026 Dimensional Inc.
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

"""Decode the standard-ROS DDS channels into generated messages.

The wire layouts are declared as ``__cdr_fields__`` specs and walked by the
generic :mod:`cdr` decoder (no hand-rolled byte cursors). A thin per-type
adapter maps the decoded wire struct into the dimos message — only the parts
that genuinely differ from a field copy (point-buffer reinterpretation, jpeg
decode, pose nesting) live there.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from dimos.message_utils import image
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    Quaternion,
    Twist,
    TwistWithCovariance,
    Vector3,
)
from dimos.messages.nav_msgs.msg import Odometry
from dimos.messages.sensor_msgs.msg import Image, Imu, PointCloud2
from dimos.messages.std_msgs.msg import Header
from dimos.robot.unitree.go2.dds import cdr
from dimos.robot.unitree.go2.dds.msgs.CompressedVideo import CompressedVideo
from dimos.robot.unitree.go2.dds.msgs.HeightMap import HeightMap
from dimos.robot.unitree.go2.dds.pointcloud import sensor_cloud


# Shared wire specs (Header/Time) reused by the per-message layouts below.
@dataclass
class _Time:
    sec: int
    nanosec: int

    __cdr_fields__ = [("sec", "i32"), ("nanosec", "u32")]


@dataclass
class _Header:
    stamp: _Time
    frame_id: str

    __cdr_fields__ = [("stamp", _Time), ("frame_id", "string")]


def _ts(h: _Header) -> float:
    return h.stamp.sec + h.stamp.nanosec * 1e-9


# sensor_msgs/Imu
@dataclass
class _ImuWire:
    header: _Header
    orientation: np.ndarray  # f64[4] xyzw
    orientation_covariance: np.ndarray  # f64[9]
    angular_velocity: np.ndarray  # f64[3]
    angular_velocity_covariance: np.ndarray
    linear_acceleration: np.ndarray  # f64[3]
    linear_acceleration_covariance: np.ndarray

    __cdr_fields__ = [
        ("header", _Header),
        ("orientation", ("array", "f64", 4)),
        ("orientation_covariance", ("array", "f64", 9)),
        ("angular_velocity", ("array", "f64", 3)),
        ("angular_velocity_covariance", ("array", "f64", 9)),
        ("linear_acceleration", ("array", "f64", 3)),
        ("linear_acceleration_covariance", ("array", "f64", 9)),
    ]


def decode_imu(buf: bytes) -> Imu:
    w: _ImuWire = cdr.decode(buf, _ImuWire)[0]
    # Unitree fills orientation wxyz even in this sensor_msgs/Imu — reorder to xyzw
    # (verified: rotating accel by this lands gravity on +z in leg-odom-stationary windows).
    qw, qx, qy, qz = (float(v) for v in w.orientation)
    return Imu(
        orientation=Quaternion(qx, qy, qz, qw),
        angular_velocity=Vector3(*w.angular_velocity.tolist()),
        linear_acceleration=Vector3(*w.linear_acceleration.tolist()),
        orientation_covariance=w.orientation_covariance.copy(),
        angular_velocity_covariance=w.angular_velocity_covariance.copy(),
        linear_acceleration_covariance=w.linear_acceleration_covariance.copy(),
        header=Header(Time(w.header.stamp.sec, w.header.stamp.nanosec), w.header.frame_id),
    )


# nav_msgs/Odometry
@dataclass
class _PoseWire:
    position: np.ndarray  # f64[3]
    orientation: np.ndarray  # f64[4] xyzw

    __cdr_fields__ = [("position", ("array", "f64", 3)), ("orientation", ("array", "f64", 4))]


@dataclass
class _PoseWithCov:
    pose: _PoseWire
    covariance: np.ndarray  # f64[36]

    __cdr_fields__ = [("pose", _PoseWire), ("covariance", ("array", "f64", 36))]


@dataclass
class _TwistWire:
    linear: np.ndarray  # f64[3]
    angular: np.ndarray  # f64[3]

    __cdr_fields__ = [("linear", ("array", "f64", 3)), ("angular", ("array", "f64", 3))]


@dataclass
class _TwistWithCov:
    twist: _TwistWire
    covariance: np.ndarray  # f64[36]

    __cdr_fields__ = [("twist", _TwistWire), ("covariance", ("array", "f64", 36))]


@dataclass
class _OdomWire:
    header: _Header
    child_frame_id: str
    pose: _PoseWithCov
    twist: _TwistWithCov

    __cdr_fields__ = [
        ("header", _Header),
        ("child_frame_id", "string"),
        ("pose", _PoseWithCov),
        ("twist", _TwistWithCov),
    ]


def decode_odometry(buf: bytes) -> Odometry:
    w: _OdomWire = cdr.decode(buf, _OdomWire)[0]
    return Odometry(
        Header(Time(w.header.stamp.sec, w.header.stamp.nanosec), w.header.frame_id),
        w.child_frame_id,
        PoseWithCovariance(
            Pose(
                Point(*w.pose.pose.position.tolist()), Quaternion(*w.pose.pose.orientation.tolist())
            ),
            np.zeros(36, dtype=np.float64),
        ),
        TwistWithCovariance(
            Twist(
                Vector3(*w.twist.twist.linear.tolist()), Vector3(*w.twist.twist.angular.tolist())
            ),
            np.zeros(36, dtype=np.float64),
        ),
    )


# sensor_msgs/PointCloud2
@dataclass
class _PointField:
    name: str
    offset: int
    datatype: int
    count: int

    __cdr_fields__ = [
        ("name", "string"),
        ("offset", "u32"),
        ("datatype", "u8"),
        ("count", "u32"),
    ]


@dataclass
class _Pc2Wire:
    header: _Header
    height: int
    width: int
    fields: list[_PointField]
    is_bigendian: int
    point_step: int
    row_step: int
    data: np.ndarray  # u8[]
    is_dense: int

    __cdr_fields__ = [
        ("header", _Header),
        ("height", "u32"),
        ("width", "u32"),
        ("fields", ("seq", _PointField)),
        ("is_bigendian", "u8"),
        ("point_step", "u32"),
        ("row_step", "u32"),
        ("data", ("seq", "u8")),
        ("is_dense", "u8"),
    ]


# ROS PointField datatype code -> numpy dtype
_PF_DT = {1: "<i1", 2: "<u1", 3: "<i2", 4: "<u2", 5: "<i4", 6: "<u4", 7: "<f4", 8: "<f8"}


def decode_pointcloud2(buf: bytes) -> PointCloud2:
    w: _Pc2Wire = cdr.decode(buf, _Pc2Wire)[0]
    header = Header(Time(w.header.stamp.sec, w.header.stamp.nanosec), w.header.frame_id)
    if w.point_step == 0 or w.data.size < w.point_step:
        return sensor_cloud(np.empty((0, 3), np.float32), header=header)
    dt = np.dtype(
        {
            "names": [f.name for f in w.fields],
            "formats": [_PF_DT[f.datatype] for f in w.fields],
            "offsets": [f.offset for f in w.fields],
            "itemsize": w.point_step,
        }
    )
    arr = w.data.view(dt)
    xyz = np.stack([arr["x"], arr["y"], arr["z"]], axis=-1).astype(np.float32)
    names = set(dt.names or ())

    def field(*keys: str) -> np.ndarray | None:
        return next((arr[k] for k in keys if k in names), None)

    inten, time_s, line, tag = (
        field("intensity"),
        field("time"),
        field("ring", "line"),
        field("tag"),
    )
    return sensor_cloud(
        xyz,
        header=header,
        intensities=None if inten is None else inten.astype(np.float32),
        # per-point `time` is seconds after the header stamp; dimos carries it as u32 ns,
        # which holds 4.29 s (a scan the L1 stalled through spans longer)
        offset_times=None
        if time_s is None
        else np.round(np.clip(np.nan_to_num(time_s), 0.0, 4.29) * 1e9),
        tags=tag,
        lines=line,
    )


# sensor_msgs/CompressedImage
@dataclass
class _CImgWire:
    header: _Header
    format: str
    data: np.ndarray  # u8[]

    __cdr_fields__ = [
        ("header", _Header),
        ("format", "string"),
        ("data", ("seq", "u8")),
    ]


def decode_compressed_image(buf: bytes) -> Image | None:
    import cv2

    w: _CImgWire = cdr.decode(buf, _CImgWire)[0]
    bgr = cv2.imdecode(w.data, cv2.IMREAD_COLOR)
    if bgr is None:
        return None
    return image.from_array(
        bgr,
        encoding="bgr8",
        header=Header(Time(w.header.stamp.sec, w.header.stamp.nanosec), w.header.frame_id),
    )


# foxglove_msgs/CompressedVideo (field order differs from CompressedImage:
# timestamp, frame_id, data, format).
@dataclass
class _CVideoWire:
    stamp: _Time
    frame_id: str
    data: np.ndarray  # u8[]
    format: str

    __cdr_fields__ = [
        ("stamp", _Time),
        ("frame_id", "string"),
        ("data", ("seq", "u8")),
        ("format", "string"),
    ]


def decode_compressed_video(buf: bytes) -> CompressedVideo:
    """Decode the CDR envelope only — the encoded packet is left for H264Decoder."""
    w: _CVideoWire = cdr.decode(buf, _CVideoWire)[0]
    return CompressedVideo(data=w.data, format=w.format, frame_id=w.frame_id)


# unitree_go/HeightMap (rt/utlidar/height_map_array)
@dataclass
class _HeightMapWire:
    stamp: float
    frame_id: str
    resolution: float
    width: int
    height: int
    origin: np.ndarray  # f32[2]
    data: np.ndarray  # f32[]

    __cdr_fields__ = [
        ("stamp", "f64"),
        ("frame_id", "string"),
        ("resolution", "f32"),
        ("width", "u32"),
        ("height", "u32"),
        ("origin", ("array", "f32", 2)),
        ("data", ("seq", "f32")),
    ]


def decode_height_map(buf: bytes) -> HeightMap:
    w: _HeightMapWire = cdr.decode(buf, _HeightMapWire)[0]
    return HeightMap(
        resolution=w.resolution,
        origin=w.origin,
        data=w.data.reshape(w.height, w.width),
        frame_id=w.frame_id,
        ts=w.stamp,
    )

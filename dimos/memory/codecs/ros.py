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

"""Explicit ROS 2 decoded-message → DimOS conversions, independent of transport."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

import cv2
import numpy as np
from open3d.core import Tensor  # type: ignore[import-untyped]

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.PoseWithCovariance import PoseWithCovariance
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistWithCovariance import TwistWithCovariance
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat, _parse_lcm_encoding
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


def _timestamp(header: Any) -> float:
    return float(header.stamp.sec + header.stamp.nanosec / 1e9)


def _vector(value: Any) -> Vector3:
    return Vector3(value.x, value.y, value.z)


def _quaternion(value: Any) -> Quaternion:
    return Quaternion(value.x, value.y, value.z, value.w)


def _pose(value: Any) -> Pose:
    return Pose(position=_vector(value.position), orientation=_quaternion(value.orientation))


def pose_stamped(value: Any) -> PoseStamped:
    return PoseStamped(
        ts=_timestamp(value.header),
        frame_id=value.header.frame_id,
        position=_vector(value.pose.position),
        orientation=_quaternion(value.pose.orientation),
    )


def imu(value: Any) -> Imu:
    return Imu(
        ts=_timestamp(value.header),
        frame_id=value.header.frame_id,
        orientation=_quaternion(value.orientation),
        angular_velocity=_vector(value.angular_velocity),
        linear_acceleration=_vector(value.linear_acceleration),
        orientation_covariance=list(value.orientation_covariance),
        angular_velocity_covariance=list(value.angular_velocity_covariance),
        linear_acceleration_covariance=list(value.linear_acceleration_covariance),
    )


def odometry(value: Any) -> Odometry:
    return Odometry(
        ts=_timestamp(value.header),
        frame_id=value.header.frame_id,
        child_frame_id=value.child_frame_id,
        pose=PoseWithCovariance(_pose(value.pose.pose), list(value.pose.covariance)),
        twist=TwistWithCovariance(
            Twist(_vector(value.twist.twist.linear), _vector(value.twist.twist.angular)),
            list(value.twist.covariance),
        ),
    )


def path(value: Any) -> Path:
    return Path(
        ts=_timestamp(value.header),
        frame_id=value.header.frame_id,
        poses=[pose_stamped(pose) for pose in value.poses],
    )


def tf_message(value: Any) -> TFMessage:
    return TFMessage(
        *[
            Transform(
                ts=_timestamp(transform.header),
                frame_id=transform.header.frame_id,
                child_frame_id=transform.child_frame_id,
                translation=_vector(transform.transform.translation),
                rotation=_quaternion(transform.transform.rotation),
            )
            for transform in value.transforms
        ]
    )


def joint_state(value: Any) -> JointState:
    return JointState(
        ts=_timestamp(value.header),
        frame_id=value.header.frame_id,
        name=list(value.name),
        position=list(value.position),
        velocity=list(value.velocity),
        effort=list(value.effort),
    )


def camera_info(value: Any) -> CameraInfo:
    result = CameraInfo(
        ts=_timestamp(value.header),
        frame_id=value.header.frame_id,
        height=value.height,
        width=value.width,
        distortion_model=value.distortion_model,
        D=list(value.d),
        K=list(value.k),
        R=list(value.r),
        P=list(value.p),
        binning_x=value.binning_x,
        binning_y=value.binning_y,
    )
    result.roi_x_offset = value.roi.x_offset
    result.roi_y_offset = value.roi.y_offset
    result.roi_height = value.roi.height
    result.roi_width = value.roi.width
    result.roi_do_rectify = value.roi.do_rectify
    return result


def image(value: Any) -> Image:
    # These are standard ROS image encoding names, also used by the LCM input.
    fmt, scalar, channels = _parse_lcm_encoding(value.encoding)
    dtype = np.dtype(scalar).newbyteorder(">" if value.is_bigendian else "<")
    raw = bytes(value.data)
    row_size = value.width * channels * dtype.itemsize
    if value.step < row_size or len(raw) != value.height * value.step:
        raise ValueError("Image payload does not match its dimensions and stride")
    shape = (value.height, value.width, channels)
    array = np.ndarray(
        shape,
        dtype=dtype,
        buffer=raw,
        strides=(value.step, channels * dtype.itemsize, dtype.itemsize),
    )
    pixels: np.ndarray = array.astype(scalar, copy=True)
    if channels == 1:
        pixels = pixels[:, :, 0]
    return Image(pixels, fmt, value.header.frame_id, _timestamp(value.header))


def compressed_image(value: Any) -> Image:
    pixels = cv2.imdecode(np.frombuffer(bytes(value.data), dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    if pixels is None:
        raise ValueError("Invalid compressed image")
    fmt = ImageFormat.GRAY if pixels.ndim == 2 else ImageFormat.BGR
    if value.format.startswith(("rgb8;", "rgba8;")):
        pixels = cv2.cvtColor(pixels, cv2.COLOR_BGR2RGB)
        fmt = ImageFormat.RGB
    return Image(pixels, fmt, value.header.frame_id, _timestamp(value.header))


def pointcloud(value: Any) -> PointCloud2:
    """Honor ROS field offsets, row padding and endianness when loading points."""
    if value.width == 0 or value.height == 0:
        return PointCloud2.from_numpy(
            np.empty((0, 3)), value.header.frame_id, _timestamp(value.header)
        )
    primitive = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}
    endian = ">" if value.is_bigendian else "<"
    dtype = np.dtype(
        {
            "names": [field.name for field in value.fields],
            "formats": [
                (endian + primitive[field.datatype], (field.count,)) for field in value.fields
            ],
            "offsets": [field.offset for field in value.fields],
            "itemsize": value.point_step,
        }
    )
    raw = bytes(value.data)
    if value.row_step < value.width * value.point_step or len(raw) != value.height * value.row_step:
        raise ValueError("PointCloud2 payload does not match its dimensions and stride")
    points = np.ndarray(
        (value.height, value.width),
        dtype=dtype,
        buffer=raw,
        strides=(value.row_step, value.point_step),
    )

    def field(name: str) -> np.ndarray | None:
        if name not in (dtype.names or ()):
            return None
        data = points[name]
        if data.shape[-1] != 1:
            raise ValueError(f"Expected scalar PointCloud2 field {name!r}")
        return data.reshape(-1).copy()

    xyz = [field(name) for name in ("x", "y", "z")]
    if any(component is None for component in xyz):
        raise ValueError("PointCloud2 requires x, y, z fields")
    result = PointCloud2.from_numpy(
        np.column_stack([component for component in xyz if component is not None]),
        value.header.frame_id,
        _timestamp(value.header),
        intensities=field("intensity"),
        offset_times=field("offset_time"),
        tags=field("tag"),
        lines=field("line"),
    )
    rgb = field("rgb")
    if rgb is not None:
        packed = rgb.view(np.dtype(endian + "u4")).astype(np.uint32)
        colors = (
            np.column_stack([(packed >> shift) & 255 for shift in (16, 8, 0)]).astype(np.float32)
            / 255
        )
        result.pointcloud_tensor.point["colors"] = Tensor(colors)
    return result


ROS_READERS: dict[str, tuple[type, Callable[[Any], Any]]] = {
    "sensor_msgs/msg/Image": (Image, image),
    "sensor_msgs/msg/CompressedImage": (Image, compressed_image),
    "sensor_msgs/msg/PointCloud2": (PointCloud2, pointcloud),
    "sensor_msgs/msg/CameraInfo": (CameraInfo, camera_info),
    "sensor_msgs/msg/Imu": (Imu, imu),
    "sensor_msgs/msg/JointState": (JointState, joint_state),
    "geometry_msgs/msg/PoseStamped": (PoseStamped, pose_stamped),
    "nav_msgs/msg/Odometry": (Odometry, odometry),
    "nav_msgs/msg/Path": (Path, path),
    "tf2_msgs/msg/TFMessage": (TFMessage, tf_message),
}

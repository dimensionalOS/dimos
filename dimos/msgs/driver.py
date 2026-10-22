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

"""Explicit adapters from internal numerical driver values to canonical wire values."""

from copy import deepcopy

import numpy as np

from dimos.message_utils.image import image_from_array
from dimos.message_utils.time import time_from_seconds
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import (
    Quaternion,
    Transform as CdrTransform,
    TransformStamped,
    Vector3,
)
from dimos.messages.sensor_msgs.msg import (
    CameraInfo as CdrCameraInfo,
    Image as CdrImage,
    Imu as CdrImu,
    PointCloud2 as CdrCloud,
    PointField,
    RegionOfInterest,
)
from dimos.messages.std_msgs.msg import Header
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def camera_info_message(info: CameraInfo) -> CdrCameraInfo:
    return CdrCameraInfo(
        header=Header(time_from_seconds(info.ts), info.frame_id),
        height=info.height,
        width=info.width,
        distortion_model=info.distortion_model,
        d=np.asarray(info.D, dtype=np.float64).copy(),
        k=np.asarray(info.K, dtype=np.float64).copy(),
        r=np.asarray(info.R, dtype=np.float64).copy(),
        p=np.asarray(info.P, dtype=np.float64).copy(),
        binning_x=info.binning_x,
        binning_y=info.binning_y,
        roi=RegionOfInterest(0, 0, 0, 0, False),
    )


def cloud_message(cloud: PointCloud2) -> CdrCloud:
    """Adapt the driver's numerical cloud, preserving every supported point attribute."""
    points, colors = cloud.as_numpy()
    arrays = {name: points[:, index] for index, name in enumerate(("x", "y", "z"))}
    types = {name: ("<f4", PointField.FLOAT32) for name in arrays}
    if colors is not None:
        rgb = np.asarray(colors)
        rgb = (
            (rgb * 255).astype(np.uint8) if rgb.size and rgb.max() <= 1.0 else rgb.astype(np.uint8)
        )
        rgb = rgb.astype(np.uint32)
        arrays["rgb"] = (rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]
        types["rgb"] = ("<u4", PointField.UINT32)
    for name, values, dtype, datatype in (
        ("intensity", cloud.intensities_f32(), "<f4", PointField.FLOAT32),
        ("offset_time", cloud.offset_times_u32(), "<u4", PointField.UINT32),
        ("tag", cloud.tags_u8(), "u1", PointField.UINT8),
        ("line", cloud.lines_u8(), "u1", PointField.UINT8),
        ("stamps", cloud.stamps_f64(), "<f8", PointField.FLOAT64),
    ):
        if values is not None:
            arrays[name] = values
            types[name] = (dtype, datatype)
    packed = np.empty(len(points), dtype=[(name, dtype) for name, (dtype, _) in types.items()])
    fields = []
    assert packed.dtype.fields is not None
    for name, (_, datatype) in types.items():
        packed[name] = arrays[name]
        fields.append(PointField(name, packed.dtype.fields[name][1], datatype, 1))
    return CdrCloud(
        header=Header(time_from_seconds(cloud.ts), cloud.frame_id),
        height=1,
        width=len(points),
        fields=fields,
        is_bigendian=False,
        point_step=packed.dtype.itemsize,
        row_step=len(points) * packed.dtype.itemsize,
        data=np.frombuffer(packed.tobytes(), dtype=np.uint8).copy(),
        is_dense=bool(np.isfinite(points).all()),
    )


def image_message(image: Image) -> CdrImage:
    encodings = {
        "RGB": "rgb8",
        "BGR": "bgr8",
        "RGBA": "rgba8",
        "BGRA": "bgra8",
        "GRAY": "mono8",
        "GRAY16": "mono16",
        "DEPTH": "32FC1",
        "DEPTH16": "16UC1",
    }
    return image_from_array(
        image.data,
        encoding=encodings[image.format.value],
        header=Header(time_from_seconds(image.ts), image.frame_id),
    )


def transform_message(value: Transform, stamp: Time | None = None) -> TransformStamped:
    """Keep local geometry intact, copying the full frames/pose at the wire edge."""
    t, q = value.translation, value.rotation
    return TransformStamped(
        header=Header(
            deepcopy(stamp) if stamp is not None else time_from_seconds(value.ts), value.frame_id
        ),
        child_frame_id=value.child_frame_id,
        transform=CdrTransform(Vector3(t.x, t.y, t.z), Quaternion(q.x, q.y, q.z, q.w)),
    )


def imu_message(value: Imu) -> CdrImu:
    q, g, a = value.orientation, value.angular_velocity, value.linear_acceleration
    return CdrImu(
        header=Header(time_from_seconds(value.ts), value.frame_id),
        orientation=Quaternion(q.x, q.y, q.z, q.w),
        orientation_covariance=np.asarray(value.orientation_covariance, dtype=np.float64).copy(),
        angular_velocity=Vector3(g.x, g.y, g.z),
        angular_velocity_covariance=np.asarray(
            value.angular_velocity_covariance, dtype=np.float64
        ).copy(),
        linear_acceleration=Vector3(a.x, a.y, a.z),
        linear_acceleration_covariance=np.asarray(
            value.linear_acceleration_covariance, dtype=np.float64
        ).copy(),
    )

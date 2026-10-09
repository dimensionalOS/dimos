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

"""Generated ROS2 DDS codecs with explicit Unitree firmware adaptations.

Standard payloads retain all generated fields, exact stamps and arbitrary
point layouts. The device's Imu quaternion uses wxyz and is reordered only at
this Unitree boundary. HeightMap remains a device-specific wire structure.
"""

from dataclasses import dataclass

from dimos_generated.foxglove_msgs.msg import CompressedVideo
from dimos_generated.geometry_msgs.msg import Quaternion
from dimos_generated.nav_msgs.msg import Odometry
from dimos_generated.sensor_msgs.msg import CompressedImage, Imu, PointCloud2
from dimos_message_build.registry import decode as cdr_decode
import numpy as np

from dimos.robot.unitree.go2.dds import cdr
from dimos.robot.unitree.go2.dds.msgs.HeightMap import HeightMap


def decode_imu(buf: bytes) -> Imu:
    message = cdr_decode(buf, Imu)
    firmware = message.orientation
    message.orientation = Quaternion(x=firmware.y, y=firmware.z, z=firmware.w, w=firmware.x)
    return message


def decode_odometry(buf: bytes) -> Odometry:
    return cdr_decode(buf, Odometry)


def decode_pointcloud2(buf: bytes) -> PointCloud2:
    return cdr_decode(buf, PointCloud2)


def decode_compressed_image(buf: bytes) -> CompressedImage:
    return cdr_decode(buf, CompressedImage)


def decode_compressed_video(buf: bytes) -> CompressedVideo:
    return cdr_decode(buf, CompressedVideo)


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

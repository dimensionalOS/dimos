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

import struct

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.foxglove_msgs.msg import CompressedVideo
from dimos_generated.geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    Quaternion,
    Twist,
    TwistWithCovariance,
    Vector3,
)
from dimos_generated.nav_msgs.msg import Odometry
from dimos_generated.sensor_msgs.msg import CompressedImage, Imu, PointCloud2, PointField
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import encode as cdr_encode
import numpy as np
import pytest
from rosbags.typesys import Stores, get_typestore

from dimos.memory.type.observation import Observation
from dimos.msgs.image import image_view
from dimos.msgs.pointcloud import pointcloud_xyz
from dimos.robot.unitree.go2.dds import ros
from dimos.robot.unitree.go2.dds.codec import GO2_CODECS
from dimos.robot.unitree.go2.dds.video import H264Decoder


@pytest.mark.parametrize("little_endian", [True, False])
def test_odometry_preserves_independent_cdr_covariances_and_exact_stamp(little_endian):
    source = Odometry(
        header=Header(stamp=Time(sec=42, nanosec=123456789), frame_id="odom"),
        child_frame_id="base_link",
        pose=PoseWithCovariance(
            pose=Pose(
                position=Point(x=0.0, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            covariance=np.zeros(36, dtype=np.float64),
        ),
        twist=TwistWithCovariance(
            twist=Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
            covariance=np.zeros(36, dtype=np.float64),
        ),
    )
    source.pose.pose.position.x = 1.25
    source.pose.covariance = np.arange(36, dtype=np.float64)
    source.twist.covariance = np.arange(36, 72, dtype=np.float64)
    reference = get_typestore(Stores.ROS2_JAZZY)
    independent = reference.deserialize_cdr(cdr_encode(source), source.__msgtype__)
    payload = bytes(
        reference.serialize_cdr(independent, source.__msgtype__, little_endian=little_endian)
    )
    result = ros.decode_odometry(payload)
    assert result.header.stamp.nanosec == 123456789
    assert result.header.frame_id == "odom" and result.child_frame_id == "base_link"
    assert result.pose.pose.position.x == 1.25
    assert list(result.pose.covariance) == list(range(36))
    assert list(result.twist.covariance) == list(range(36, 72))


def test_pointcloud_preserves_big_endian_fields_and_row_padding():
    source = PointCloud2(
        header=Header(stamp=Time(sec=42, nanosec=123456789), frame_id="sensor"),
        height=1,
        width=1,
        point_step=26,
        row_step=34,
        is_bigendian=True,
        fields=[
            PointField(name=n, offset=i * 8, datatype=8, count=1)
            for i, n in enumerate(["x", "y", "z"])
        ]
        + [PointField(name="intensity", offset=24, datatype=4, count=1)],
        data=np.frombuffer(
            struct.pack(">dddH", 1.25, -2.5, 3.0, 1234) + b"padding!", dtype=np.uint8
        ),
        is_dense=False,
    )
    result = ros.decode_pointcloud2(cdr_encode(source))
    assert cdr_encode(result) == cdr_encode(source)
    assert result.header.stamp.nanosec == 123456789
    np.testing.assert_array_equal(pointcloud_xyz(result), [[1.25, -2.5, 3.0]])


def test_unitree_imu_reorders_firmware_quaternion_without_losing_metadata():
    source = Imu(
        header=Header(stamp=Time(nanosec=123456789, sec=0), frame_id="imu"),
        orientation=Quaternion(x=0.5, y=0.1, z=0.2, w=0.3),
        orientation_covariance=np.zeros(9, dtype=np.float64),
        angular_velocity=Vector3(x=0.0, y=0.0, z=0.0),
        angular_velocity_covariance=np.zeros(9, dtype=np.float64),
        linear_acceleration=Vector3(x=0.0, y=0.0, z=0.0),
        linear_acceleration_covariance=np.zeros(9, dtype=np.float64),
    )
    source.orientation_covariance = np.arange(9, dtype=np.float64)
    result = ros.decode_imu(cdr_encode(source))
    q = result.orientation
    assert (q.x, q.y, q.z, q.w) == (0.1, 0.2, 0.3, 0.5)
    assert list(result.orientation_covariance) == list(range(9))
    assert result.header.stamp.nanosec == 123456789


def test_compressed_image_stays_a_compressed_generated_message():
    source = CompressedImage(
        header=Header(frame_id="camera", stamp=Time(sec=0, nanosec=0)),
        format="jpeg",
        data=np.frombuffer(b"encoded-packet", dtype=np.uint8),
    )
    result = GO2_CODECS["rt/frontvideo"].decode(cdr_encode(source))
    assert isinstance(result, CompressedImage)
    assert cdr_encode(result) == cdr_encode(source)


def test_h264_emission_preserves_packet_stamp_and_pixel_layout(mocker):
    stamp = Time(sec=42, nanosec=123456789)
    packet = CompressedVideo(
        timestamp=stamp,
        frame_id="camera",
        format="h264",
        data=np.frombuffer(b"packet", dtype=np.uint8),
    )
    source = Observation(id=7, ts=99.0, data_type=CompressedVideo, _data=packet)
    frame = mocker.Mock()
    pixels = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
    frame.to_ndarray.return_value = pixels
    result = H264Decoder._emit(frame, source)
    assert result.id == 7 and result.ts == 99.0
    assert result.data.encoding == "bgr8" and result.data.header.frame_id == "camera"
    assert result.data.header.stamp.sec == 42 and result.data.header.stamp.nanosec == 123456789
    np.testing.assert_array_equal(image_view(result.data), pixels)

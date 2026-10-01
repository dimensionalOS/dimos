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

"""Conformance with installed ROS type support; skipped when ROS is absent."""

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import (
    Point,
    PointStamped,
    Pose,
    PoseStamped,
    Quaternion,
    TransformStamped,
    Vector3,
)
from dimos_generated.sensor_msgs.msg import (
    CameraInfo,
    CompressedImage,
    Image,
    PointCloud2,
    PointField,
)
from dimos_generated.std_msgs.msg import Header
from dimos_generated.tf2_msgs.msg import TFMessage
import pytest

from dimos.protocol.pubsub.impl import rospubsub_conversion as conversion

requires_ros = pytest.mark.skipif(conversion.ros_serialization is None, reason="requires ROS 2")


def messages():
    header = Header(stamp=Time(sec=1700000000, nanosec=123456789), frame_id="map/相機")
    return [
        Point(x=1.25, y=-2.5, z=3.75),
        PointStamped(header=header, point=Point(x=2.0)),
        PoseStamped(header=header, pose=Pose(orientation=Quaternion(z=0.6, w=0.8))),
        Image(
            header=header,
            height=1,
            width=2,
            encoding="rgb8",
            step=6,
            data=bytes([0, 1, 2, 253, 254, 255]),
        ),
        CompressedImage(header=header, format="jpeg", data=bytes([0, 255, 127])),
        CameraInfo(
            header=header,
            width=640,
            height=480,
            distortion_model="plumb_bob",
            d=[0.1, -0.2],
            k=[1.0, 0.0, 3.0, 0.0, 2.0, 4.0, 0.0, 0.0, 1.0],
        ),
        PointCloud2(
            header=header,
            height=1,
            width=1,
            fields=[PointField(name="x", offset=4, datatype=7, count=1)],
            is_bigendian=True,
            point_step=8,
            row_step=12,
            data=bytes([9, 8, 7, 6, 0x3F, 0x80, 0, 0, 5, 4, 3, 2]),
        ),
        PointCloud2(header=header),
        TFMessage(),
        TFMessage(transforms=[TransformStamped(header=header, child_frame_id="camera")]),
    ]


@requires_ros
@pytest.mark.parametrize("original", messages(), ids=lambda msg: msg.msg_name)
def test_generated_fields_survive_ros_serialization(original):
    ros_type = conversion.derive_ros_type(type(original))
    ros_message = conversion.dimos_to_ros(original, ros_type)
    converted = conversion.ros_to_dimos(ros_message, type(original))
    assert converted == original


@requires_ros
@pytest.mark.parametrize("sec,nanosec", [(0, 0), (-1, 999999999), (1700000000, 123456789)])
def test_ros_origin_timestamp_and_nested_fields_are_exact(sec, nanosec):
    ros_type = conversion.derive_ros_type(PointStamped)
    source = ros_type()
    source.header.stamp.sec = sec
    source.header.stamp.nanosec = nanosec
    source.header.frame_id = "odom"
    source.point.x, source.point.y, source.point.z = 1.5, -2.5, 3.5

    converted = conversion.ros_to_dimos(source, PointStamped)
    assert converted == PointStamped(
        header=Header(stamp=Time(sec=sec, nanosec=nanosec), frame_id="odom"),
        point=Point(x=1.5, y=-2.5, z=3.5),
    )
    assert conversion.dimos_to_ros(converted, ros_type) == source


@requires_ros
def test_matching_layout_does_not_allow_wrong_type():
    from_type = conversion.derive_ros_type(Point)
    to_type = conversion.derive_ros_type(Vector3)
    with pytest.raises(TypeError, match="target does not match"):
        conversion.dimos_to_ros(Point(), to_type)
    with pytest.raises(TypeError, match="source does not match"):
        conversion.ros_to_dimos(from_type(), Vector3)


def test_missing_ros_is_explicit_and_does_not_affect_generated_messages(monkeypatch):
    monkeypatch.setattr(conversion, "ros_serialization", None)
    message = Point(x=1.0)
    assert Point.decode(message.encode()) == message
    with pytest.raises(ImportError, match="requires rclpy"):
        conversion.dimos_to_ros(message, Point)
    with pytest.raises(ImportError, match="requires rclpy"):
        conversion.ros_to_dimos(message, Point)


@pytest.mark.parametrize("name", ["geometry_msgs.Point", "geometry_msgs/srv/Point", "../msg/Point"])
def test_invalid_type_name_fails_before_import(name):
    invalid = type("Invalid", (), {"msg_name": name})
    with pytest.raises(ValueError, match="expected package/msg/Type"):
        conversion.derive_ros_type(invalid)

# Copyright 2025 Dimensional Inc.
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

import pytest

try:
    from geometry_msgs.msg import TransformStamped as ROSTransformStamped
    from tf2_msgs.msg import TFMessage as ROSTFMessage
except ImportError:
    ROSTransformStamped = None
    ROSTFMessage = None

from dimos_lcm.tf2_msgs import TFMessage as LCMTFMessage

from dimos.msgs.geometry_msgs import Quaternion, Transform, Vector3
from dimos.msgs.tf2_msgs import TFMessage


def test_tfmessage_initialization() -> None:
    """Test TFMessage initialization with Transform objects."""
    # Create some transforms
    tf1 = Transform(
        translation=Vector3(1, 2, 3), rotation=Quaternion(0, 0, 0, 1), frame_id="world", ts=100.0
    )
    tf2 = Transform(
        translation=Vector3(4, 5, 6),
        rotation=Quaternion(0, 0, 0.707, 0.707),
        frame_id="map",
        ts=101.0,
    )

    # Create TFMessage with transforms
    msg = TFMessage(tf1, tf2)

    assert len(msg) == 2
    assert msg[0] == tf1
    assert msg[1] == tf2

    # Test iteration
    transforms = list(msg)
    assert transforms == [tf1, tf2]


def test_tfmessage_empty() -> None:
    """Test empty TFMessage."""
    msg = TFMessage()
    assert len(msg) == 0
    assert list(msg) == []


def test_tfmessage_add_transform() -> None:
    """Test adding transforms to TFMessage."""
    msg = TFMessage()

    tf = Transform(translation=Vector3(1, 2, 3), frame_id="base", ts=200.0)

    msg.add_transform(tf)
    assert len(msg) == 1
    assert msg[0] == tf


def test_tfmessage_lcm_encode_decode() -> None:
    """Test encoding TFMessage to LCM bytes."""
    # Create transforms
    tf1 = Transform(
        translation=Vector3(1.0, 2.0, 3.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        child_frame_id="robot",
        frame_id="world",
        ts=123.456,
    )
    tf2 = Transform(
        translation=Vector3(4.0, 5.0, 6.0),
        rotation=Quaternion(0.0, 0.0, 0.707, 0.707),
        frame_id="robot",
        child_frame_id="target",
        ts=124.567,
    )

    # Create TFMessage
    msg = TFMessage(tf1, tf2)

    # Encode with custom child_frame_ids
    encoded = msg.lcm_encode()

    # Decode using LCM to verify
    lcm_msg = LCMTFMessage.lcm_decode(encoded)

    assert lcm_msg.transforms_length == 2

    # Check first transform
    ts1 = lcm_msg.transforms[0]
    assert ts1.header.frame_id == "world"
    assert ts1.child_frame_id == "robot"
    assert ts1.header.stamp.sec == 123
    assert ts1.header.stamp.nsec == 456000000
    assert ts1.transform.translation.x == 1.0
    assert ts1.transform.translation.y == 2.0
    assert ts1.transform.translation.z == 3.0

    # Check second transform
    ts2 = lcm_msg.transforms[1]
    assert ts2.header.frame_id == "robot"
    assert ts2.child_frame_id == "target"
    assert ts2.transform.rotation.z == 0.707
    assert ts2.transform.rotation.w == 0.707


@pytest.mark.ros
def test_tfmessage_from_ros_msg() -> None:
    """Test creating a TFMessage from a ROS TFMessage message."""

    ros_msg = ROSTFMessage()

    # Add first transform
    tf1 = ROSTransformStamped()
    tf1.header.frame_id = "world"
    tf1.header.stamp.sec = 123
    tf1.header.stamp.nanosec = 456000000
    tf1.child_frame_id = "robot"
    tf1.transform.translation.x = 1.0
    tf1.transform.translation.y = 2.0
    tf1.transform.translation.z = 3.0
    tf1.transform.rotation.x = 0.0
    tf1.transform.rotation.y = 0.0
    tf1.transform.rotation.z = 0.0
    tf1.transform.rotation.w = 1.0
    ros_msg.transforms.append(tf1)

    # Add second transform
    tf2 = ROSTransformStamped()
    tf2.header.frame_id = "robot"
    tf2.header.stamp.sec = 124
    tf2.header.stamp.nanosec = 567000000
    tf2.child_frame_id = "sensor"
    tf2.transform.translation.x = 4.0
    tf2.transform.translation.y = 5.0
    tf2.transform.translation.z = 6.0
    tf2.transform.rotation.x = 0.0
    tf2.transform.rotation.y = 0.0
    tf2.transform.rotation.z = 0.707
    tf2.transform.rotation.w = 0.707
    ros_msg.transforms.append(tf2)

    # Convert to TFMessage
    tfmsg = TFMessage.from_ros_msg(ros_msg)

    assert len(tfmsg) == 2

    # Check first transform
    assert tfmsg[0].frame_id == "world"
    assert tfmsg[0].child_frame_id == "robot"
    assert tfmsg[0].ts == 123.456
    assert tfmsg[0].translation.x == 1.0
    assert tfmsg[0].translation.y == 2.0
    assert tfmsg[0].translation.z == 3.0
    assert tfmsg[0].rotation.w == 1.0

    # Check second transform
    assert tfmsg[1].frame_id == "robot"
    assert tfmsg[1].child_frame_id == "sensor"
    assert tfmsg[1].ts == 124.567
    assert tfmsg[1].translation.x == 4.0
    assert tfmsg[1].translation.y == 5.0
    assert tfmsg[1].translation.z == 6.0
    assert tfmsg[1].rotation.z == 0.707
    assert tfmsg[1].rotation.w == 0.707


@pytest.mark.ros
def test_tfmessage_to_ros_msg() -> None:
    """Test converting a TFMessage to a ROS TFMessage message."""
    # Create transforms
    tf1 = Transform(
        translation=Vector3(1.0, 2.0, 3.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id="map",
        child_frame_id="base_link",
        ts=123.456,
    )
    tf2 = Transform(
        translation=Vector3(7.0, 8.0, 9.0),
        rotation=Quaternion(0.1, 0.2, 0.3, 0.9),
        frame_id="base_link",
        child_frame_id="lidar",
        ts=125.789,
    )

    tfmsg = TFMessage(tf1, tf2)

    # Convert to ROS message
    ros_msg = tfmsg.to_ros_msg()

    assert isinstance(ros_msg, ROSTFMessage)
    assert len(ros_msg.transforms) == 2

    # Check first transform
    assert ros_msg.transforms[0].header.frame_id == "map"
    assert ros_msg.transforms[0].child_frame_id == "base_link"
    assert ros_msg.transforms[0].header.stamp.sec == 123
    assert ros_msg.transforms[0].header.stamp.nanosec == 456000000
    assert ros_msg.transforms[0].transform.translation.x == 1.0
    assert ros_msg.transforms[0].transform.translation.y == 2.0
    assert ros_msg.transforms[0].transform.translation.z == 3.0
    assert ros_msg.transforms[0].transform.rotation.w == 1.0

    # Check second transform
    assert ros_msg.transforms[1].header.frame_id == "base_link"
    assert ros_msg.transforms[1].child_frame_id == "lidar"
    assert ros_msg.transforms[1].header.stamp.sec == 125
    assert ros_msg.transforms[1].header.stamp.nanosec == 789000000
    assert ros_msg.transforms[1].transform.translation.x == 7.0
    assert ros_msg.transforms[1].transform.translation.y == 8.0
    assert ros_msg.transforms[1].transform.translation.z == 9.0
    assert ros_msg.transforms[1].transform.rotation.x == 0.1
    assert ros_msg.transforms[1].transform.rotation.y == 0.2
    assert ros_msg.transforms[1].transform.rotation.z == 0.3
    assert ros_msg.transforms[1].transform.rotation.w == 0.9


@pytest.mark.ros
def test_tfmessage_ros_roundtrip() -> None:
    """Test round-trip conversion between TFMessage and ROS TFMessage."""
    # Create transforms with various properties
    tf1 = Transform(
        translation=Vector3(1.5, 2.5, 3.5),
        rotation=Quaternion(0.15, 0.25, 0.35, 0.85),
        frame_id="odom",
        child_frame_id="base_footprint",
        ts=100.123,
    )
    tf2 = Transform(
        translation=Vector3(0.1, 0.2, 0.3),
        rotation=Quaternion(0.0, 0.0, 0.383, 0.924),
        frame_id="base_footprint",
        child_frame_id="camera",
        ts=100.456,
    )

    original = TFMessage(tf1, tf2)

    # Convert to ROS and back
    ros_msg = original.to_ros_msg()
    restored = TFMessage.from_ros_msg(ros_msg)

    assert len(restored) == len(original)

    for orig_tf, rest_tf in zip(original, restored, strict=False):
        assert rest_tf.frame_id == orig_tf.frame_id
        assert rest_tf.child_frame_id == orig_tf.child_frame_id
        assert rest_tf.ts == orig_tf.ts
        assert rest_tf.translation.x == orig_tf.translation.x
        assert rest_tf.translation.y == orig_tf.translation.y
        assert rest_tf.translation.z == orig_tf.translation.z
        assert rest_tf.rotation.x == orig_tf.rotation.x
        assert rest_tf.rotation.y == orig_tf.rotation.y
        assert rest_tf.rotation.z == orig_tf.rotation.z
        assert rest_tf.rotation.w == orig_tf.rotation.w

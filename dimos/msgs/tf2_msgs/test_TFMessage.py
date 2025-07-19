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
from dimos_lcm.tf2_msgs import TFMessage as LCMTFMessage

from dimos.msgs.geometry_msgs import Quaternion, Transform, Vector3
from dimos.msgs.tf2_msgs import TFMessage


def test_tfmessage_initialization():
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


def test_tfmessage_empty():
    """Test empty TFMessage."""
    msg = TFMessage()
    assert len(msg) == 0
    assert list(msg) == []


def test_tfmessage_add_transform():
    """Test adding transforms to TFMessage."""
    msg = TFMessage()

    tf = Transform(translation=Vector3(1, 2, 3), frame_id="base", ts=200.0)

    msg.add_transform(tf)
    assert len(msg) == 1
    assert msg[0] == tf


def test_tfmessage_lcm_encode():
    """Test encoding TFMessage to LCM bytes."""
    # Create transforms
    tf1 = Transform(
        translation=Vector3(1.0, 2.0, 3.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id="world",
        ts=123.456,
    )
    tf2 = Transform(
        translation=Vector3(4.0, 5.0, 6.0),
        rotation=Quaternion(0.0, 0.0, 0.707, 0.707),
        frame_id="map",
        ts=124.567,
    )

    # Create TFMessage
    msg = TFMessage(tf1, tf2)

    # Encode with custom child_frame_ids
    encoded = msg.lcm_encode(child_frame_ids=["robot", "sensor"])

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
    assert ts2.header.frame_id == "map"
    assert ts2.child_frame_id == "sensor"
    assert ts2.transform.rotation.z == 0.707
    assert ts2.transform.rotation.w == 0.707


def test_tfmessage_lcm_encode_default_child_ids():
    """Test encoding with default child_frame_ids."""
    tf = Transform(translation=Vector3(1, 2, 3), frame_id="odom", ts=100.0)
    msg = TFMessage(tf)

    encoded = msg.lcm_encode()  # No child_frame_ids provided

    lcm_msg = LCMTFMessage.lcm_decode(encoded)
    assert lcm_msg.transforms[0].child_frame_id == "base_link"


def test_tfmessage_lcm_decode():
    """Test decoding TFMessage from LCM bytes."""
    # Create original message
    tf1 = Transform(
        translation=Vector3(10, 20, 30),
        rotation=Quaternion(0, 0, 0, 1),
        frame_id="base",
        ts=500.123,
    )
    tf2 = Transform(
        translation=Vector3(40, 50, 60),
        rotation=Quaternion(0.5, 0.5, 0.5, 0.5),
        frame_id="camera",
        ts=501.234,
    )

    original = TFMessage(tf1, tf2)

    # Encode and decode
    encoded = original.lcm_encode()
    decoded = TFMessage.lcm_decode(encoded)

    # Verify
    assert len(decoded) == 2

    # Check first transform
    assert decoded[0].translation.x == 10
    assert decoded[0].translation.y == 20
    assert decoded[0].translation.z == 30
    assert decoded[0].frame_id == "base"
    assert abs(decoded[0].ts - 500.123) < 1e-6

    # Check second transform
    assert decoded[1].translation.x == 40
    assert decoded[1].rotation.x == 0.5
    assert decoded[1].rotation.y == 0.5
    assert decoded[1].rotation.z == 0.5
    assert decoded[1].rotation.w == 0.5
    assert decoded[1].frame_id == "camera"


def test_tfmessage_string_representations():
    """Test string representations of TFMessage."""
    tf1 = Transform(frame_id="world", ts=100.123)
    tf2 = Transform(frame_id="map", ts=101.456)

    msg = TFMessage(tf1, tf2)

    # Test repr
    repr_str = repr(msg)
    assert "TFMessage" in repr_str
    assert "2 transforms" in repr_str

    # Test str
    str_str = str(msg)
    assert "TFMessage with 2 transforms:" in str_str
    assert "[0] world @ 100.123" in str_str
    assert "[1] map @ 101.456" in str_str

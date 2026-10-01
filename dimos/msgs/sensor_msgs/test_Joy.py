#!/usr/bin/env python3
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


from dimos_generated.sensor_msgs.msg import Joy
from dimos_generated.std_msgs.msg import Header
import pytest
from rosbags.typesys import Stores, get_typestore

from dimos.msgs.time import time_from_nanoseconds, time_from_seconds, to_nanoseconds


def test_cdr_encode_decode() -> None:
    """Test CDR encode/decode preserves Joy data."""
    print("Testing Joy CDR encode/decode...")
    original = Joy(
        axes=[0.5, -0.25, 1.0, -1.0, 0.0, 0.75],
        buttons=[1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0],
        header=Header(stamp=time_from_seconds(1234567890.1234567), frame_id="gamepad"),
    )
    encoded = original.encode()
    assert isinstance(encoded, bytes)
    assert len(encoded) > 0
    decoded = Joy.decode(encoded)
    assert abs(to_nanoseconds(decoded.header.stamp) - to_nanoseconds(original.header.stamp)) < 1e-09
    assert decoded.header.frame_id == original.header.frame_id
    assert decoded.axes == original.axes
    assert decoded.buttons == original.buttons
    print("✓ Joy CDR encode/decode test passed")


def test_initialization_methods() -> None:
    """Test various initialization methods for Joy."""
    print("Testing Joy initialization methods...")
    joy1 = Joy()
    assert joy1.axes == []
    assert joy1.buttons == []
    assert joy1.header.frame_id == ""
    assert to_nanoseconds(joy1.header.stamp) == 0
    joy2 = Joy(
        axes=[0.1, 0.2, 0.3],
        buttons=[1, 0, 1],
        header=Header(stamp=time_from_seconds(1234567890.0), frame_id="xbox_controller"),
    )
    assert to_nanoseconds(joy2.header.stamp) == 1234567890000000000
    assert joy2.header.frame_id == "xbox_controller"
    assert list(joy2.axes) == pytest.approx([0.1, 0.2, 0.3], abs=1e-6)
    assert joy2.buttons == [1, 0, 1]
    joy3 = Joy(axes=[0.5, -0.5], buttons=[1, 1, 0])
    assert joy3.axes == [0.5, -0.5]
    assert joy3.buttons == [1, 1, 0]
    joy4 = Joy(axes=[0.7, 0.8], buttons=[0, 1], header=Header(frame_id="ps4_controller"))
    assert list(joy4.axes) == pytest.approx([0.7, 0.8], abs=1e-6)
    assert joy4.buttons == [0, 1]
    assert joy4.header.frame_id == "ps4_controller"
    joy5 = Joy.decode(joy2.encode())
    assert to_nanoseconds(joy5.header.stamp) == to_nanoseconds(joy2.header.stamp)
    assert joy5.header.frame_id == joy2.header.frame_id
    assert joy5.axes == joy2.axes
    assert joy5.buttons == joy2.buttons
    assert joy5 is not joy2
    print("✓ Joy initialization methods test passed")


def test_equality() -> None:
    """Test Joy equality comparison."""
    print("Testing Joy equality...")
    joy1 = Joy(
        axes=[0.5, -0.5],
        buttons=[1, 0, 1],
        header=Header(stamp=time_from_seconds(1000.0), frame_id="controller1"),
    )
    joy2 = Joy(
        axes=[0.5, -0.5],
        buttons=[1, 0, 1],
        header=Header(stamp=time_from_seconds(1000.0), frame_id="controller1"),
    )
    joy3 = Joy(
        axes=[0.5, -0.5],
        buttons=[1, 0, 1],
        header=Header(stamp=time_from_seconds(1000.0), frame_id="controller2"),
    )
    joy4 = Joy(
        axes=[0.6, -0.5],
        buttons=[1, 0, 1],
        header=Header(stamp=time_from_seconds(1000.0), frame_id="controller1"),
    )
    assert joy1 == joy2
    assert joy1 != joy3
    assert joy1 != joy4
    assert joy1 != "not a joy"
    assert joy1 != 42
    print("✓ Joy equality test passed")


def test_independent_schema_decoding() -> None:
    joy = Joy(
        header=Header(stamp=time_from_nanoseconds(1234567890123456789), frame_id="test_controller"),
        axes=[0.1, -0.2, 0.3, 0.4],
        buttons=[1, 0, 1, 0, 0, 1],
    )
    decoded = get_typestore(Stores.ROS2_JAZZY).deserialize_cdr(joy.encode(), Joy.msg_name)
    assert decoded.header.frame_id == "test_controller"
    assert decoded.header.stamp.sec == 1234567890
    assert decoded.header.stamp.nanosec == 123456789
    assert list(decoded.axes) == pytest.approx([0.1, -0.2, 0.3, 0.4], abs=1e-6)
    assert list(decoded.buttons) == [1, 0, 1, 0, 0, 1]


def test_edge_cases() -> None:
    """Test Joy with edge cases."""
    print("Testing Joy edge cases...")
    joy1 = Joy(axes=[], buttons=[])
    assert joy1.axes == []
    assert joy1.buttons == []
    encoded = joy1.encode()
    decoded = Joy.decode(encoded)
    assert decoded.axes == []
    assert decoded.buttons == []
    many_axes = [float(i) / 100.0 for i in range(20)]
    many_buttons = [i % 2 for i in range(32)]
    joy2 = Joy(axes=many_axes, buttons=many_buttons)
    assert len(joy2.axes) == 20
    assert len(joy2.buttons) == 32
    encoded = joy2.encode()
    decoded = Joy.decode(encoded)
    assert len(decoded.axes) == len(many_axes)
    for i, (a, b) in enumerate(zip(decoded.axes, many_axes, strict=False)):
        assert abs(a - b) < 1e-06, f"Axis {i}: {a} != {b}"
    assert decoded.buttons == many_buttons
    extreme_axes = [-1.0, 1.0, 0.0, -0.999999, 0.999999]
    joy3 = Joy(axes=extreme_axes)
    assert list(joy3.axes) == pytest.approx(extreme_axes, abs=1e-6)
    print("✓ Joy edge cases test passed")

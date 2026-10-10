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


from dataclasses import asdict

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.sensor_msgs.msg import Joy
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
import pytest
from rosbags.typesys import Stores, get_typestore

from dimos.msgs.time import time_from_nanoseconds, time_from_seconds, to_nanoseconds


def test_cdr_encode_decode() -> None:
    """Test CDR encode/decode preserves Joy data."""
    print("Testing Joy CDR encode/decode...")
    original = Joy(
        axes=np.array([0.5, -0.25, 1.0, -1.0, 0.0, 0.75], dtype=np.float32),
        buttons=np.array([1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0], dtype=np.int32),
        header=Header(stamp=time_from_seconds(1234567890.1234567), frame_id="gamepad"),
    )
    encoded = cdr_encode(original)
    assert isinstance(encoded, bytes)
    assert len(encoded) > 0
    decoded = cdr_decode(encoded, Joy)
    assert abs(to_nanoseconds(decoded.header.stamp) - to_nanoseconds(original.header.stamp)) < 1e-09
    assert decoded.header.frame_id == original.header.frame_id
    np.testing.assert_array_equal(decoded.axes, original.axes)
    np.testing.assert_array_equal(decoded.buttons, original.buttons)
    print("✓ Joy CDR encode/decode test passed")


def test_initialization_methods() -> None:
    """Test various initialization methods for Joy."""
    print("Testing Joy initialization methods...")
    joy1 = Joy(
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        axes=np.array([], dtype=np.float32),
        buttons=np.array([], dtype=np.int32),
    )
    np.testing.assert_array_equal(joy1.axes, [])
    np.testing.assert_array_equal(joy1.buttons, [])
    assert joy1.header.frame_id == ""
    assert to_nanoseconds(joy1.header.stamp) == 0
    joy2 = Joy(
        axes=np.array([0.1, 0.2, 0.3], dtype=np.float32),
        buttons=np.array([1, 0, 1], dtype=np.int32),
        header=Header(stamp=time_from_seconds(1234567890.0), frame_id="xbox_controller"),
    )
    assert to_nanoseconds(joy2.header.stamp) == 1234567890000000000
    assert joy2.header.frame_id == "xbox_controller"
    assert list(joy2.axes) == pytest.approx([0.1, 0.2, 0.3], abs=1e-6)
    np.testing.assert_array_equal(joy2.buttons, [1, 0, 1])
    joy3 = Joy(
        axes=np.array([0.5, -0.5], dtype=np.float32),
        buttons=np.array([1, 1, 0], dtype=np.int32),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    np.testing.assert_array_equal(joy3.axes, [0.5, -0.5])
    np.testing.assert_array_equal(joy3.buttons, [1, 1, 0])
    joy4 = Joy(
        axes=np.array([0.7, 0.8], dtype=np.float32),
        buttons=np.array([0, 1], dtype=np.int32),
        header=Header(frame_id="ps4_controller", stamp=Time(sec=0, nanosec=0)),
    )
    assert list(joy4.axes) == pytest.approx([0.7, 0.8], abs=1e-6)
    np.testing.assert_array_equal(joy4.buttons, [0, 1])
    assert joy4.header.frame_id == "ps4_controller"
    joy5 = cdr_decode(cdr_encode(joy2), Joy)
    assert to_nanoseconds(joy5.header.stamp) == to_nanoseconds(joy2.header.stamp)
    assert joy5.header.frame_id == joy2.header.frame_id
    np.testing.assert_array_equal(joy5.axes, joy2.axes)
    np.testing.assert_array_equal(joy5.buttons, joy2.buttons)
    assert joy5 is not joy2
    print("✓ Joy initialization methods test passed")


def test_equality() -> None:
    """Test Joy equality comparison."""
    print("Testing Joy equality...")
    joy1 = Joy(
        axes=np.array([0.5, -0.5], dtype=np.float32),
        buttons=np.array([1, 0, 1], dtype=np.int32),
        header=Header(stamp=time_from_seconds(1000.0), frame_id="controller1"),
    )
    joy2 = Joy(
        axes=np.array([0.5, -0.5], dtype=np.float32),
        buttons=np.array([1, 0, 1], dtype=np.int32),
        header=Header(stamp=time_from_seconds(1000.0), frame_id="controller1"),
    )
    joy3 = Joy(
        axes=np.array([0.5, -0.5], dtype=np.float32),
        buttons=np.array([1, 0, 1], dtype=np.int32),
        header=Header(stamp=time_from_seconds(1000.0), frame_id="controller2"),
    )
    joy4 = Joy(
        axes=np.array([0.6, -0.5], dtype=np.float32),
        buttons=np.array([1, 0, 1], dtype=np.int32),
        header=Header(stamp=time_from_seconds(1000.0), frame_id="controller1"),
    )
    np.testing.assert_equal(asdict(joy1), asdict(joy2))
    assert joy1.header != joy3.header
    assert not np.array_equal(joy1.axes, joy4.axes)
    assert joy1 != "not a joy"
    assert joy1 != 42
    print("✓ Joy equality test passed")


def test_independent_schema_decoding() -> None:
    joy = Joy(
        header=Header(stamp=time_from_nanoseconds(1234567890123456789), frame_id="test_controller"),
        axes=np.array([0.1, -0.2, 0.3, 0.4], dtype=np.float32),
        buttons=np.array([1, 0, 1, 0, 0, 1], dtype=np.int32),
    )
    decoded = get_typestore(Stores.ROS2_JAZZY).deserialize_cdr(cdr_encode(joy), Joy.__msgtype__)
    assert decoded.header.frame_id == "test_controller"
    assert decoded.header.stamp.sec == 1234567890
    assert decoded.header.stamp.nanosec == 123456789
    assert list(decoded.axes) == pytest.approx([0.1, -0.2, 0.3, 0.4], abs=1e-6)
    assert list(decoded.buttons) == [1, 0, 1, 0, 0, 1]


def test_edge_cases() -> None:
    """Test Joy with edge cases."""
    print("Testing Joy edge cases...")
    joy1 = Joy(
        axes=np.array([], dtype=np.float32),
        buttons=np.array([], dtype=np.int32),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    np.testing.assert_array_equal(joy1.axes, [])
    np.testing.assert_array_equal(joy1.buttons, [])
    encoded = cdr_encode(joy1)
    decoded = cdr_decode(encoded, Joy)
    np.testing.assert_array_equal(decoded.axes, [])
    np.testing.assert_array_equal(decoded.buttons, [])
    many_axes = [float(i) / 100.0 for i in range(20)]
    many_buttons = [i % 2 for i in range(32)]
    joy2 = Joy(
        axes=np.asarray(many_axes, dtype=np.float32),
        buttons=np.asarray(many_buttons, dtype=np.int32),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    assert len(joy2.axes) == 20
    assert len(joy2.buttons) == 32
    encoded = cdr_encode(joy2)
    decoded = cdr_decode(encoded, Joy)
    assert len(decoded.axes) == len(many_axes)
    for i, (a, b) in enumerate(zip(decoded.axes, many_axes, strict=False)):
        assert abs(a - b) < 1e-06, f"Axis {i}: {a} != {b}"
    np.testing.assert_array_equal(decoded.buttons, many_buttons)
    extreme_axes = [-1.0, 1.0, 0.0, -0.999999, 0.999999]
    joy3 = Joy(
        axes=np.asarray(extreme_axes, dtype=np.float32),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        buttons=np.array([], dtype=np.int32),
    )
    assert list(joy3.axes) == pytest.approx(extreme_axes, abs=1e-6)
    print("✓ Joy edge cases test passed")

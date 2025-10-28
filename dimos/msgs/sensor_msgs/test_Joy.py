#!/usr/bin/env python3
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
    from sensor_msgs.msg import Joy as ROSJoy
    from std_msgs.msg import Header as ROSHeader

    ROS_AVAILABLE = True
except ImportError:
    ROSJoy = None
    ROSHeader = None
    ROS_AVAILABLE = False

from dimos.msgs.sensor_msgs.Joy import Joy


def test_lcm_encode_decode() -> None:
    """Test LCM encode/decode preserves Joy data."""
    print("Testing Joy LCM encode/decode...")

    # Create test joy message with sample gamepad data
    original = Joy(
        ts=1234567890.123456789,
        frame_id="gamepad",
        axes=[0.5, -0.25, 1.0, -1.0, 0.0, 0.75],  # 6 axes (e.g., left/right sticks + triggers)
        buttons=[1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0],  # 12 buttons
    )

    # Encode to LCM bytes
    encoded = original.lcm_encode()
    assert isinstance(encoded, bytes)
    assert len(encoded) > 0

    # Decode back
    decoded = Joy.lcm_decode(encoded)

    # Verify all fields match
    assert abs(decoded.ts - original.ts) < 1e-9
    assert decoded.frame_id == original.frame_id
    assert decoded.axes == original.axes
    assert decoded.buttons == original.buttons

    print("✓ Joy LCM encode/decode test passed")


def test_initialization_methods() -> None:
    """Test various initialization methods for Joy."""
    print("Testing Joy initialization methods...")

    # Test default initialization
    joy1 = Joy()
    assert joy1.axes == []
    assert joy1.buttons == []
    assert joy1.frame_id == ""
    assert joy1.ts > 0  # Should have current time

    # Test full initialization
    joy2 = Joy(ts=1234567890.0, frame_id="xbox_controller", axes=[0.1, 0.2, 0.3], buttons=[1, 0, 1])
    assert joy2.ts == 1234567890.0
    assert joy2.frame_id == "xbox_controller"
    assert joy2.axes == [0.1, 0.2, 0.3]
    assert joy2.buttons == [1, 0, 1]

    # Test tuple initialization
    joy3 = Joy(([0.5, -0.5], [1, 1, 0]))
    assert joy3.axes == [0.5, -0.5]
    assert joy3.buttons == [1, 1, 0]

    # Test dict initialization
    joy4 = Joy({"axes": [0.7, 0.8], "buttons": [0, 1], "frame_id": "ps4_controller"})
    assert joy4.axes == [0.7, 0.8]
    assert joy4.buttons == [0, 1]
    assert joy4.frame_id == "ps4_controller"

    # Test copy constructor
    joy5 = Joy(joy2)
    assert joy5.ts == joy2.ts
    assert joy5.frame_id == joy2.frame_id
    assert joy5.axes == joy2.axes
    assert joy5.buttons == joy2.buttons
    assert joy5 is not joy2  # Different objects

    print("✓ Joy initialization methods test passed")


def test_equality() -> None:
    """Test Joy equality comparison."""
    print("Testing Joy equality...")

    joy1 = Joy(ts=1000.0, frame_id="controller1", axes=[0.5, -0.5], buttons=[1, 0, 1])

    joy2 = Joy(ts=1000.0, frame_id="controller1", axes=[0.5, -0.5], buttons=[1, 0, 1])

    joy3 = Joy(
        ts=1000.0,
        frame_id="controller2",  # Different frame_id
        axes=[0.5, -0.5],
        buttons=[1, 0, 1],
    )

    joy4 = Joy(
        ts=1000.0,
        frame_id="controller1",
        axes=[0.6, -0.5],  # Different axes
        buttons=[1, 0, 1],
    )

    # Same content should be equal
    assert joy1 == joy2

    # Different frame_id should not be equal
    assert joy1 != joy3

    # Different axes should not be equal
    assert joy1 != joy4

    # Different type should not be equal
    assert joy1 != "not a joy"
    assert joy1 != 42

    print("✓ Joy equality test passed")


def test_string_representation() -> None:
    """Test Joy string representations."""
    print("Testing Joy string representations...")

    joy = Joy(
        ts=1234567890.123,
        frame_id="test_controller",
        axes=[0.1, -0.2, 0.3, 0.4],
        buttons=[1, 0, 1, 0, 0, 1],
    )

    # Test __str__
    str_repr = str(joy)
    assert "Joy" in str_repr
    assert "axes=4 values" in str_repr
    assert "buttons=6 values" in str_repr
    assert "test_controller" in str_repr

    # Test __repr__
    repr_str = repr(joy)
    assert "Joy" in repr_str
    assert "1234567890.123" in repr_str
    assert "test_controller" in repr_str
    assert "[0.1, -0.2, 0.3, 0.4]" in repr_str
    assert "[1, 0, 1, 0, 0, 1]" in repr_str

    print("✓ Joy string representation test passed")


@pytest.mark.ros
def test_ros_conversion() -> None:
    """Test conversion to/from ROS Joy messages."""
    print("Testing Joy ROS conversion...")

    # Create a ROS Joy message
    ros_msg = ROSJoy()
    ros_msg.header = ROSHeader()
    ros_msg.header.stamp.sec = 1234567890
    ros_msg.header.stamp.nanosec = 123456789
    ros_msg.header.frame_id = "ros_gamepad"
    ros_msg.axes = [0.25, -0.75, 0.0, 1.0, -1.0]
    ros_msg.buttons = [1, 1, 0, 0, 1, 0, 1, 0]

    # Convert from ROS
    joy = Joy.from_ros_msg(ros_msg)
    assert abs(joy.ts - 1234567890.123456789) < 1e-9
    assert joy.frame_id == "ros_gamepad"
    assert joy.axes == [0.25, -0.75, 0.0, 1.0, -1.0]
    assert joy.buttons == [1, 1, 0, 0, 1, 0, 1, 0]

    # Convert back to ROS
    ros_msg2 = joy.to_ros_msg()
    assert ros_msg2.header.frame_id == "ros_gamepad"
    assert ros_msg2.header.stamp.sec == 1234567890
    assert abs(ros_msg2.header.stamp.nanosec - 123456789) < 100  # Allow small rounding
    assert list(ros_msg2.axes) == [0.25, -0.75, 0.0, 1.0, -1.0]
    assert list(ros_msg2.buttons) == [1, 1, 0, 0, 1, 0, 1, 0]

    print("✓ Joy ROS conversion test passed")


def test_edge_cases() -> None:
    """Test Joy with edge cases."""
    print("Testing Joy edge cases...")

    # Empty axes and buttons
    joy1 = Joy(axes=[], buttons=[])
    assert joy1.axes == []
    assert joy1.buttons == []
    encoded = joy1.lcm_encode()
    decoded = Joy.lcm_decode(encoded)
    assert decoded.axes == []
    assert decoded.buttons == []

    # Large number of axes and buttons
    many_axes = [float(i) / 100.0 for i in range(20)]
    many_buttons = [i % 2 for i in range(32)]
    joy2 = Joy(axes=many_axes, buttons=many_buttons)
    assert len(joy2.axes) == 20
    assert len(joy2.buttons) == 32
    encoded = joy2.lcm_encode()
    decoded = Joy.lcm_decode(encoded)
    # Check axes with floating point tolerance
    assert len(decoded.axes) == len(many_axes)
    for i, (a, b) in enumerate(zip(decoded.axes, many_axes, strict=False)):
        assert abs(a - b) < 1e-6, f"Axis {i}: {a} != {b}"
    assert decoded.buttons == many_buttons

    # Extreme axis values
    extreme_axes = [-1.0, 1.0, 0.0, -0.999999, 0.999999]
    joy3 = Joy(axes=extreme_axes)
    assert joy3.axes == extreme_axes

    print("✓ Joy edge cases test passed")

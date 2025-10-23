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

import pickle
import time

import pytest

try:
    from geometry_msgs.msg import TwistStamped as ROSTwistStamped
except ImportError:
    ROSTwistStamped = None

from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped


def test_lcm_encode_decode() -> None:
    """Test encoding and decoding of TwistStamped to/from binary LCM format."""
    twist_source = TwistStamped(
        ts=time.time(),
        linear=(1.0, 2.0, 3.0),
        angular=(0.1, 0.2, 0.3),
    )
    binary_msg = twist_source.lcm_encode()
    twist_dest = TwistStamped.lcm_decode(binary_msg)

    assert isinstance(twist_dest, TwistStamped)
    assert twist_dest is not twist_source

    print(twist_source.linear)
    print(twist_source.angular)

    print(twist_dest.linear)
    print(twist_dest.angular)
    assert twist_dest == twist_source


def test_pickle_encode_decode() -> None:
    """Test encoding and decoding of TwistStamped to/from binary pickle format."""

    twist_source = TwistStamped(
        ts=time.time(),
        linear=(1.0, 2.0, 3.0),
        angular=(0.1, 0.2, 0.3),
    )
    binary_msg = pickle.dumps(twist_source)
    twist_dest = pickle.loads(binary_msg)
    assert isinstance(twist_dest, TwistStamped)
    assert twist_dest is not twist_source
    assert twist_dest == twist_source


@pytest.mark.ros
def test_twist_stamped_from_ros_msg() -> None:
    """Test creating a TwistStamped from a ROS TwistStamped message."""
    ros_msg = ROSTwistStamped()
    ros_msg.header.frame_id = "world"
    ros_msg.header.stamp.sec = 123
    ros_msg.header.stamp.nanosec = 456000000
    ros_msg.twist.linear.x = 1.0
    ros_msg.twist.linear.y = 2.0
    ros_msg.twist.linear.z = 3.0
    ros_msg.twist.angular.x = 0.1
    ros_msg.twist.angular.y = 0.2
    ros_msg.twist.angular.z = 0.3

    twist_stamped = TwistStamped.from_ros_msg(ros_msg)

    assert twist_stamped.frame_id == "world"
    assert twist_stamped.ts == 123.456
    assert twist_stamped.linear.x == 1.0
    assert twist_stamped.linear.y == 2.0
    assert twist_stamped.linear.z == 3.0
    assert twist_stamped.angular.x == 0.1
    assert twist_stamped.angular.y == 0.2
    assert twist_stamped.angular.z == 0.3


@pytest.mark.ros
def test_twist_stamped_to_ros_msg() -> None:
    """Test converting a TwistStamped to a ROS TwistStamped message."""
    twist_stamped = TwistStamped(
        ts=123.456,
        frame_id="base_link",
        linear=(1.0, 2.0, 3.0),
        angular=(0.1, 0.2, 0.3),
    )

    ros_msg = twist_stamped.to_ros_msg()

    assert isinstance(ros_msg, ROSTwistStamped)
    assert ros_msg.header.frame_id == "base_link"
    assert ros_msg.header.stamp.sec == 123
    assert ros_msg.header.stamp.nanosec == 456000000
    assert ros_msg.twist.linear.x == 1.0
    assert ros_msg.twist.linear.y == 2.0
    assert ros_msg.twist.linear.z == 3.0
    assert ros_msg.twist.angular.x == 0.1
    assert ros_msg.twist.angular.y == 0.2
    assert ros_msg.twist.angular.z == 0.3


@pytest.mark.ros
def test_twist_stamped_ros_roundtrip() -> None:
    """Test round-trip conversion between TwistStamped and ROS TwistStamped."""
    original = TwistStamped(
        ts=123.789,
        frame_id="odom",
        linear=(1.5, 2.5, 3.5),
        angular=(0.15, 0.25, 0.35),
    )

    ros_msg = original.to_ros_msg()
    restored = TwistStamped.from_ros_msg(ros_msg)

    assert restored.frame_id == original.frame_id
    assert restored.ts == original.ts
    assert restored.linear.x == original.linear.x
    assert restored.linear.y == original.linear.y
    assert restored.linear.z == original.linear.z
    assert restored.angular.x == original.angular.x
    assert restored.angular.y == original.angular.y
    assert restored.angular.z == original.angular.z


if __name__ == "__main__":
    print("Running test_lcm_encode_decode...")
    test_lcm_encode_decode()
    print("✓ test_lcm_encode_decode passed")

    print("Running test_pickle_encode_decode...")
    test_pickle_encode_decode()
    print("✓ test_pickle_encode_decode passed")

    print("Running test_twist_stamped_from_ros_msg...")
    test_twist_stamped_from_ros_msg()
    print("✓ test_twist_stamped_from_ros_msg passed")

    print("Running test_twist_stamped_to_ros_msg...")
    test_twist_stamped_to_ros_msg()
    print("✓ test_twist_stamped_to_ros_msg passed")

    print("Running test_twist_stamped_ros_roundtrip...")
    test_twist_stamped_ros_roundtrip()
    print("✓ test_twist_stamped_ros_roundtrip passed")

    print("\nAll tests passed!")

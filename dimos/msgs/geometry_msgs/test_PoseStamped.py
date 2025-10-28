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
    from geometry_msgs.msg import PoseStamped as ROSPoseStamped
except ImportError:
    ROSPoseStamped = None

from dimos.msgs.geometry_msgs import PoseStamped


def test_lcm_encode_decode() -> None:
    """Test encoding and decoding of Pose to/from binary LCM format."""

    pose_source = PoseStamped(
        ts=time.time(),
        position=(1.0, 2.0, 3.0),
        orientation=(0.1, 0.2, 0.3, 0.9),
    )
    binary_msg = pose_source.lcm_encode()
    pose_dest = PoseStamped.lcm_decode(binary_msg)

    assert isinstance(pose_dest, PoseStamped)
    assert pose_dest is not pose_source

    print(pose_source.position)
    print(pose_source.orientation)

    print(pose_dest.position)
    print(pose_dest.orientation)
    assert pose_dest == pose_source


def test_pickle_encode_decode() -> None:
    """Test encoding and decoding of PoseStamped to/from binary LCM format."""

    pose_source = PoseStamped(
        ts=time.time(),
        position=(1.0, 2.0, 3.0),
        orientation=(0.1, 0.2, 0.3, 0.9),
    )
    binary_msg = pickle.dumps(pose_source)
    pose_dest = pickle.loads(binary_msg)
    assert isinstance(pose_dest, PoseStamped)
    assert pose_dest is not pose_source
    assert pose_dest == pose_source


@pytest.mark.ros
def test_pose_stamped_from_ros_msg() -> None:
    """Test creating a PoseStamped from a ROS PoseStamped message."""
    ros_msg = ROSPoseStamped()
    ros_msg.header.frame_id = "world"
    ros_msg.header.stamp.sec = 123
    ros_msg.header.stamp.nanosec = 456000000
    ros_msg.pose.position.x = 1.0
    ros_msg.pose.position.y = 2.0
    ros_msg.pose.position.z = 3.0
    ros_msg.pose.orientation.x = 0.1
    ros_msg.pose.orientation.y = 0.2
    ros_msg.pose.orientation.z = 0.3
    ros_msg.pose.orientation.w = 0.9

    pose_stamped = PoseStamped.from_ros_msg(ros_msg)

    assert pose_stamped.frame_id == "world"
    assert pose_stamped.ts == 123.456
    assert pose_stamped.position.x == 1.0
    assert pose_stamped.position.y == 2.0
    assert pose_stamped.position.z == 3.0
    assert pose_stamped.orientation.x == 0.1
    assert pose_stamped.orientation.y == 0.2
    assert pose_stamped.orientation.z == 0.3
    assert pose_stamped.orientation.w == 0.9


@pytest.mark.ros
def test_pose_stamped_to_ros_msg() -> None:
    """Test converting a PoseStamped to a ROS PoseStamped message."""
    pose_stamped = PoseStamped(
        ts=123.456,
        frame_id="base_link",
        position=(1.0, 2.0, 3.0),
        orientation=(0.1, 0.2, 0.3, 0.9),
    )

    ros_msg = pose_stamped.to_ros_msg()

    assert isinstance(ros_msg, ROSPoseStamped)
    assert ros_msg.header.frame_id == "base_link"
    assert ros_msg.header.stamp.sec == 123
    assert ros_msg.header.stamp.nanosec == 456000000
    assert ros_msg.pose.position.x == 1.0
    assert ros_msg.pose.position.y == 2.0
    assert ros_msg.pose.position.z == 3.0
    assert ros_msg.pose.orientation.x == 0.1
    assert ros_msg.pose.orientation.y == 0.2
    assert ros_msg.pose.orientation.z == 0.3
    assert ros_msg.pose.orientation.w == 0.9


@pytest.mark.ros
def test_pose_stamped_ros_roundtrip() -> None:
    """Test round-trip conversion between PoseStamped and ROS PoseStamped."""
    original = PoseStamped(
        ts=123.789,
        frame_id="odom",
        position=(1.5, 2.5, 3.5),
        orientation=(0.15, 0.25, 0.35, 0.85),
    )

    ros_msg = original.to_ros_msg()
    restored = PoseStamped.from_ros_msg(ros_msg)

    assert restored.frame_id == original.frame_id
    assert restored.ts == original.ts
    assert restored.position.x == original.position.x
    assert restored.position.y == original.position.y
    assert restored.position.z == original.position.z
    assert restored.orientation.x == original.orientation.x
    assert restored.orientation.y == original.orientation.y
    assert restored.orientation.z == original.orientation.z
    assert restored.orientation.w == original.orientation.w

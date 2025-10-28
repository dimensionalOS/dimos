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
    from geometry_msgs.msg import PoseStamped as ROSPoseStamped
    from nav_msgs.msg import Path as ROSPath
except ImportError:
    ROSPoseStamped = None
    ROSPath = None

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.nav_msgs.Path import Path


def create_test_pose(x: float, y: float, z: float, frame_id: str = "map") -> PoseStamped:
    """Helper to create a test PoseStamped."""
    return PoseStamped(
        frame_id=frame_id,
        position=[x, y, z],
        orientation=Quaternion(0, 0, 0, 1),  # Identity quaternion
    )


def test_init_empty() -> None:
    """Test creating an empty path."""
    path = Path(frame_id="map")
    assert path.frame_id == "map"
    assert len(path) == 0
    assert not path  # Should be falsy when empty
    assert path.poses == []


def test_init_with_poses() -> None:
    """Test creating a path with initial poses."""
    poses = [create_test_pose(i, i, 0) for i in range(3)]
    path = Path(frame_id="map", poses=poses)
    assert len(path) == 3
    assert bool(path)  # Should be truthy when has poses
    assert path.poses == poses


def test_head() -> None:
    """Test getting the first pose."""
    poses = [create_test_pose(i, i, 0) for i in range(3)]
    path = Path(poses=poses)
    assert path.head() == poses[0]

    # Test empty path
    empty_path = Path()
    assert empty_path.head() is None


def test_last() -> None:
    """Test getting the last pose."""
    poses = [create_test_pose(i, i, 0) for i in range(3)]
    path = Path(poses=poses)
    assert path.last() == poses[-1]

    # Test empty path
    empty_path = Path()
    assert empty_path.last() is None


def test_tail() -> None:
    """Test getting all poses except the first."""
    poses = [create_test_pose(i, i, 0) for i in range(3)]
    path = Path(poses=poses)
    tail = path.tail()
    assert len(tail) == 2
    assert tail.poses == poses[1:]
    assert tail.frame_id == path.frame_id

    # Test single element path
    single_path = Path(poses=[poses[0]])
    assert len(single_path.tail()) == 0

    # Test empty path
    empty_path = Path()
    assert len(empty_path.tail()) == 0


def test_push_immutable() -> None:
    """Test immutable push operation."""
    path = Path(frame_id="map")
    pose1 = create_test_pose(1, 1, 0)
    pose2 = create_test_pose(2, 2, 0)

    # Push should return new path
    path2 = path.push(pose1)
    assert len(path) == 0  # Original unchanged
    assert len(path2) == 1
    assert path2.poses[0] == pose1

    # Chain pushes
    path3 = path2.push(pose2)
    assert len(path2) == 1  # Previous unchanged
    assert len(path3) == 2
    assert path3.poses == [pose1, pose2]


def test_push_mutable() -> None:
    """Test mutable push operation."""
    path = Path(frame_id="map")
    pose1 = create_test_pose(1, 1, 0)
    pose2 = create_test_pose(2, 2, 0)

    # Push should modify in place
    path.push_mut(pose1)
    assert len(path) == 1
    assert path.poses[0] == pose1

    path.push_mut(pose2)
    assert len(path) == 2
    assert path.poses == [pose1, pose2]


def test_indexing() -> None:
    """Test indexing and slicing."""
    poses = [create_test_pose(i, i, 0) for i in range(5)]
    path = Path(poses=poses)

    # Single index
    assert path[0] == poses[0]
    assert path[-1] == poses[-1]

    # Slicing
    assert path[1:3] == poses[1:3]
    assert path[:2] == poses[:2]
    assert path[3:] == poses[3:]


def test_iteration() -> None:
    """Test iterating over poses."""
    poses = [create_test_pose(i, i, 0) for i in range(3)]
    path = Path(poses=poses)

    collected = []
    for pose in path:
        collected.append(pose)
    assert collected == poses


def test_slice_method() -> None:
    """Test slice method."""
    poses = [create_test_pose(i, i, 0) for i in range(5)]
    path = Path(frame_id="map", poses=poses)

    sliced = path.slice(1, 4)
    assert len(sliced) == 3
    assert sliced.poses == poses[1:4]
    assert sliced.frame_id == "map"

    # Test open-ended slice
    sliced2 = path.slice(2)
    assert sliced2.poses == poses[2:]


def test_extend_immutable() -> None:
    """Test immutable extend operation."""
    poses1 = [create_test_pose(i, i, 0) for i in range(2)]
    poses2 = [create_test_pose(i + 2, i + 2, 0) for i in range(2)]

    path1 = Path(frame_id="map", poses=poses1)
    path2 = Path(frame_id="odom", poses=poses2)

    extended = path1.extend(path2)
    assert len(path1) == 2  # Original unchanged
    assert len(extended) == 4
    assert extended.poses == poses1 + poses2
    assert extended.frame_id == "map"  # Keeps first path's frame


def test_extend_mutable() -> None:
    """Test mutable extend operation."""
    poses1 = [create_test_pose(i, i, 0) for i in range(2)]
    poses2 = [create_test_pose(i + 2, i + 2, 0) for i in range(2)]

    path1 = Path(frame_id="map", poses=poses1.copy())  # Use copy to avoid modifying original
    path2 = Path(frame_id="odom", poses=poses2)

    path1.extend_mut(path2)
    assert len(path1) == 4
    # Check poses are the same as concatenation
    for _i, (p1, p2) in enumerate(zip(path1.poses, poses1 + poses2, strict=False)):
        assert p1.x == p2.x
        assert p1.y == p2.y
        assert p1.z == p2.z


def test_reverse() -> None:
    """Test reverse operation."""
    poses = [create_test_pose(i, i, 0) for i in range(3)]
    path = Path(poses=poses)

    reversed_path = path.reverse()
    assert len(path) == 3  # Original unchanged
    assert reversed_path.poses == list(reversed(poses))


def test_clear() -> None:
    """Test clear operation."""
    poses = [create_test_pose(i, i, 0) for i in range(3)]
    path = Path(poses=poses)

    path.clear()
    assert len(path) == 0
    assert path.poses == []


def test_lcm_encode_decode() -> None:
    """Test encoding and decoding of Path to/from binary LCM format."""
    # Create path with poses
    # Use timestamps that can be represented exactly in float64
    path_ts = 1234567890.5
    poses = [
        PoseStamped(
            ts=1234567890.0 + i * 0.1,  # Use simpler timestamps
            frame_id=f"frame_{i}",
            position=[i * 1.5, i * 2.5, i * 3.5],
            orientation=(0.1 * i, 0.2 * i, 0.3 * i, 0.9),
        )
        for i in range(3)
    ]

    path_source = Path(ts=path_ts, frame_id="world", poses=poses)

    # Encode to binary
    binary_msg = path_source.lcm_encode()

    # Decode from binary
    path_dest = Path.lcm_decode(binary_msg)

    assert isinstance(path_dest, Path)
    assert path_dest is not path_source

    # Check header
    assert path_dest.frame_id == path_source.frame_id
    # Path timestamp should be preserved
    assert abs(path_dest.ts - path_source.ts) < 1e-6  # Microsecond precision

    # Check poses
    assert len(path_dest.poses) == len(path_source.poses)

    for orig, decoded in zip(path_source.poses, path_dest.poses, strict=False):
        # Check pose timestamps
        assert abs(decoded.ts - orig.ts) < 1e-6
        # All poses should have the path's frame_id
        assert decoded.frame_id == path_dest.frame_id

        # Check position
        assert decoded.x == orig.x
        assert decoded.y == orig.y
        assert decoded.z == orig.z

        # Check orientation
        assert decoded.orientation.x == orig.orientation.x
        assert decoded.orientation.y == orig.orientation.y
        assert decoded.orientation.z == orig.orientation.z
        assert decoded.orientation.w == orig.orientation.w


def test_lcm_encode_decode_empty() -> None:
    """Test encoding and decoding of empty Path."""
    path_source = Path(frame_id="base_link")

    binary_msg = path_source.lcm_encode()
    path_dest = Path.lcm_decode(binary_msg)

    assert isinstance(path_dest, Path)
    assert path_dest.frame_id == path_source.frame_id
    assert len(path_dest.poses) == 0


def test_str_representation() -> None:
    """Test string representation."""
    path = Path(frame_id="map")
    assert str(path) == "Path(frame_id='map', poses=0)"

    path.push_mut(create_test_pose(1, 1, 0))
    path.push_mut(create_test_pose(2, 2, 0))
    assert str(path) == "Path(frame_id='map', poses=2)"


@pytest.mark.ros
def test_path_from_ros_msg() -> None:
    """Test creating a Path from a ROS Path message."""
    ros_msg = ROSPath()
    ros_msg.header.frame_id = "map"
    ros_msg.header.stamp.sec = 123
    ros_msg.header.stamp.nanosec = 456000000

    # Add some poses
    for i in range(3):
        ros_pose = ROSPoseStamped()
        ros_pose.header.frame_id = "map"
        ros_pose.header.stamp.sec = 123 + i
        ros_pose.header.stamp.nanosec = 0
        ros_pose.pose.position.x = float(i)
        ros_pose.pose.position.y = float(i * 2)
        ros_pose.pose.position.z = float(i * 3)
        ros_pose.pose.orientation.x = 0.0
        ros_pose.pose.orientation.y = 0.0
        ros_pose.pose.orientation.z = 0.0
        ros_pose.pose.orientation.w = 1.0
        ros_msg.poses.append(ros_pose)

    path = Path.from_ros_msg(ros_msg)

    assert path.frame_id == "map"
    assert path.ts == 123.456
    assert len(path.poses) == 3

    for i, pose in enumerate(path.poses):
        assert pose.position.x == float(i)
        assert pose.position.y == float(i * 2)
        assert pose.position.z == float(i * 3)
        assert pose.orientation.w == 1.0


@pytest.mark.ros
def test_path_to_ros_msg() -> None:
    """Test converting a Path to a ROS Path message."""
    poses = [
        PoseStamped(
            ts=124.0 + i, frame_id="odom", position=[i, i * 2, i * 3], orientation=[0, 0, 0, 1]
        )
        for i in range(3)
    ]

    path = Path(ts=123.456, frame_id="odom", poses=poses)

    ros_msg = path.to_ros_msg()

    assert isinstance(ros_msg, ROSPath)
    assert ros_msg.header.frame_id == "odom"
    assert ros_msg.header.stamp.sec == 123
    assert ros_msg.header.stamp.nanosec == 456000000
    assert len(ros_msg.poses) == 3

    for i, ros_pose in enumerate(ros_msg.poses):
        assert ros_pose.pose.position.x == float(i)
        assert ros_pose.pose.position.y == float(i * 2)
        assert ros_pose.pose.position.z == float(i * 3)
        assert ros_pose.pose.orientation.w == 1.0


@pytest.mark.ros
def test_path_ros_roundtrip() -> None:
    """Test round-trip conversion between Path and ROS Path."""
    poses = [
        PoseStamped(
            ts=100.0 + i * 0.1,
            frame_id="world",
            position=[i * 1.5, i * 2.5, i * 3.5],
            orientation=[0.1, 0.2, 0.3, 0.9],
        )
        for i in range(3)
    ]

    original = Path(ts=99.789, frame_id="world", poses=poses)

    ros_msg = original.to_ros_msg()
    restored = Path.from_ros_msg(ros_msg)

    assert restored.frame_id == original.frame_id
    assert restored.ts == original.ts
    assert len(restored.poses) == len(original.poses)

    for orig_pose, rest_pose in zip(original.poses, restored.poses, strict=False):
        assert rest_pose.position.x == orig_pose.position.x
        assert rest_pose.position.y == orig_pose.position.y
        assert rest_pose.position.z == orig_pose.position.z
        assert rest_pose.orientation.x == orig_pose.orientation.x
        assert rest_pose.orientation.y == orig_pose.orientation.y
        assert rest_pose.orientation.z == orig_pose.orientation.z
        assert rest_pose.orientation.w == orig_pose.orientation.w

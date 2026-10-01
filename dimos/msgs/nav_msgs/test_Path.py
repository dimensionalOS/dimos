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


from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos_generated.nav_msgs.msg import Path
from dimos_generated.std_msgs.msg import Header
from rosbags.typesys import Stores, get_typestore

from dimos.msgs.time import time_from_nanoseconds


def create_test_pose(x: float, y: float, z: float, frame_id: str = "map") -> PoseStamped:
    """Helper to create a test PoseStamped."""
    return PoseStamped(
        header=Header(frame_id=frame_id),
        pose=Pose(position=Point(x=x, y=y, z=z), orientation=Quaternion(w=1)),
    )


def test_init_empty() -> None:
    """Test creating an empty path."""
    path = Path(header=Header(frame_id="map"))
    assert path.header.frame_id == "map"
    assert len(path.poses) == 0
    assert not list(path.poses)  # Should be falsy when empty
    assert path.poses == []


def test_init_with_poses() -> None:
    """Test creating a path with initial poses."""
    poses = [create_test_pose(i, i, 0) for i in range(3)]
    path = Path(header=Header(frame_id="map"), poses=poses)
    assert len(path.poses) == 3
    assert bool(list(path.poses))  # Should be truthy when has poses
    assert path.poses == poses


def test_head() -> None:
    """Test getting the first pose."""
    poses = [create_test_pose(i, i, 0) for i in range(3)]
    path = Path(poses=poses)
    assert (path.poses[0] if path.poses else None) == poses[0]

    # Test empty path
    empty_path = Path()
    assert (empty_path.poses[0] if empty_path.poses else None) is None


def test_last() -> None:
    """Test getting the last pose."""
    poses = [create_test_pose(i, i, 0) for i in range(3)]
    path = Path(poses=poses)
    assert (path.poses[-1] if path.poses else None) == poses[-1]

    # Test empty path
    empty_path = Path()
    assert (empty_path.poses[-1] if empty_path.poses else None) is None


def test_tail() -> None:
    """Test getting all poses except the first."""
    poses = [create_test_pose(i, i, 0) for i in range(3)]
    path = Path(poses=poses)
    tail = Path(header=path.header, poses=list(path.poses)[1:])
    assert len(tail.poses) == 2
    assert tail.poses == poses[1:]
    assert tail.header.frame_id == path.header.frame_id

    # Test single element path
    single_path = Path(poses=[poses[0]])
    assert len(list(single_path.poses)[1:]) == 0

    # Test empty path
    empty_path = Path()
    assert len(list(empty_path.poses)[1:]) == 0


def test_push_immutable() -> None:
    """Test immutable push operation."""
    path = Path(header=Header(frame_id="map"))
    pose1 = create_test_pose(1, 1, 0)
    pose2 = create_test_pose(2, 2, 0)

    # Push should return new path
    path2 = Path(header=path.header, poses=[*path.poses, pose1])
    assert len(path.poses) == 0  # Original unchanged
    assert len(path2.poses) == 1
    assert path2.poses[0] == pose1

    # Chain pushes
    path3 = Path(header=path2.header, poses=[*path2.poses, pose2])
    assert len(path2.poses) == 1  # Previous unchanged
    assert len(path3.poses) == 2
    assert path3.poses == [pose1, pose2]


def test_push_mutable() -> None:
    """Test mutable push operation."""
    path = Path(header=Header(frame_id="map"))
    pose1 = create_test_pose(1, 1, 0)
    pose2 = create_test_pose(2, 2, 0)

    # Push should modify in place
    path.poses.append(pose1)
    assert len(path.poses) == 1
    assert path.poses[0] == pose1

    path.poses.append(pose2)
    assert len(path.poses) == 2
    assert path.poses == [pose1, pose2]


def test_indexing() -> None:
    """Test indexing and slicing."""
    poses = [create_test_pose(i, i, 0) for i in range(5)]
    path = Path(poses=poses)

    # Single index
    assert next(iter(path.poses)) == poses[0]
    assert list(path.poses)[-1] == poses[-1]

    # Slicing
    assert list(path.poses)[1:3] == poses[1:3]
    assert list(path.poses)[:2] == poses[:2]
    assert list(path.poses)[3:] == poses[3:]


def test_iteration() -> None:
    """Test iterating over poses."""
    poses = [create_test_pose(i, i, 0) for i in range(3)]
    path = Path(poses=poses)

    collected = []
    for pose in path.poses:
        collected.append(pose)
    assert collected == poses


def test_slice_method() -> None:
    """Test slice method."""
    poses = [create_test_pose(i, i, 0) for i in range(5)]
    path = Path(header=Header(frame_id="map"), poses=poses)

    sliced = Path(header=path.header, poses=list(path.poses)[1:4])
    assert len(sliced.poses) == 3
    assert sliced.poses == poses[1:4]
    assert sliced.header.frame_id == "map"

    # Test open-ended slice
    sliced2 = Path(header=path.header, poses=list(path.poses)[2:])
    assert sliced2.poses == poses[2:]


def test_extend_immutable() -> None:
    """Test immutable extend operation."""
    poses1 = [create_test_pose(i, i, 0) for i in range(2)]
    poses2 = [create_test_pose(i + 2, i + 2, 0) for i in range(2)]

    path1 = Path(header=Header(frame_id="map"), poses=poses1)
    path2 = Path(header=Header(frame_id="odom"), poses=poses2)

    extended = Path(header=path1.header, poses=[*path1.poses, *path2.poses])
    assert len(path1.poses) == 2  # Original unchanged
    assert len(extended.poses) == 4
    assert extended.poses == poses1 + poses2
    assert extended.header.frame_id == "map"  # Keeps first path's frame


def test_extend_mutable() -> None:
    """Test mutable extend operation."""
    poses1 = [create_test_pose(i, i, 0) for i in range(2)]
    poses2 = [create_test_pose(i + 2, i + 2, 0) for i in range(2)]

    path1 = Path(
        header=Header(frame_id="map"), poses=poses1.copy()
    )  # Use copy to avoid modifying original
    path2 = Path(header=Header(frame_id="odom"), poses=poses2)

    path1.poses.extend(path2.poses)
    assert len(path1.poses) == 4
    # Check poses are the same as concatenation
    for _i, (p1, p2) in enumerate(zip(path1.poses, poses1 + poses2, strict=False)):
        assert p1.pose.position.x == p2.pose.position.x
        assert p1.pose.position.y == p2.pose.position.y
        assert p1.pose.position.z == p2.pose.position.z


def test_reverse() -> None:
    """Test reverse operation."""
    poses = [create_test_pose(i, i, 0) for i in range(3)]
    path = Path(poses=poses)

    reversed_path = Path(header=path.header, poses=list(reversed(list(path.poses))))
    assert len(path.poses) == 3  # Original unchanged
    assert reversed_path.poses == list(reversed(poses))


def test_clear() -> None:
    """Test clear operation."""
    poses = [create_test_pose(i, i, 0) for i in range(3)]
    path = Path(poses=poses)

    path.poses.clear()
    assert len(path.poses) == 0
    assert path.poses == []


def test_cdr_roundtrip_preserves_per_pose_frames_and_stamps() -> None:
    poses = [
        PoseStamped(
            header=Header(
                stamp=time_from_nanoseconds(1234567890000000000 + i * 100000000),
                frame_id=f"frame_{i}",
            ),
            pose=Pose(
                position=Point(x=i * 1.5, y=i * 2.5, z=i * 3.5),
                orientation=Quaternion(x=0.1 * i, y=0.2 * i, z=0.3 * i, w=0.9),
            ),
        )
        for i in range(3)
    ]
    source = Path(
        header=Header(stamp=time_from_nanoseconds(1234567890500000000), frame_id="world"),
        poses=poses,
    )
    decoded = Path.decode(source.encode())
    assert decoded is not source
    assert decoded.header == source.header
    assert list(decoded.poses) == poses
    independent = get_typestore(Stores.ROS2_JAZZY).deserialize_cdr(source.encode(), Path.msg_name)
    assert independent.header.frame_id == "world"
    assert independent.header.stamp.sec == 1234567890
    assert independent.header.stamp.nanosec == 500000000
    for i, value in enumerate(independent.poses):
        assert value.header.frame_id == f"frame_{i}"
        assert value.header.stamp.sec == 1234567890
        assert value.header.stamp.nanosec == i * 100000000
        assert (value.pose.position.x, value.pose.position.y, value.pose.position.z) == (
            i * 1.5,
            i * 2.5,
            i * 3.5,
        )
        assert (
            value.pose.orientation.x,
            value.pose.orientation.y,
            value.pose.orientation.z,
            value.pose.orientation.w,
        ) == (0.1 * i, 0.2 * i, 0.3 * i, 0.9)


def test_cdr_empty_path() -> None:
    source = Path(header=Header(frame_id="base_link"))
    decoded = Path.decode(source.encode())
    assert decoded.header.frame_id == "base_link"
    assert len(decoded.poses) == 0

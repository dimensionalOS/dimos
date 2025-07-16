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
import numpy as np
import math
from dimos.types.pose import Pose, to_pose
from dimos.types.vector import Vector


def test_pose_default_init():
    """Test that default initialization of Pose() has zero vectors for pos and rot."""
    pose = Pose()

    # Check that pos is a zero vector
    assert isinstance(pose.pos, Vector)
    assert pose.pos.is_zero()
    assert pose.pos.x == 0.0
    assert pose.pos.y == 0.0
    assert pose.pos.z == 0.0

    # Check that rot is a zero vector
    assert isinstance(pose.rot, Vector)
    assert pose.rot.is_zero()
    assert pose.rot.x == 0.0
    assert pose.rot.y == 0.0
    assert pose.rot.z == 0.0

    assert pose.is_zero()

    assert not pose


def test_pose_vector_init():
    """Test initialization with custom vectors."""
    pos = Vector(1.0, 2.0, 3.0)
    rot = Vector(4.0, 5.0, 6.0)

    pose = Pose(pos, rot)

    # Check pos vector
    assert pose.pos == pos
    assert pose.pos.x == 1.0
    assert pose.pos.y == 2.0
    assert pose.pos.z == 3.0

    # Check rot vector
    assert pose.rot == rot
    assert pose.rot.x == 4.0
    assert pose.rot.y == 5.0
    assert pose.rot.z == 6.0

    # even if pos has the same xyz as pos vector
    # it shouldn't accept equality comparisons
    # as both are not the same type
    assert not pose == pos


def test_pose_partial_init():
    """Test initialization with only one custom vector."""
    pos = Vector(1.0, 2.0, 3.0)
    assert pos

    # Only specify pos
    pose1 = Pose(pos)
    assert pose1.pos == pos
    assert pose1.pos.x == 1.0
    assert pose1.pos.y == 2.0
    assert pose1.pos.z == 3.0
    assert not pose1.pos.is_zero()

    assert isinstance(pose1.rot, Vector)
    assert pose1.rot.is_zero()
    assert pose1.rot.x == 0.0
    assert pose1.rot.y == 0.0
    assert pose1.rot.z == 0.0


def test_pose_equality():
    """Test equality comparison between positions."""
    pos1 = Pose(Vector(1.0, 2.0, 3.0), Vector(4.0, 5.0, 6.0))
    pos2 = Pose(Vector(1.0, 2.0, 3.0), Vector(4.0, 5.0, 6.0))
    pos3 = Pose(Vector(1.0, 2.0, 3.0), Vector(7.0, 8.0, 9.0))
    pos4 = Pose(Vector(7.0, 8.0, 9.0), Vector(4.0, 5.0, 6.0))

    # Same pos and rot values should be equal
    assert pos1 == pos2

    # Different rot values should not be equal
    assert pos1 != pos3

    # Different pos values should not be equal
    assert pos1 != pos4

    # Pose should not equal a vector even if values match
    assert pos1 != Vector(1.0, 2.0, 3.0)


def test_pose_vector_operations():
    """Test that Pose inherits Vector operations."""
    pos1 = Pose(Vector(1.0, 2.0, 3.0), Vector(4.0, 5.0, 6.0))
    pos2 = Pose(Vector(2.0, 3.0, 4.0), Vector(7.0, 8.0, 9.0))

    # Addition should work on both position and rotation components
    sum_pos = pos1 + pos2
    assert isinstance(sum_pos, Pose)
    assert sum_pos.x == 3.0
    assert sum_pos.y == 5.0
    assert sum_pos.z == 7.0
    # Rotation should be added as well
    assert sum_pos.rot.x == 11.0  # 4.0 + 7.0
    assert sum_pos.rot.y == 13.0  # 5.0 + 8.0
    assert sum_pos.rot.z == 15.0  # 6.0 + 9.0

    # Subtraction should work on both position and rotation components
    diff_pos = pos2 - pos1
    assert isinstance(diff_pos, Pose)
    assert diff_pos.x == 1.0
    assert diff_pos.y == 1.0
    assert diff_pos.z == 1.0
    # Rotation should be subtracted as well
    assert diff_pos.rot.x == 3.0  # 7.0 - 4.0
    assert diff_pos.rot.y == 3.0  # 8.0 - 5.0
    assert diff_pos.rot.z == 3.0  # 9.0 - 6.0

    # Scalar multiplication
    scaled_pos = pos1 * 2.0
    assert isinstance(scaled_pos, Pose)
    assert scaled_pos.x == 2.0
    assert scaled_pos.y == 4.0
    assert scaled_pos.z == 6.0
    assert scaled_pos.rot == pos1.rot  # Rotation not affected by scalar multiplication

    # Adding a Vector to a Pose (only affects position component)
    vec = Vector(5.0, 6.0, 7.0)
    pos_plus_vec = pos1 + vec
    assert isinstance(pos_plus_vec, Pose)
    assert pos_plus_vec.x == 6.0
    assert pos_plus_vec.y == 8.0
    assert pos_plus_vec.z == 10.0
    assert pos_plus_vec.rot == pos1.rot  # Rotation unchanged


def test_pose_serialization():
    """Test pose serialization."""
    pos = Pose(Vector(1.0, 2.0, 3.0), Vector(4.0, 5.0, 6.0))
    serialized = pos.serialize()

    assert serialized["type"] == "pose"
    assert serialized["pos"] == [1.0, 2.0, 3.0]
    assert serialized["rot"] == [4.0, 5.0, 6.0]


def test_pose_initialization_with_arrays():
    """Test initialization with numpy arrays, lists and tuples."""
    # Test with numpy arrays
    np_pos = np.array([1.0, 2.0, 3.0])
    np_rot = np.array([4.0, 5.0, 6.0])

    pos1 = Pose(np_pos, np_rot)

    assert pos1.x == 1.0
    assert pos1.y == 2.0
    assert pos1.z == 3.0
    assert pos1.rot.x == 4.0
    assert pos1.rot.y == 5.0
    assert pos1.rot.z == 6.0

    # Test with lists
    list_pos = [7.0, 8.0, 9.0]
    list_rot = [10.0, 11.0, 12.0]
    pos2 = Pose(list_pos, list_rot)

    assert pos2.x == 7.0
    assert pos2.y == 8.0
    assert pos2.z == 9.0
    assert pos2.rot.x == 10.0
    assert pos2.rot.y == 11.0
    assert pos2.rot.z == 12.0

    # Test with tuples
    tuple_pos = (13.0, 14.0, 15.0)
    tuple_rot = (16.0, 17.0, 18.0)
    pos3 = Pose(tuple_pos, tuple_rot)

    assert pos3.x == 13.0
    assert pos3.y == 14.0
    assert pos3.z == 15.0
    assert pos3.rot.x == 16.0
    assert pos3.rot.y == 17.0
    assert pos3.rot.z == 18.0


def test_to_pose_with_pose():
    """Test to_pose with Pose input."""
    # Create a pose
    original_pos = Pose(Vector(1.0, 2.0, 3.0), Vector(4.0, 5.0, 6.0))

    # Convert using to_pose
    converted_pos = to_pose(original_pos)

    # Should return the exact same object
    assert converted_pos is original_pos
    assert converted_pos == original_pos

    # Check values
    assert converted_pos.x == 1.0
    assert converted_pos.y == 2.0
    assert converted_pos.z == 3.0
    assert converted_pos.rot.x == 4.0
    assert converted_pos.rot.y == 5.0
    assert converted_pos.rot.z == 6.0


def test_to_pose_with_vector():
    """Test to_pose with Vector input."""
    # Create a vector
    vec = Vector(1.0, 2.0, 3.0)

    # Convert using to_pose
    pos = to_pose(vec)

    # Should return a Pose with the vector as position and zero rotation
    assert isinstance(pos, Pose)
    assert pos.pos == vec
    assert pos.x == 1.0
    assert pos.y == 2.0
    assert pos.z == 3.0

    # Rotation should be zero
    assert isinstance(pos.rot, Vector)
    assert pos.rot.is_zero()
    assert pos.rot.x == 0.0
    assert pos.rot.y == 0.0
    assert pos.rot.z == 0.0


def test_to_pose_with_vectorlike():
    """Test to_pose with VectorLike inputs (arrays, lists, tuples)."""
    # Test with numpy arrays
    np_arr = np.array([1.0, 2.0, 3.0])
    pos1 = to_pose(np_arr)

    assert isinstance(pos1, Pose)
    assert pos1.x == 1.0
    assert pos1.y == 2.0
    assert pos1.z == 3.0
    assert pos1.rot.is_zero()

    # Test with lists
    list_val = [4.0, 5.0, 6.0]
    pos2 = to_pose(list_val)

    assert isinstance(pos2, Pose)
    assert pos2.x == 4.0
    assert pos2.y == 5.0
    assert pos2.z == 6.0
    assert pos2.rot.is_zero()

    # Test with tuples
    tuple_val = (7.0, 8.0, 9.0)
    pos3 = to_pose(tuple_val)

    assert isinstance(pos3, Pose)
    assert pos3.x == 7.0
    assert pos3.y == 8.0
    assert pos3.z == 9.0
    assert pos3.rot.is_zero()


def test_to_pose_with_sequence():
    """Test to_pose with Sequence of VectorLike inputs."""
    # Test with sequence of two vectors
    pos_vec = Vector(1.0, 2.0, 3.0)
    rot_vec = Vector(4.0, 5.0, 6.0)
    pos1 = to_pose([pos_vec, rot_vec])

    assert isinstance(pos1, Pose)
    assert pos1.pos == pos_vec
    assert pos1.rot == rot_vec
    assert pos1.x == 1.0
    assert pos1.y == 2.0
    assert pos1.z == 3.0
    assert pos1.rot.x == 4.0
    assert pos1.rot.y == 5.0
    assert pos1.rot.z == 6.0

    # Test with sequence of lists
    pos2 = to_pose([[7.0, 8.0, 9.0], [10.0, 11.0, 12.0]])

    assert isinstance(pos2, Pose)
    assert pos2.x == 7.0
    assert pos2.y == 8.0
    assert pos2.z == 9.0
    assert pos2.rot.x == 10.0
    assert pos2.rot.y == 11.0
    assert pos2.rot.z == 12.0

    # Test with mixed sequence (tuple and numpy array)
    pos3 = to_pose([(13.0, 14.0, 15.0), np.array([16.0, 17.0, 18.0])])

    assert isinstance(pos3, Pose)
    assert pos3.x == 13.0
    assert pos3.y == 14.0
    assert pos3.z == 15.0
    assert pos3.rot.x == 16.0
    assert pos3.rot.y == 17.0
    assert pos3.rot.z == 18.0


def test_vector_transform():
    robot_pose = Pose(Vector(4.0, 2.0, 0.5), Vector(0.0, 0.0, math.pi / 2))
    target = Vector(1.0, 3.0, 0.0)
    print(robot_pose.vector_to(target))

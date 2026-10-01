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

import pickle

from dimos_generated.geometry_msgs.msg import Point, Pose, Quaternion, Vector3
import numpy as np
import pytest
from rosbags.typesys import Stores, get_typestore

from dimos.msgs.geometry import (
    compose_poses,
    point_from_array,
    quaternion_euler,
    quaternion_from_array,
    quaternion_from_euler,
    vector_array,
    vector_from_array,
)


def test_pose_default_init() -> None:
    """Test that default initialization creates a pose at origin with identity orientation."""
    pose = Pose()
    assert pose.position.x == 0.0
    assert pose.position.y == 0.0
    assert pose.position.z == 0.0
    assert pose.orientation.x == 0.0
    assert pose.orientation.y == 0.0
    assert pose.orientation.z == 0.0
    assert pose.orientation.w == 1.0
    assert pose.position.x == 0.0
    assert pose.position.y == 0.0
    assert pose.position.z == 0.0


def test_pose_pose_init() -> None:
    """Test initialization with position coordinates only (identity orientation)."""
    pose_data = Pose(position=Point(x=1.0, y=2.0, z=3.0))
    pose = pose_data
    assert pose.position.x == 1.0
    assert pose.position.y == 2.0
    assert pose.position.z == 3.0
    assert pose.orientation.x == 0.0
    assert pose.orientation.y == 0.0
    assert pose.orientation.z == 0.0
    assert pose.orientation.w == 1.0
    assert pose.position.x == 1.0
    assert pose.position.y == 2.0
    assert pose.position.z == 3.0


def test_pose_position_init() -> None:
    """Test initialization with position coordinates only (identity orientation)."""
    pose = Pose(position=Point(x=1.0, y=2.0, z=3.0))
    assert pose.position.x == 1.0
    assert pose.position.y == 2.0
    assert pose.position.z == 3.0
    assert pose.orientation.x == 0.0
    assert pose.orientation.y == 0.0
    assert pose.orientation.z == 0.0
    assert pose.orientation.w == 1.0
    assert pose.position.x == 1.0
    assert pose.position.y == 2.0
    assert pose.position.z == 3.0


def test_pose_full_init() -> None:
    """Test initialization with position and orientation coordinates."""
    pose = Pose(
        position=Point(x=1.0, y=2.0, z=3.0), orientation=Quaternion(x=0.1, y=0.2, z=0.3, w=0.9)
    )
    assert pose.position.x == 1.0
    assert pose.position.y == 2.0
    assert pose.position.z == 3.0
    assert pose.orientation.x == 0.1
    assert pose.orientation.y == 0.2
    assert pose.orientation.z == 0.3
    assert pose.orientation.w == 0.9
    assert pose.position.x == 1.0
    assert pose.position.y == 2.0
    assert pose.position.z == 3.0


def test_pose_vector_position_init() -> None:
    """Test initialization with Vector3 position (identity orientation)."""
    position = Vector3(x=4.0, y=5.0, z=6.0)
    pose = Pose(position=point_from_array(vector_array(position)))
    assert pose.position.x == 4.0
    assert pose.position.y == 5.0
    assert pose.position.z == 6.0
    assert pose.orientation.x == 0.0
    assert pose.orientation.y == 0.0
    assert pose.orientation.z == 0.0
    assert pose.orientation.w == 1.0


def test_pose_vector_quaternion_init() -> None:
    """Test initialization with Vector3 position and Quaternion orientation."""
    position = Vector3(x=1.0, y=2.0, z=3.0)
    orientation = Quaternion(x=0.1, y=0.2, z=0.3, w=0.9)
    pose = Pose(position=point_from_array(vector_array(position)), orientation=orientation)
    assert pose.position.x == 1.0
    assert pose.position.y == 2.0
    assert pose.position.z == 3.0
    assert pose.orientation.x == 0.1
    assert pose.orientation.y == 0.2
    assert pose.orientation.z == 0.3
    assert pose.orientation.w == 0.9


def test_pose_list_init() -> None:
    """Test initialization with lists for position and orientation."""
    position_list = [1.0, 2.0, 3.0]
    orientation_list = [0.1, 0.2, 0.3, 0.9]
    pose = Pose(
        position=point_from_array(position_list),
        orientation=quaternion_from_array(orientation_list),
    )
    assert pose.position.x == 1.0
    assert pose.position.y == 2.0
    assert pose.position.z == 3.0
    assert pose.orientation.x == 0.1
    assert pose.orientation.y == 0.2
    assert pose.orientation.z == 0.3
    assert pose.orientation.w == 0.9


def test_pose_tuple_init() -> None:
    """Test initialization from a tuple of (position, orientation)."""
    position = [1.0, 2.0, 3.0]
    orientation = [0.1, 0.2, 0.3, 0.9]
    pose_tuple = (position, orientation)
    pose = Pose(
        position=point_from_array(pose_tuple[0]), orientation=quaternion_from_array(pose_tuple[1])
    )
    assert pose.position.x == 1.0
    assert pose.position.y == 2.0
    assert pose.position.z == 3.0
    assert pose.orientation.x == 0.1
    assert pose.orientation.y == 0.2
    assert pose.orientation.z == 0.3
    assert pose.orientation.w == 0.9


def test_pose_dict_init() -> None:
    """Test initialization from a dictionary with 'position' and 'orientation' keys."""
    pose_dict = {"position": [1.0, 2.0, 3.0], "orientation": [0.1, 0.2, 0.3, 0.9]}
    pose = Pose(
        position=point_from_array(pose_dict["position"]),
        orientation=quaternion_from_array(pose_dict["orientation"]),
    )
    assert pose.position.x == 1.0
    assert pose.position.y == 2.0
    assert pose.position.z == 3.0
    assert pose.orientation.x == 0.1
    assert pose.orientation.y == 0.2
    assert pose.orientation.z == 0.3
    assert pose.orientation.w == 0.9


def test_pose_copy_init() -> None:
    """Test initialization from another Pose (copy constructor)."""
    original = Pose(
        position=Point(x=1.0, y=2.0, z=3.0), orientation=Quaternion(x=0.1, y=0.2, z=0.3, w=0.9)
    )
    copy = Pose.decode(original.encode())
    assert copy.position.x == 1.0
    assert copy.position.y == 2.0
    assert copy.position.z == 3.0
    assert copy.orientation.x == 0.1
    assert copy.orientation.y == 0.2
    assert copy.orientation.z == 0.3
    assert copy.orientation.w == 0.9
    assert copy is not original
    assert copy == original


def test_pose_mutated_value_copy() -> None:
    """Test initialization from a generated Pose."""
    source_pose = Pose()
    source_pose.position.x = 1.0
    source_pose.position.y = 2.0
    source_pose.position.z = 3.0
    source_pose.orientation.x = 0.1
    source_pose.orientation.y = 0.2
    source_pose.orientation.z = 0.3
    source_pose.orientation.w = 0.9
    pose = Pose.decode(source_pose.encode())
    assert pose.position.x == 1.0
    assert pose.position.y == 2.0
    assert pose.position.z == 3.0
    assert pose.orientation.x == 0.1
    assert pose.orientation.y == 0.2
    assert pose.orientation.z == 0.3
    assert pose.orientation.w == 0.9


def test_pose_properties() -> None:
    """Test pose property access."""
    pose = Pose(
        position=Point(x=1.0, y=2.0, z=3.0), orientation=Quaternion(x=0.1, y=0.2, z=0.3, w=0.9)
    )
    assert pose.position.x == 1.0
    assert pose.position.y == 2.0
    assert pose.position.z == 3.0
    euler = vector_from_array(quaternion_euler(pose.orientation))
    assert quaternion_euler(pose.orientation)[0] == euler.x
    assert quaternion_euler(pose.orientation)[1] == euler.y
    assert quaternion_euler(pose.orientation)[2] == euler.z


def test_pose_euler_properties_identity() -> None:
    """Test pose Euler angle properties with identity orientation."""
    pose = Pose(position=Point(x=1.0, y=2.0, z=3.0))
    assert np.isclose(quaternion_euler(pose.orientation)[0], 0.0, atol=1e-10)
    assert np.isclose(quaternion_euler(pose.orientation)[1], 0.0, atol=1e-10)
    assert np.isclose(quaternion_euler(pose.orientation)[2], 0.0, atol=1e-10)
    assert np.isclose(vector_from_array(quaternion_euler(pose.orientation)).x, 0.0, atol=1e-10)
    assert np.isclose(vector_from_array(quaternion_euler(pose.orientation)).y, 0.0, atol=1e-10)
    assert np.isclose(vector_from_array(quaternion_euler(pose.orientation)).z, 0.0, atol=1e-10)


def test_pose_independent_cdr_fields() -> None:
    source = Pose(
        position=Point(x=1.234, y=2.567, z=3.891),
        orientation=Quaternion(x=0.1, y=0.2, z=0.3, w=0.9),
    )
    decoded = get_typestore(Stores.ROS2_JAZZY).deserialize_cdr(source.encode(), Pose.msg_name)
    assert (decoded.position.x, decoded.position.y, decoded.position.z) == (1.234, 2.567, 3.891)
    assert (
        decoded.orientation.x,
        decoded.orientation.y,
        decoded.orientation.z,
        decoded.orientation.w,
    ) == (0.1, 0.2, 0.3, 0.9)


def test_pose_equality() -> None:
    """Test pose equality comparison."""
    pose1 = Pose(
        position=Point(x=1.0, y=2.0, z=3.0), orientation=Quaternion(x=0.1, y=0.2, z=0.3, w=0.9)
    )
    pose2 = Pose(
        position=Point(x=1.0, y=2.0, z=3.0), orientation=Quaternion(x=0.1, y=0.2, z=0.3, w=0.9)
    )
    pose3 = Pose(
        position=Point(x=1.1, y=2.0, z=3.0), orientation=Quaternion(x=0.1, y=0.2, z=0.3, w=0.9)
    )
    pose4 = Pose(
        position=Point(x=1.0, y=2.0, z=3.0), orientation=Quaternion(x=0.11, y=0.2, z=0.3, w=0.9)
    )
    assert pose1 == pose2
    assert pose2 == pose1
    assert pose1 != pose3
    assert pose1 != pose4
    assert pose3 != pose4
    assert pose1 != "not a pose"
    assert pose1 != [1.0, 2.0, 3.0]
    assert pose1 is not None


def test_pose_with_numpy_arrays() -> None:
    """Test pose initialization with numpy arrays."""
    position_array = np.array([1.0, 2.0, 3.0])
    orientation_array = np.array([0.1, 0.2, 0.3, 0.9])
    pose = Pose(
        position=point_from_array(position_array),
        orientation=quaternion_from_array(orientation_array),
    )
    assert pose.position.x == 1.0
    assert pose.position.y == 2.0
    assert pose.position.z == 3.0
    assert pose.orientation.x == 0.1
    assert pose.orientation.y == 0.2
    assert pose.orientation.z == 0.3
    assert pose.orientation.w == 0.9


def test_pose_with_mixed_types() -> None:
    """Test pose initialization with mixed input types."""
    pose1 = Pose(
        position=point_from_array((1.0, 2.0, 3.0)),
        orientation=quaternion_from_array([0.1, 0.2, 0.3, 0.9]),
    )
    position = np.array([1.0, 2.0, 3.0])
    orientation = Quaternion(x=0.1, y=0.2, z=0.3, w=0.9)
    pose2 = Pose(position=point_from_array(position), orientation=orientation)
    assert pose1.position.x == pose2.position.x
    assert pose1.position.y == pose2.position.y
    assert pose1.position.z == pose2.position.z
    assert pose1.orientation.x == pose2.orientation.x
    assert pose1.orientation.y == pose2.orientation.y
    assert pose1.orientation.z == pose2.orientation.z
    assert pose1.orientation.w == pose2.orientation.w


def test_pose_nested_value_copy() -> None:
    source = Pose(position=Point(x=1, y=2, z=3))
    copied = Pose(position=source.position, orientation=source.orientation)
    copied.position.x = 10
    assert source.position == Point(x=1, y=2, z=3)
    assert copied.position == Point(x=10, y=2, z=3)


def test_to_pose_conversion() -> None:
    """Test to_pose function with convertible inputs."""
    pose_tuple = ([1.0, 2.0, 3.0], [0.1, 0.2, 0.3, 0.9])
    result1 = Pose(
        position=point_from_array(pose_tuple[0]), orientation=quaternion_from_array(pose_tuple[1])
    )
    assert isinstance(result1, Pose)
    assert result1.position.x == 1.0
    assert result1.position.y == 2.0
    assert result1.position.z == 3.0
    assert result1.orientation.x == 0.1
    assert result1.orientation.y == 0.2
    assert result1.orientation.z == 0.3
    assert result1.orientation.w == 0.9
    pose_dict = {"position": [1.0, 2.0, 3.0], "orientation": [0.1, 0.2, 0.3, 0.9]}
    result2 = Pose(
        position=point_from_array(pose_dict["position"]),
        orientation=quaternion_from_array(pose_dict["orientation"]),
    )
    assert isinstance(result2, Pose)
    assert result2.position.x == 1.0
    assert result2.position.y == 2.0
    assert result2.position.z == 3.0
    assert result2.orientation.x == 0.1
    assert result2.orientation.y == 0.2
    assert result2.orientation.z == 0.3
    assert result2.orientation.w == 0.9


def test_pose_euler_roundtrip() -> None:
    """Test conversion from Euler angles to quaternion and back."""
    roll = 0.1
    pitch = 0.2
    yaw = 0.3
    euler_vector = Vector3(x=roll, y=pitch, z=yaw)
    quaternion = quaternion_from_euler(euler_vector.x, euler_vector.y, euler_vector.z)
    pose = Pose(
        position=point_from_array(vector_array(Vector3(x=0, y=0, z=0))), orientation=quaternion
    )
    result_euler = vector_from_array(quaternion_euler(pose.orientation))
    assert np.isclose(result_euler.x, roll, atol=1e-06)
    assert np.isclose(result_euler.y, pitch, atol=1e-06)
    assert np.isclose(result_euler.z, yaw, atol=1e-06)


def test_pose_zero_position() -> None:
    """Test pose with zero position vector."""
    pose = Pose(position=Point(x=0.0, y=0.0, z=0.0))
    assert pose.position.x == 0.0
    assert pose.position.y == 0.0
    assert pose.position.z == 0.0
    assert np.isclose(quaternion_euler(pose.orientation)[0], 0.0, atol=1e-10)
    assert np.isclose(quaternion_euler(pose.orientation)[1], 0.0, atol=1e-10)
    assert np.isclose(quaternion_euler(pose.orientation)[2], 0.0, atol=1e-10)


def test_pose_unit_vectors() -> None:
    """Test pose with unit vector positions."""
    pose_x = Pose(position=point_from_array(vector_array(Vector3(x=1))))
    assert pose_x.position.x == 1.0
    assert pose_x.position.y == 0.0
    assert pose_x.position.z == 0.0
    pose_y = Pose(position=point_from_array(vector_array(Vector3(y=1))))
    assert pose_y.position.x == 0.0
    assert pose_y.position.y == 1.0
    assert pose_y.position.z == 0.0
    pose_z = Pose(position=point_from_array(vector_array(Vector3(z=1))))
    assert pose_z.position.x == 0.0
    assert pose_z.position.y == 0.0
    assert pose_z.position.z == 1.0


def test_pose_negative_coordinates() -> None:
    """Test pose with negative coordinates."""
    pose = Pose(
        position=Point(x=-1.0, y=-2.0, z=-3.0),
        orientation=Quaternion(x=-0.1, y=-0.2, z=-0.3, w=0.9),
    )
    assert pose.position.x == -1.0
    assert pose.position.y == -2.0
    assert pose.position.z == -3.0
    assert pose.orientation.x == -0.1
    assert pose.orientation.y == -0.2
    assert pose.orientation.z == -0.3
    assert pose.orientation.w == 0.9


def test_pose_large_coordinates() -> None:
    """Test pose with large coordinate values."""
    large_value = 1000.0
    pose = Pose(position=Point(x=large_value, y=large_value, z=large_value))
    assert pose.position.x == large_value
    assert pose.position.y == large_value
    assert pose.position.z == large_value
    assert pose.orientation.x == 0.0
    assert pose.orientation.y == 0.0
    assert pose.orientation.z == 0.0
    assert pose.orientation.w == 1.0


@pytest.mark.parametrize(
    "x,y,z",
    [(0.0, 0.0, 0.0), (1.0, 2.0, 3.0), (-1.0, -2.0, -3.0), (0.5, -0.5, 1.5), (100.0, -100.0, 0.0)],
)
def test_pose_parametrized_positions(x, y, z) -> None:
    """Parametrized test for various position values."""
    pose = Pose(position=Point(x=x, y=y, z=z))
    assert pose.position.x == x
    assert pose.position.y == y
    assert pose.position.z == z
    assert pose.orientation.x == 0.0
    assert pose.orientation.y == 0.0
    assert pose.orientation.z == 0.0
    assert pose.orientation.w == 1.0


@pytest.mark.parametrize(
    "qx,qy,qz,qw",
    [
        (0.0, 0.0, 0.0, 1.0),
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.5, 0.5, 0.5, 0.5),
    ],
)
def test_pose_parametrized_orientations(qx, qy, qz, qw) -> None:
    """Parametrized test for various orientation values."""
    pose = Pose(position=Point(x=0.0, y=0.0, z=0.0), orientation=Quaternion(x=qx, y=qy, z=qz, w=qw))
    assert pose.position.x == 0.0
    assert pose.position.y == 0.0
    assert pose.position.z == 0.0
    assert pose.orientation.x == qx
    assert pose.orientation.y == qy
    assert pose.orientation.z == qz
    assert pose.orientation.w == qw


def test_cdr_encode_decode() -> None:
    """Test encoding and decoding of Pose to/from binary CDR format."""

    def encodepass() -> None:
        pose_source = Pose(
            position=Point(x=1.0, y=2.0, z=3.0), orientation=Quaternion(x=0.1, y=0.2, z=0.3, w=0.9)
        )
        binary_msg = pose_source.encode()
        pose_dest = Pose.decode(binary_msg)
        assert isinstance(pose_dest, Pose)
        assert pose_dest is not pose_source
        assert pose_dest == pose_source
        assert isinstance(pose_dest.position, Point)
        assert isinstance(pose_dest.orientation, Quaternion)

    import timeit

    timeit.timeit(encodepass, number=1000)


def test_pickle_encode_decode() -> None:
    """Test encoding and decoding of Pose to/from binary CDR format."""

    def encodepass() -> None:
        pose_source = Pose(
            position=Point(x=1.0, y=2.0, z=3.0), orientation=Quaternion(x=0.1, y=0.2, z=0.3, w=0.9)
        )
        binary_msg = pickle.dumps(pose_source)
        pose_dest = pickle.loads(binary_msg)
        assert isinstance(pose_dest, Pose)
        assert pose_dest is not pose_source
        assert pose_dest == pose_source

    import timeit

    timeit.timeit(encodepass, number=1000)


def test_pose_addition_translation_only() -> None:
    """Test pose addition with translation only (identity rotations)."""
    pose1 = Pose(position=Point(x=1.0, y=2.0, z=3.0))
    pose2 = Pose(position=Point(x=4.0, y=5.0, z=6.0))
    result = compose_poses(pose1, pose2)
    assert result.position.x == 5.0
    assert result.position.y == 7.0
    assert result.position.z == 9.0
    assert result.orientation.x == 0.0
    assert result.orientation.y == 0.0
    assert result.orientation.z == 0.0
    assert result.orientation.w == 1.0


def test_pose_addition_with_rotation() -> None:
    """Test pose addition with rotation applied to translation."""
    angle = np.pi / 2
    pose1 = Pose(
        position=Point(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=np.sin(angle / 2), w=np.cos(angle / 2)),
    )
    pose2 = Pose(position=Point(x=1.0, y=0.0, z=0.0))
    result = compose_poses(pose1, pose2)
    assert np.isclose(result.position.x, 0.0, atol=1e-10)
    assert np.isclose(result.position.y, 1.0, atol=1e-10)
    assert np.isclose(result.position.z, 0.0, atol=1e-10)
    assert np.isclose(result.orientation.x, 0.0, atol=1e-10)
    assert np.isclose(result.orientation.y, 0.0, atol=1e-10)
    assert np.isclose(result.orientation.z, np.sin(angle / 2), atol=1e-10)
    assert np.isclose(result.orientation.w, np.cos(angle / 2), atol=1e-10)


def test_pose_addition_rotation_composition() -> None:
    """Test that rotations are properly composed."""
    angle1 = np.pi / 4
    pose1 = Pose(
        position=Point(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=np.sin(angle1 / 2), w=np.cos(angle1 / 2)),
    )
    angle2 = np.pi / 4
    pose2 = Pose(
        position=Point(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=np.sin(angle2 / 2), w=np.cos(angle2 / 2)),
    )
    result = compose_poses(pose1, pose2)
    expected_angle = angle1 + angle2
    expected_qz = np.sin(expected_angle / 2)
    expected_qw = np.cos(expected_angle / 2)
    assert np.isclose(result.orientation.z, expected_qz, atol=1e-10)
    assert np.isclose(result.orientation.w, expected_qw, atol=1e-10)


def test_pose_addition_full_transform() -> None:
    """Test full pose composition with translation and rotation."""
    robot_yaw = np.pi / 2
    robot_pose = Pose(
        position=Point(x=2.0, y=1.0, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=np.sin(robot_yaw / 2), w=np.cos(robot_yaw / 2)),
    )
    object_in_robot = Pose(position=Point(x=3.0, y=-1.0, z=0.0))
    object_in_world = compose_poses(robot_pose, object_in_robot)
    assert np.isclose(object_in_world.position.x, 3.0, atol=1e-10)
    assert np.isclose(object_in_world.position.y, 4.0, atol=1e-10)
    assert np.isclose(object_in_world.position.z, 0.0, atol=1e-10)
    assert np.isclose(quaternion_euler(object_in_world.orientation)[2], robot_yaw, atol=1e-10)


def test_pose_addition_chain() -> None:
    """Test chaining multiple pose additions."""
    pose1 = Pose(position=Point(x=1.0, y=0.0, z=0.0))
    pose2 = Pose(position=Point(x=0.0, y=1.0, z=0.0))
    pose3 = Pose(position=Point(x=0.0, y=0.0, z=1.0))
    result = compose_poses(compose_poses(pose1, pose2), pose3)
    assert result.position.x == 1.0
    assert result.position.y == 1.0
    assert result.position.z == 1.0


def test_pose_addition_with_convertible() -> None:
    """Test pose addition with convertible types."""
    pose1 = Pose(position=Point(x=1.0, y=2.0, z=3.0))
    pose_tuple = ([4.0, 5.0, 6.0], [0.0, 0.0, 0.0, 1.0])
    result1 = compose_poses(
        pose1,
        Pose(
            position=point_from_array(pose_tuple[0]),
            orientation=quaternion_from_array(pose_tuple[1]),
        ),
    )
    assert result1.position.x == 5.0
    assert result1.position.y == 7.0
    assert result1.position.z == 9.0
    pose_dict = {"position": [1.0, 0.0, 0.0], "orientation": [0.0, 0.0, 0.0, 1.0]}
    result2 = compose_poses(
        pose1,
        Pose(
            position=point_from_array(pose_dict["position"]),
            orientation=quaternion_from_array(pose_dict["orientation"]),
        ),
    )
    assert result2.position.x == 2.0
    assert result2.position.y == 2.0
    assert result2.position.z == 3.0


def test_pose_identity_addition() -> None:
    """Test that adding identity pose leaves pose unchanged."""
    pose = Pose(
        position=Point(x=1.0, y=2.0, z=3.0), orientation=Quaternion(x=0.1, y=0.2, z=0.3, w=0.9)
    )
    identity = Pose()
    result = compose_poses(pose, identity)
    assert result.position.x == pose.position.x
    assert result.position.y == pose.position.y
    assert result.position.z == pose.position.z
    assert result.orientation.x == pose.orientation.x
    assert result.orientation.y == pose.orientation.y
    assert result.orientation.z == pose.orientation.z
    assert result.orientation.w == pose.orientation.w


def test_pose_addition_3d_rotation() -> None:
    """Test pose addition with 3D rotations."""
    roll = np.pi / 4
    pose1 = Pose(
        position=Point(x=1.0, y=0.0, z=0.0),
        orientation=Quaternion(x=np.sin(roll / 2), y=0.0, z=0.0, w=np.cos(roll / 2)),
    )
    pose2 = Pose(position=Point(x=0.0, y=1.0, z=1.0))
    result = compose_poses(pose1, pose2)
    cos45 = np.cos(roll)
    sin45 = np.sin(roll)
    assert np.isclose(result.position.x, 1.0, atol=1e-10)
    assert np.isclose(result.position.y, cos45 - sin45, atol=1e-10)
    assert np.isclose(result.position.z, sin45 + cos45, atol=1e-10)

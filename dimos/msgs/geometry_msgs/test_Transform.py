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
import pytest
from dimos_lcm.geometry_msgs import Transform as LCMTransform

from dimos.msgs.geometry_msgs import Pose, Quaternion, Transform, Vector3


def test_transform_initialization():
    # Test default initialization (identity transform)
    tf = Transform()
    assert tf.translation.x == 0.0
    assert tf.translation.y == 0.0
    assert tf.translation.z == 0.0
    assert tf.rotation.x == 0.0
    assert tf.rotation.y == 0.0
    assert tf.rotation.z == 0.0
    assert tf.rotation.w == 1.0

    # Test initialization with Vector3 and Quaternion
    trans = Vector3(1.0, 2.0, 3.0)
    rot = Quaternion(0.0, 0.0, 0.707107, 0.707107)  # 90 degrees around Z
    tf2 = Transform(trans, rot)
    assert tf2.translation == trans
    assert tf2.rotation == rot

    # Test copy constructor
    tf3 = Transform(tf2)
    assert tf3.translation == tf2.translation
    assert tf3.rotation == tf2.rotation
    assert tf3 == tf2
    # Ensure it's a deep copy
    tf3.translation.x = 10.0
    assert tf2.translation.x == 1.0

    # Test initialization from LCM Transform
    lcm_tf = LCMTransform()
    lcm_tf.translation = Vector3(4.0, 5.0, 6.0)
    lcm_tf.rotation = Quaternion(0.0, 0.707107, 0.0, 0.707107)  # 90 degrees around Y
    tf4 = Transform(lcm_tf)
    assert tf4.translation.x == 4.0
    assert tf4.translation.y == 5.0
    assert tf4.translation.z == 6.0
    assert np.isclose(tf4.rotation.x, 0.0)
    assert np.isclose(tf4.rotation.y, 0.707107)
    assert np.isclose(tf4.rotation.z, 0.0)
    assert np.isclose(tf4.rotation.w, 0.707107)

    # Test initialization with only translation
    tf5 = Transform(Vector3(7.0, 8.0, 9.0))
    assert tf5.translation.x == 7.0
    assert tf5.translation.y == 8.0
    assert tf5.translation.z == 9.0
    assert tf5.rotation.w == 1.0  # Identity rotation

    # Test initialization with only rotation
    tf6 = Transform(Quaternion(0.0, 0.0, 0.0, 1.0))
    assert tf6.translation.is_zero()  # Zero translation
    assert tf6.rotation.w == 1.0

    # Test keyword argument initialization
    tf7 = Transform(translation=Vector3(1, 2, 3), rotation=Quaternion())
    assert tf7.translation == Vector3(1, 2, 3)
    assert tf7.rotation == Quaternion()

    # Test keyword with only translation
    tf8 = Transform(translation=Vector3(4, 5, 6))
    assert tf8.translation == Vector3(4, 5, 6)
    assert tf8.rotation.w == 1.0

    # Test keyword with only rotation
    tf9 = Transform(rotation=Quaternion(0, 0, 1, 0))
    assert tf9.translation.is_zero()
    assert tf9.rotation == Quaternion(0, 0, 1, 0)


def test_transform_identity():
    # Test identity class method
    tf = Transform.identity()
    assert tf.translation.is_zero()
    assert tf.rotation.x == 0.0
    assert tf.rotation.y == 0.0
    assert tf.rotation.z == 0.0
    assert tf.rotation.w == 1.0

    # Identity should equal default constructor
    assert tf == Transform()


def test_transform_equality():
    tf1 = Transform(Vector3(1, 2, 3), Quaternion(0, 0, 0, 1))
    tf2 = Transform(Vector3(1, 2, 3), Quaternion(0, 0, 0, 1))
    tf3 = Transform(Vector3(1, 2, 4), Quaternion(0, 0, 0, 1))  # Different z
    tf4 = Transform(Vector3(1, 2, 3), Quaternion(0, 0, 1, 0))  # Different rotation

    assert tf1 == tf2
    assert tf1 != tf3
    assert tf1 != tf4
    assert tf1 != "not a transform"


def test_transform_string_representations():
    tf = Transform(Vector3(1.5, -2.0, 3.14), Quaternion(0, 0, 0.707107, 0.707107))

    # Test repr
    repr_str = repr(tf)
    assert "Transform" in repr_str
    assert "translation=" in repr_str
    assert "rotation=" in repr_str
    assert "1.5" in repr_str

    # Test str
    str_str = str(tf)
    assert "Transform:" in str_str
    assert "Translation:" in str_str
    assert "Rotation:" in str_str


def test_pose_add_transform():
    initial_pose = Pose(1.0, 0.0, 0.0)

    # 90 degree rotation around Z axis
    angle = np.pi / 2
    transform = Transform(
        translation=Vector3(2.0, 1.0, 0.0),
        rotation=Quaternion(0.0, 0.0, np.sin(angle / 2), np.cos(angle / 2)),
    )

    transformed_pose = initial_pose @ transform

    # - Translation (2, 1, 0) is added directly to position (1, 0, 0)
    # - Result position: (3, 1, 0)
    assert np.isclose(transformed_pose.position.x, 3.0, atol=1e-10)
    assert np.isclose(transformed_pose.position.y, 1.0, atol=1e-10)
    assert np.isclose(transformed_pose.position.z, 0.0, atol=1e-10)

    # Rotation should be 90 degrees around Z
    assert np.isclose(transformed_pose.orientation.x, 0.0, atol=1e-10)
    assert np.isclose(transformed_pose.orientation.y, 0.0, atol=1e-10)
    assert np.isclose(transformed_pose.orientation.z, np.sin(angle / 2), atol=1e-10)
    assert np.isclose(transformed_pose.orientation.w, np.cos(angle / 2), atol=1e-10)

    found_tf = initial_pose.find_transform(transformed_pose)

    assert found_tf.translation == transform.translation
    assert found_tf.rotation == transform.rotation
    assert found_tf.translation.x == transform.translation.x
    assert found_tf.translation.y == transform.translation.y
    assert found_tf.translation.z == transform.translation.z

    assert found_tf.rotation.x == transform.rotation.x
    assert found_tf.rotation.y == transform.rotation.y
    assert found_tf.rotation.z == transform.rotation.z
    assert found_tf.rotation.w == transform.rotation.w

    print(found_tf.rotation, found_tf.translation)


def test_pose_add_transform_with_rotation():
    # Create a pose at (0, 0, 0) rotated 90 degrees around Z
    angle = np.pi / 2
    initial_pose = Pose(0.0, 0.0, 0.0, 0.0, 0.0, np.sin(angle / 2), np.cos(angle / 2))

    # Add 45 degree rotation to transform1
    rotation_angle = np.pi / 4  # 45 degrees
    transform1 = Transform(
        translation=Vector3(1.0, 0.0, 0.0),
        rotation=Quaternion(
            0.0, 0.0, np.sin(rotation_angle / 2), np.cos(rotation_angle / 2)
        ),  # 45° around Z
    )

    transform2 = Transform(
        translation=Vector3(0.0, 1.0, 1.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),  # No rotation
    )

    transformed_pose1 = initial_pose @ transform1
    transformed_pose2 = initial_pose @ transform1 @ transform2

    # Test transformed_pose1: initial_pose + transform1
    # Since the pose is rotated 90° (facing +Y), moving forward (local X)
    # means moving in the +Y direction in world frame
    assert np.isclose(transformed_pose1.position.x, 0.0, atol=1e-10)
    assert np.isclose(transformed_pose1.position.y, 1.0, atol=1e-10)
    assert np.isclose(transformed_pose1.position.z, 0.0, atol=1e-10)

    # Orientation should be 90° + 45° = 135° around Z
    total_angle1 = angle + rotation_angle  # 135 degrees
    assert np.isclose(transformed_pose1.orientation.x, 0.0, atol=1e-10)
    assert np.isclose(transformed_pose1.orientation.y, 0.0, atol=1e-10)
    assert np.isclose(transformed_pose1.orientation.z, np.sin(total_angle1 / 2), atol=1e-10)
    assert np.isclose(transformed_pose1.orientation.w, np.cos(total_angle1 / 2), atol=1e-10)

    # Test transformed_pose2: initial_pose + transform1 + transform2
    # Starting from (0, 0, 0) facing 90°:
    #
    # - Apply transform1: move 1 forward (along +Y) → (0, 1, 0), now facing 135°
    #
    # - Apply transform2: move 1 in local Y and 1 up
    #   At 135°, local Y points at 225° (135° + 90°)
    #
    #   x += cos(225°) = -√2/2, y += sin(225°) = -√2/2
    sqrt2_2 = np.sqrt(2) / 2
    expected_x = 0.0 - sqrt2_2  # 0 - √2/2 ≈ -0.707
    expected_y = 1.0 - sqrt2_2  # 1 - √2/2 ≈ 0.293
    expected_z = 1.0  # 0 + 1

    assert np.isclose(transformed_pose2.position.x, expected_x, atol=1e-10)
    assert np.isclose(transformed_pose2.position.y, expected_y, atol=1e-10)
    assert np.isclose(transformed_pose2.position.z, expected_z, atol=1e-10)

    # Orientation should be 135° (only transform1 has rotation)
    total_angle2 = total_angle1  # 135 degrees (transform2 has no rotation)
    assert np.isclose(transformed_pose2.orientation.x, 0.0, atol=1e-10)
    assert np.isclose(transformed_pose2.orientation.y, 0.0, atol=1e-10)
    assert np.isclose(transformed_pose2.orientation.z, np.sin(total_angle2 / 2), atol=1e-10)
    assert np.isclose(transformed_pose2.orientation.w, np.cos(total_angle2 / 2), atol=1e-10)


def test_pose_add_lcm_transform():
    """Test that Pose works with both LCMTransform and Transform types."""
    initial_pose = Pose(1.0, 0.0, 0.0)

    # Create an LCMTransform
    lcm_transform = LCMTransform()
    lcm_transform.translation = Vector3(2.0, 1.0, 0.0)
    angle = np.pi / 2  # 90 degrees
    lcm_transform.rotation = Quaternion(0.0, 0.0, np.sin(angle / 2), np.cos(angle / 2))

    # Create a regular Transform with same values
    transform = Transform(
        translation=Vector3(2.0, 1.0, 0.0),
        rotation=Quaternion(0.0, 0.0, np.sin(angle / 2), np.cos(angle / 2)),
    )

    # Test both + and @ operators with LCMTransform
    pose_lcm_add = initial_pose + lcm_transform
    pose_lcm_matmul = initial_pose @ lcm_transform

    # Test both + and @ operators with Transform
    pose_tf_add = initial_pose + transform
    pose_tf_matmul = initial_pose @ transform

    # All results should be the same
    assert np.isclose(pose_lcm_add.position.x, 3.0, atol=1e-10)
    assert np.isclose(pose_lcm_add.position.y, 1.0, atol=1e-10)
    assert np.isclose(pose_lcm_add.position.z, 0.0, atol=1e-10)

    # Check that all methods produce the same result
    assert pose_lcm_add.position == pose_lcm_matmul.position
    assert pose_lcm_add.position == pose_tf_add.position
    assert pose_lcm_add.position == pose_tf_matmul.position

    assert pose_lcm_add.orientation == pose_lcm_matmul.orientation
    assert pose_lcm_add.orientation == pose_tf_add.orientation
    assert pose_lcm_add.orientation == pose_tf_matmul.orientation

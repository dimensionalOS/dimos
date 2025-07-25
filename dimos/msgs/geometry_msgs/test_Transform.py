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

import time

import numpy as np
import pytest
from dimos_lcm.geometry_msgs import Transform as LCMTransform
from dimos_lcm.geometry_msgs import TransformStamped as LCMTransformStamped

from dimos.msgs.geometry_msgs import Pose, PoseStamped, Quaternion, Transform, Vector3


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
    tf2 = Transform(translation=trans, rotation=rot)
    assert tf2.translation == trans
    assert tf2.rotation == rot

    # Test initialization with only translation
    tf5 = Transform(translation=Vector3(7.0, 8.0, 9.0))
    assert tf5.translation.x == 7.0
    assert tf5.translation.y == 8.0
    assert tf5.translation.z == 9.0
    assert tf5.rotation.w == 1.0  # Identity rotation

    # Test initialization with only rotation
    tf6 = Transform(rotation=Quaternion(0.0, 0.0, 0.0, 1.0))
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
    tf1 = Transform(translation=Vector3(1, 2, 3), rotation=Quaternion(0, 0, 0, 1))
    tf2 = Transform(translation=Vector3(1, 2, 3), rotation=Quaternion(0, 0, 0, 1))
    tf3 = Transform(translation=Vector3(1, 2, 4), rotation=Quaternion(0, 0, 0, 1))  # Different z
    tf4 = Transform(
        translation=Vector3(1, 2, 3), rotation=Quaternion(0, 0, 1, 0)
    )  # Different rotation

    assert tf1 == tf2
    assert tf1 != tf3
    assert tf1 != tf4
    assert tf1 != "not a transform"


def test_transform_string_representations():
    tf = Transform(
        translation=Vector3(1.5, -2.0, 3.14), rotation=Quaternion(0, 0, 0.707107, 0.707107)
    )

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
        ),  # 45� around Z
    )

    transform2 = Transform(
        translation=Vector3(0.0, 1.0, 1.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),  # No rotation
    )

    transformed_pose1 = initial_pose @ transform1
    transformed_pose2 = initial_pose @ transform1 @ transform2

    # Test transformed_pose1: initial_pose + transform1
    # Since the pose is rotated 90� (facing +Y), moving forward (local X)
    # means moving in the +Y direction in world frame
    assert np.isclose(transformed_pose1.position.x, 0.0, atol=1e-10)
    assert np.isclose(transformed_pose1.position.y, 1.0, atol=1e-10)
    assert np.isclose(transformed_pose1.position.z, 0.0, atol=1e-10)

    # Orientation should be 90� + 45� = 135� around Z
    total_angle1 = angle + rotation_angle  # 135 degrees
    assert np.isclose(transformed_pose1.orientation.x, 0.0, atol=1e-10)
    assert np.isclose(transformed_pose1.orientation.y, 0.0, atol=1e-10)
    assert np.isclose(transformed_pose1.orientation.z, np.sin(total_angle1 / 2), atol=1e-10)
    assert np.isclose(transformed_pose1.orientation.w, np.cos(total_angle1 / 2), atol=1e-10)

    # Test transformed_pose2: initial_pose + transform1 + transform2
    # Starting from (0, 0, 0) facing 90�:
    #
    # - Apply transform1: move 1 forward (along +Y) � (0, 1, 0), now facing 135�
    #
    # - Apply transform2: move 1 in local Y and 1 up
    #   At 135�, local Y points at 225� (135� + 90�)
    #
    #   x += cos(225�) = -2/2, y += sin(225�) = -2/2
    sqrt2_2 = np.sqrt(2) / 2
    expected_x = 0.0 - sqrt2_2  # 0 - 2/2 H -0.707
    expected_y = 1.0 - sqrt2_2  # 1 - 2/2 H 0.293
    expected_z = 1.0  # 0 + 1

    assert np.isclose(transformed_pose2.position.x, expected_x, atol=1e-10)
    assert np.isclose(transformed_pose2.position.y, expected_y, atol=1e-10)
    assert np.isclose(transformed_pose2.position.z, expected_z, atol=1e-10)

    # Orientation should be 135� (only transform1 has rotation)
    total_angle2 = total_angle1  # 135 degrees (transform2 has no rotation)
    assert np.isclose(transformed_pose2.orientation.x, 0.0, atol=1e-10)
    assert np.isclose(transformed_pose2.orientation.y, 0.0, atol=1e-10)
    assert np.isclose(transformed_pose2.orientation.z, np.sin(total_angle2 / 2), atol=1e-10)
    assert np.isclose(transformed_pose2.orientation.w, np.cos(total_angle2 / 2), atol=1e-10)


def test_lcm_encode_decode():
    angle = np.pi / 2
    transform = Transform(
        translation=Vector3(2.0, 1.0, 0.0),
        rotation=Quaternion(0.0, 0.0, np.sin(angle / 2), np.cos(angle / 2)),
    )

    data = transform.lcm_encode()

    decoded_transform = Transform.lcm_decode(data)

    assert decoded_transform == transform


def test_transform_addition():
    # Test 1: Simple translation addition (no rotation)
    t1 = Transform(
        translation=Vector3(1, 0, 0),
        rotation=Quaternion(0, 0, 0, 1),  # identity rotation
    )
    t2 = Transform(
        translation=Vector3(2, 0, 0),
        rotation=Quaternion(0, 0, 0, 1),  # identity rotation
    )
    t3 = t1 + t2
    assert t3.translation == Vector3(3, 0, 0)
    assert t3.rotation == Quaternion(0, 0, 0, 1)

    # Test 2: 90-degree rotation composition
    # First transform: move 1 unit in X
    t1 = Transform(
        translation=Vector3(1, 0, 0),
        rotation=Quaternion(0, 0, 0, 1),  # identity
    )
    # Second transform: move 1 unit in X with 90-degree rotation around Z
    angle = np.pi / 2
    t2 = Transform(
        translation=Vector3(1, 0, 0),
        rotation=Quaternion(0, 0, np.sin(angle / 2), np.cos(angle / 2)),
    )
    t3 = t1 + t2
    assert t3.translation == Vector3(2, 0, 0)
    # Rotation should be 90 degrees around Z
    assert np.isclose(t3.rotation.x, 0.0, atol=1e-10)
    assert np.isclose(t3.rotation.y, 0.0, atol=1e-10)
    assert np.isclose(t3.rotation.z, np.sin(angle / 2), atol=1e-10)
    assert np.isclose(t3.rotation.w, np.cos(angle / 2), atol=1e-10)

    # Test 3: Rotation affects translation
    # First transform: 90-degree rotation around Z
    t1 = Transform(
        translation=Vector3(0, 0, 0),
        rotation=Quaternion(0, 0, np.sin(angle / 2), np.cos(angle / 2)),  # 90° around Z
    )
    # Second transform: move 1 unit in X
    t2 = Transform(
        translation=Vector3(1, 0, 0),
        rotation=Quaternion(0, 0, 0, 1),  # identity
    )
    t3 = t1 + t2
    # X direction rotated 90° becomes Y direction
    assert np.isclose(t3.translation.x, 0.0, atol=1e-10)
    assert np.isclose(t3.translation.y, 1.0, atol=1e-10)
    assert np.isclose(t3.translation.z, 0.0, atol=1e-10)
    # Rotation remains 90° around Z
    assert np.isclose(t3.rotation.z, np.sin(angle / 2), atol=1e-10)
    assert np.isclose(t3.rotation.w, np.cos(angle / 2), atol=1e-10)

    # Test 4: Frame tracking
    t1 = Transform(
        translation=Vector3(1, 0, 0),
        rotation=Quaternion(0, 0, 0, 1),
        frame_id="world",
        child_frame_id="robot",
    )
    t2 = Transform(
        translation=Vector3(2, 0, 0),
        rotation=Quaternion(0, 0, 0, 1),
        frame_id="robot",
        child_frame_id="sensor",
    )
    t3 = t1 + t2
    assert t3.frame_id == "world"
    assert t3.child_frame_id == "sensor"

    # Test 5: Type error
    with pytest.raises(TypeError):
        t1 + "not a transform"


def test_transform_from_pose():
    """Test converting Pose to Transform"""
    # Create a Pose with position and orientation
    pose = Pose(
        position=Vector3(1.0, 2.0, 3.0),
        orientation=Quaternion(0.0, 0.0, 0.707, 0.707),  # 90 degrees around Z
    )

    # Convert to Transform
    transform = Transform.from_pose("base_link", pose)

    # Check that translation and rotation match
    assert transform.translation == pose.position
    assert transform.rotation == pose.orientation
    assert transform.frame_id == "world"  # default frame_id
    assert transform.child_frame_id == "base_link"  # passed as first argument


def test_transform_from_pose_stamped():
    """Test converting PoseStamped to Transform"""
    # Create a PoseStamped with position, orientation, timestamp and frame
    test_time = time.time()
    pose_stamped = PoseStamped(
        ts=test_time,
        frame_id="map",
        position=Vector3(4.0, 5.0, 6.0),
        orientation=Quaternion(0.0, 0.707, 0.0, 0.707),  # 90 degrees around Y
    )

    # Convert to Transform
    transform = Transform.from_pose("robot_base", pose_stamped)

    # Check that all fields match
    assert transform.translation == pose_stamped.position
    assert transform.rotation == pose_stamped.orientation
    assert transform.frame_id == pose_stamped.frame_id
    assert transform.ts == pose_stamped.ts
    assert transform.child_frame_id == "robot_base"  # passed as first argument


def test_transform_from_pose_variants():
    """Test from_pose with different Pose initialization methods"""
    # Test with Pose created from x,y,z
    pose1 = Pose(1.0, 2.0, 3.0)
    transform1 = Transform.from_pose("base_link", pose1)
    assert transform1.translation.x == 1.0
    assert transform1.translation.y == 2.0
    assert transform1.translation.z == 3.0
    assert transform1.rotation.w == 1.0  # Identity quaternion

    # Test with Pose created from tuple
    pose2 = Pose(([7.0, 8.0, 9.0], [0.0, 0.0, 0.0, 1.0]))
    transform2 = Transform.from_pose("base_link", pose2)
    assert transform2.translation.x == 7.0
    assert transform2.translation.y == 8.0
    assert transform2.translation.z == 9.0

    # Test with Pose created from dict
    pose3 = Pose({"position": [10.0, 11.0, 12.0], "orientation": [0.0, 0.0, 0.0, 1.0]})
    transform3 = Transform.from_pose("base_link", pose3)
    assert transform3.translation.x == 10.0
    assert transform3.translation.y == 11.0
    assert transform3.translation.z == 12.0


def test_transform_from_pose_invalid_type():
    """Test that from_pose raises TypeError for invalid types"""
    with pytest.raises(TypeError):
        Transform.from_pose("not a pose")

    with pytest.raises(TypeError):
        Transform.from_pose(42)

    with pytest.raises(TypeError):
        Transform.from_pose(None)

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
from dimos_lcm.geometry_msgs import Transform

from dimos.msgs.geometry_msgs import Pose, Quaternion, Vector3


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

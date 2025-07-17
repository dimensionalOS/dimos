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
    """Test adding a Transform to a Pose to get a new transformed Pose."""
    # Create a pose at (1, 0, 0) with no rotation
    initial_pose = Pose(1.0, 0.0, 0.0)

    # Create a transform that translates by (2, 1, 0) and rotates 90 degrees around Z
    transform = Transform()
    transform.translation = Vector3(2.0, 1.0, 0.0)
    # 90 degree rotation around Z axis
    angle = np.pi / 2
    transform.rotation = Quaternion(0.0, 0.0, np.sin(angle / 2), np.cos(angle / 2))

    # Apply the transform to the pose
    transformed_pose = initial_pose + transform

    # The expected result:
    # - Transform is applied in the pose's frame
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


def test_pose_add_transform_with_rotation():
    """Test adding a Transform to a rotated Pose."""
    # Create a pose at (0, 0, 0) rotated 90 degrees around Z
    angle = np.pi / 2
    initial_pose = Pose(0.0, 0.0, 0.0, 0.0, 0.0, np.sin(angle / 2), np.cos(angle / 2))

    # Create a transform that moves 1 unit forward (along X in local frame)
    transform = Transform()
    transform.translation = Vector3(1.0, 0.0, 0.0)
    transform.rotation = Quaternion(0.0, 0.0, 0.0, 1.0)  # No rotation

    # Apply the transform
    transformed_pose = initial_pose + transform

    # Since the pose is rotated 90° (facing left), moving forward (local X)
    # means moving in the negative Y direction in world frame
    assert np.isclose(transformed_pose.position.x, 0.0, atol=1e-10)
    assert np.isclose(transformed_pose.position.y, 1.0, atol=1e-10)
    assert np.isclose(transformed_pose.position.z, 0.0, atol=1e-10)

    # Orientation should remain 90 degrees around Z
    assert np.isclose(transformed_pose.orientation.x, 0.0, atol=1e-10)
    assert np.isclose(transformed_pose.orientation.y, 0.0, atol=1e-10)
    assert np.isclose(transformed_pose.orientation.z, np.sin(angle / 2), atol=1e-10)
    assert np.isclose(transformed_pose.orientation.w, np.cos(angle / 2), atol=1e-10)

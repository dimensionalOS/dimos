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
from scipy.spatial.transform import Rotation as R

from dimos.msgs.geometry_msgs import Pose, Quaternion, Transform, Vector3
from dimos.utils import transform_utils


class TestNormalizeAngle:
    def test_normalize_angle_zero(self) -> None:
        assert transform_utils.normalize_angle(0) == 0

    def test_normalize_angle_pi(self) -> None:
        assert np.isclose(transform_utils.normalize_angle(np.pi), np.pi)

    def test_normalize_angle_negative_pi(self) -> None:
        assert np.isclose(transform_utils.normalize_angle(-np.pi), -np.pi)

    def test_normalize_angle_two_pi(self) -> None:
        # 2*pi should normalize to 0
        assert np.isclose(transform_utils.normalize_angle(2 * np.pi), 0, atol=1e-10)

    def test_normalize_angle_large_positive(self) -> None:
        # Large positive angle should wrap to [-pi, pi]
        angle = 5 * np.pi
        normalized = transform_utils.normalize_angle(angle)
        assert -np.pi <= normalized <= np.pi
        assert np.isclose(normalized, np.pi)

    def test_normalize_angle_large_negative(self) -> None:
        # Large negative angle should wrap to [-pi, pi]
        angle = -5 * np.pi
        normalized = transform_utils.normalize_angle(angle)
        assert -np.pi <= normalized <= np.pi
        # -5*pi = -pi (odd multiple of pi wraps to -pi)
        assert np.isclose(normalized, -np.pi) or np.isclose(normalized, np.pi)


# Tests for distance_angle_to_goal_xy removed as function doesn't exist in the module


class TestPoseToMatrix:
    def test_identity_pose(self) -> None:
        pose = Pose(Vector3(0, 0, 0), Quaternion(0, 0, 0, 1))
        T = transform_utils.pose_to_matrix(pose)
        assert np.allclose(T, np.eye(4))

    def test_translation_only(self) -> None:
        pose = Pose(Vector3(1, 2, 3), Quaternion(0, 0, 0, 1))
        T = transform_utils.pose_to_matrix(pose)
        expected = np.eye(4)
        expected[:3, 3] = [1, 2, 3]
        assert np.allclose(T, expected)

    def test_rotation_only_90_degrees_z(self) -> None:
        # 90 degree rotation around z-axis
        quat = R.from_euler("z", np.pi / 2).as_quat()
        pose = Pose(Vector3(0, 0, 0), Quaternion(quat[0], quat[1], quat[2], quat[3]))
        T = transform_utils.pose_to_matrix(pose)

        # Check rotation part
        expected_rot = R.from_euler("z", np.pi / 2).as_matrix()
        assert np.allclose(T[:3, :3], expected_rot)

        # Check translation is zero
        assert np.allclose(T[:3, 3], [0, 0, 0])

    def test_translation_and_rotation(self) -> None:
        quat = R.from_euler("xyz", [np.pi / 4, np.pi / 6, np.pi / 3]).as_quat()
        pose = Pose(Vector3(5, -3, 2), Quaternion(quat[0], quat[1], quat[2], quat[3]))
        T = transform_utils.pose_to_matrix(pose)

        # Check translation
        assert np.allclose(T[:3, 3], [5, -3, 2])

        # Check rotation
        expected_rot = R.from_euler("xyz", [np.pi / 4, np.pi / 6, np.pi / 3]).as_matrix()
        assert np.allclose(T[:3, :3], expected_rot)

        # Check bottom row
        assert np.allclose(T[3, :], [0, 0, 0, 1])

    def test_zero_norm_quaternion(self) -> None:
        # Test handling of zero norm quaternion
        pose = Pose(Vector3(1, 2, 3), Quaternion(0, 0, 0, 0))
        T = transform_utils.pose_to_matrix(pose)

        # Should use identity rotation
        expected = np.eye(4)
        expected[:3, 3] = [1, 2, 3]
        assert np.allclose(T, expected)


class TestMatrixToPose:
    def test_identity_matrix(self) -> None:
        T = np.eye(4)
        pose = transform_utils.matrix_to_pose(T)
        assert pose.position.x == 0
        assert pose.position.y == 0
        assert pose.position.z == 0
        assert np.isclose(pose.orientation.w, 1)
        assert np.isclose(pose.orientation.x, 0)
        assert np.isclose(pose.orientation.y, 0)
        assert np.isclose(pose.orientation.z, 0)

    def test_translation_only(self) -> None:
        T = np.eye(4)
        T[:3, 3] = [1, 2, 3]
        pose = transform_utils.matrix_to_pose(T)
        assert pose.position.x == 1
        assert pose.position.y == 2
        assert pose.position.z == 3
        assert np.isclose(pose.orientation.w, 1)

    def test_rotation_only(self) -> None:
        T = np.eye(4)
        T[:3, :3] = R.from_euler("z", np.pi / 2).as_matrix()
        pose = transform_utils.matrix_to_pose(T)

        # Check position is zero
        assert pose.position.x == 0
        assert pose.position.y == 0
        assert pose.position.z == 0

        # Check rotation
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        recovered_rot = R.from_quat(quat).as_matrix()
        assert np.allclose(recovered_rot, T[:3, :3])

    def test_round_trip_conversion(self) -> None:
        # Test that pose -> matrix -> pose gives same result
        # Use a properly normalized quaternion
        quat = R.from_euler("xyz", [0.1, 0.2, 0.3]).as_quat()
        original_pose = Pose(
            Vector3(1.5, -2.3, 0.7), Quaternion(quat[0], quat[1], quat[2], quat[3])
        )
        T = transform_utils.pose_to_matrix(original_pose)
        recovered_pose = transform_utils.matrix_to_pose(T)

        assert np.isclose(recovered_pose.position.x, original_pose.position.x)
        assert np.isclose(recovered_pose.position.y, original_pose.position.y)
        assert np.isclose(recovered_pose.position.z, original_pose.position.z)
        assert np.isclose(recovered_pose.orientation.x, original_pose.orientation.x, atol=1e-6)
        assert np.isclose(recovered_pose.orientation.y, original_pose.orientation.y, atol=1e-6)
        assert np.isclose(recovered_pose.orientation.z, original_pose.orientation.z, atol=1e-6)
        assert np.isclose(recovered_pose.orientation.w, original_pose.orientation.w, atol=1e-6)


class TestApplyTransform:
    def test_identity_transform(self) -> None:
        pose = Pose(Vector3(1, 2, 3), Quaternion(0, 0, 0, 1))
        T_identity = np.eye(4)
        result = transform_utils.apply_transform(pose, T_identity)

        assert np.isclose(result.position.x, pose.position.x)
        assert np.isclose(result.position.y, pose.position.y)
        assert np.isclose(result.position.z, pose.position.z)

    def test_translation_transform(self) -> None:
        pose = Pose(Vector3(1, 0, 0), Quaternion(0, 0, 0, 1))
        T = np.eye(4)
        T[:3, 3] = [2, 3, 4]
        result = transform_utils.apply_transform(pose, T)

        assert np.isclose(result.position.x, 3)  # 2 + 1
        assert np.isclose(result.position.y, 3)  # 3 + 0
        assert np.isclose(result.position.z, 4)  # 4 + 0

    def test_rotation_transform(self) -> None:
        pose = Pose(Vector3(1, 0, 0), Quaternion(0, 0, 0, 1))
        T = np.eye(4)
        T[:3, :3] = R.from_euler("z", np.pi / 2).as_matrix()  # 90 degree rotation
        result = transform_utils.apply_transform(pose, T)

        # After 90 degree rotation around z, point (1,0,0) becomes (0,1,0)
        assert np.isclose(result.position.x, 0, atol=1e-10)
        assert np.isclose(result.position.y, 1)
        assert np.isclose(result.position.z, 0)

    def test_transform_with_transform_object(self) -> None:
        pose = Pose(Vector3(1, 0, 0), Quaternion(0, 0, 0, 1))
        pose.frame_id = "base"

        transform = Transform()
        transform.frame_id = "world"
        transform.child_frame_id = "base"
        transform.translation = Vector3(2, 3, 4)
        transform.rotation = Quaternion(0, 0, 0, 1)

        result = transform_utils.apply_transform(pose, transform)
        assert np.isclose(result.position.x, 3)
        assert np.isclose(result.position.y, 3)
        assert np.isclose(result.position.z, 4)

    def test_transform_frame_mismatch_raises(self) -> None:
        pose = Pose(Vector3(1, 0, 0), Quaternion(0, 0, 0, 1))
        pose.frame_id = "base"

        transform = Transform()
        transform.frame_id = "world"
        transform.child_frame_id = "different_frame"
        transform.translation = Vector3(2, 3, 4)
        transform.rotation = Quaternion(0, 0, 0, 1)

        with pytest.raises(ValueError, match="does not match"):
            transform_utils.apply_transform(pose, transform)


class TestOpticalToRobotFrame:
    def test_identity_at_origin(self) -> None:
        pose = Pose(Vector3(0, 0, 0), Quaternion(0, 0, 0, 1))
        result = transform_utils.optical_to_robot_frame(pose)
        assert result.position.x == 0
        assert result.position.y == 0
        assert result.position.z == 0

    def test_position_transformation(self) -> None:
        # Optical: X=right(1), Y=down(0), Z=forward(0)
        pose = Pose(Vector3(1, 0, 0), Quaternion(0, 0, 0, 1))
        result = transform_utils.optical_to_robot_frame(pose)

        # Robot: X=forward(0), Y=left(-1), Z=up(0)
        assert np.isclose(result.position.x, 0)  # Forward = Camera Z
        assert np.isclose(result.position.y, -1)  # Left = -Camera X
        assert np.isclose(result.position.z, 0)  # Up = -Camera Y

    def test_forward_position(self) -> None:
        # Optical: X=right(0), Y=down(0), Z=forward(2)
        pose = Pose(Vector3(0, 0, 2), Quaternion(0, 0, 0, 1))
        result = transform_utils.optical_to_robot_frame(pose)

        # Robot: X=forward(2), Y=left(0), Z=up(0)
        assert np.isclose(result.position.x, 2)
        assert np.isclose(result.position.y, 0)
        assert np.isclose(result.position.z, 0)

    def test_down_position(self) -> None:
        # Optical: X=right(0), Y=down(3), Z=forward(0)
        pose = Pose(Vector3(0, 3, 0), Quaternion(0, 0, 0, 1))
        result = transform_utils.optical_to_robot_frame(pose)

        # Robot: X=forward(0), Y=left(0), Z=up(-3)
        assert np.isclose(result.position.x, 0)
        assert np.isclose(result.position.y, 0)
        assert np.isclose(result.position.z, -3)

    def test_round_trip_optical_robot(self) -> None:
        original_pose = Pose(Vector3(1, 2, 3), Quaternion(0.1, 0.2, 0.3, 0.9165151389911680))
        robot_pose = transform_utils.optical_to_robot_frame(original_pose)
        recovered_pose = transform_utils.robot_to_optical_frame(robot_pose)

        assert np.isclose(recovered_pose.position.x, original_pose.position.x, atol=1e-10)
        assert np.isclose(recovered_pose.position.y, original_pose.position.y, atol=1e-10)
        assert np.isclose(recovered_pose.position.z, original_pose.position.z, atol=1e-10)


class TestRobotToOpticalFrame:
    def test_position_transformation(self) -> None:
        # Robot: X=forward(1), Y=left(0), Z=up(0)
        pose = Pose(Vector3(1, 0, 0), Quaternion(0, 0, 0, 1))
        result = transform_utils.robot_to_optical_frame(pose)

        # Optical: X=right(0), Y=down(0), Z=forward(1)
        assert np.isclose(result.position.x, 0)
        assert np.isclose(result.position.y, 0)
        assert np.isclose(result.position.z, 1)

    def test_left_position(self) -> None:
        # Robot: X=forward(0), Y=left(2), Z=up(0)
        pose = Pose(Vector3(0, 2, 0), Quaternion(0, 0, 0, 1))
        result = transform_utils.robot_to_optical_frame(pose)

        # Optical: X=right(-2), Y=down(0), Z=forward(0)
        assert np.isclose(result.position.x, -2)
        assert np.isclose(result.position.y, 0)
        assert np.isclose(result.position.z, 0)

    def test_up_position(self) -> None:
        # Robot: X=forward(0), Y=left(0), Z=up(3)
        pose = Pose(Vector3(0, 0, 3), Quaternion(0, 0, 0, 1))
        result = transform_utils.robot_to_optical_frame(pose)

        # Optical: X=right(0), Y=down(-3), Z=forward(0)
        assert np.isclose(result.position.x, 0)
        assert np.isclose(result.position.y, -3)
        assert np.isclose(result.position.z, 0)


class TestYawTowardsPoint:
    def test_yaw_from_origin(self) -> None:
        # Point at (1, 0) from origin should have yaw = 0
        position = Vector3(1, 0, 0)
        yaw = transform_utils.yaw_towards_point(position)
        assert np.isclose(yaw, 0)

    def test_yaw_ninety_degrees(self) -> None:
        # Point at (0, 1) from origin should have yaw = pi/2
        position = Vector3(0, 1, 0)
        yaw = transform_utils.yaw_towards_point(position)
        assert np.isclose(yaw, np.pi / 2)

    def test_yaw_negative_ninety_degrees(self) -> None:
        # Point at (0, -1) from origin should have yaw = -pi/2
        position = Vector3(0, -1, 0)
        yaw = transform_utils.yaw_towards_point(position)
        assert np.isclose(yaw, -np.pi / 2)

    def test_yaw_forty_five_degrees(self) -> None:
        # Point at (1, 1) from origin should have yaw = pi/4
        position = Vector3(1, 1, 0)
        yaw = transform_utils.yaw_towards_point(position)
        assert np.isclose(yaw, np.pi / 4)

    def test_yaw_with_custom_target(self) -> None:
        # Point at (3, 2) from target (1, 1)
        position = Vector3(3, 2, 0)
        target = Vector3(1, 1, 0)
        yaw = transform_utils.yaw_towards_point(position, target)
        # Direction is (2, 1), so yaw = atan2(1, 2)
        expected = np.arctan2(1, 2)
        assert np.isclose(yaw, expected)


# Tests for transform_robot_to_map removed as function doesn't exist in the module


class TestCreateTransformFrom6DOF:
    def test_identity_transform(self) -> None:
        trans = Vector3(0, 0, 0)
        euler = Vector3(0, 0, 0)
        T = transform_utils.create_transform_from_6dof(trans, euler)
        assert np.allclose(T, np.eye(4))

    def test_translation_only(self) -> None:
        trans = Vector3(1, 2, 3)
        euler = Vector3(0, 0, 0)
        T = transform_utils.create_transform_from_6dof(trans, euler)

        expected = np.eye(4)
        expected[:3, 3] = [1, 2, 3]
        assert np.allclose(T, expected)

    def test_rotation_only(self) -> None:
        trans = Vector3(0, 0, 0)
        euler = Vector3(np.pi / 4, np.pi / 6, np.pi / 3)
        T = transform_utils.create_transform_from_6dof(trans, euler)

        expected_rot = R.from_euler("xyz", [np.pi / 4, np.pi / 6, np.pi / 3]).as_matrix()
        assert np.allclose(T[:3, :3], expected_rot)
        assert np.allclose(T[:3, 3], [0, 0, 0])
        assert np.allclose(T[3, :], [0, 0, 0, 1])

    def test_translation_and_rotation(self) -> None:
        trans = Vector3(5, -3, 2)
        euler = Vector3(0.1, 0.2, 0.3)
        T = transform_utils.create_transform_from_6dof(trans, euler)

        expected_rot = R.from_euler("xyz", [0.1, 0.2, 0.3]).as_matrix()
        assert np.allclose(T[:3, :3], expected_rot)
        assert np.allclose(T[:3, 3], [5, -3, 2])

    def test_small_angles_threshold(self) -> None:
        trans = Vector3(1, 2, 3)
        euler = Vector3(1e-7, 1e-8, 1e-9)  # Very small angles
        T = transform_utils.create_transform_from_6dof(trans, euler)

        # Should be effectively identity rotation
        expected = np.eye(4)
        expected[:3, 3] = [1, 2, 3]
        assert np.allclose(T, expected, atol=1e-6)


class TestInvertTransform:
    def test_identity_inverse(self) -> None:
        T = np.eye(4)
        T_inv = transform_utils.invert_transform(T)
        assert np.allclose(T_inv, np.eye(4))

    def test_translation_inverse(self) -> None:
        T = np.eye(4)
        T[:3, 3] = [1, 2, 3]
        T_inv = transform_utils.invert_transform(T)

        # Inverse should negate translation
        expected = np.eye(4)
        expected[:3, 3] = [-1, -2, -3]
        assert np.allclose(T_inv, expected)

    def test_rotation_inverse(self) -> None:
        T = np.eye(4)
        T[:3, :3] = R.from_euler("z", np.pi / 2).as_matrix()
        T_inv = transform_utils.invert_transform(T)

        # Inverse rotation is transpose
        expected = np.eye(4)
        expected[:3, :3] = R.from_euler("z", -np.pi / 2).as_matrix()
        assert np.allclose(T_inv, expected)

    def test_general_transform_inverse(self) -> None:
        T = np.eye(4)
        T[:3, :3] = R.from_euler("xyz", [0.1, 0.2, 0.3]).as_matrix()
        T[:3, 3] = [1, 2, 3]

        T_inv = transform_utils.invert_transform(T)

        # T @ T_inv should be identity
        result = T @ T_inv
        assert np.allclose(result, np.eye(4))

        # T_inv @ T should also be identity
        result2 = T_inv @ T
        assert np.allclose(result2, np.eye(4))


class TestComposeTransforms:
    def test_no_transforms(self) -> None:
        result = transform_utils.compose_transforms()
        assert np.allclose(result, np.eye(4))

    def test_single_transform(self) -> None:
        T = np.eye(4)
        T[:3, 3] = [1, 2, 3]
        result = transform_utils.compose_transforms(T)
        assert np.allclose(result, T)

    def test_two_translations(self) -> None:
        T1 = np.eye(4)
        T1[:3, 3] = [1, 0, 0]

        T2 = np.eye(4)
        T2[:3, 3] = [0, 2, 0]

        result = transform_utils.compose_transforms(T1, T2)

        expected = np.eye(4)
        expected[:3, 3] = [1, 2, 0]
        assert np.allclose(result, expected)

    def test_three_transforms(self) -> None:
        T1 = np.eye(4)
        T1[:3, 3] = [1, 0, 0]

        T2 = np.eye(4)
        T2[:3, :3] = R.from_euler("z", np.pi / 2).as_matrix()

        T3 = np.eye(4)
        T3[:3, 3] = [1, 0, 0]

        result = transform_utils.compose_transforms(T1, T2, T3)
        expected = T1 @ T2 @ T3
        assert np.allclose(result, expected)


class TestEulerToQuaternion:
    def test_zero_euler(self) -> None:
        euler = Vector3(0, 0, 0)
        quat = transform_utils.euler_to_quaternion(euler)
        assert np.isclose(quat.w, 1)
        assert np.isclose(quat.x, 0)
        assert np.isclose(quat.y, 0)
        assert np.isclose(quat.z, 0)

    def test_roll_only(self) -> None:
        euler = Vector3(np.pi / 2, 0, 0)
        quat = transform_utils.euler_to_quaternion(euler)

        # Verify by converting back
        recovered = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler("xyz")
        assert np.isclose(recovered[0], np.pi / 2)
        assert np.isclose(recovered[1], 0)
        assert np.isclose(recovered[2], 0)

    def test_pitch_only(self) -> None:
        euler = Vector3(0, np.pi / 3, 0)
        quat = transform_utils.euler_to_quaternion(euler)

        recovered = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler("xyz")
        assert np.isclose(recovered[0], 0)
        assert np.isclose(recovered[1], np.pi / 3)
        assert np.isclose(recovered[2], 0)

    def test_yaw_only(self) -> None:
        euler = Vector3(0, 0, np.pi / 4)
        quat = transform_utils.euler_to_quaternion(euler)

        recovered = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler("xyz")
        assert np.isclose(recovered[0], 0)
        assert np.isclose(recovered[1], 0)
        assert np.isclose(recovered[2], np.pi / 4)

    def test_degrees_mode(self) -> None:
        euler = Vector3(45, 30, 60)  # degrees
        quat = transform_utils.euler_to_quaternion(euler, degrees=True)

        recovered = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler("xyz", degrees=True)
        assert np.isclose(recovered[0], 45)
        assert np.isclose(recovered[1], 30)
        assert np.isclose(recovered[2], 60)


class TestQuaternionToEuler:
    def test_identity_quaternion(self) -> None:
        quat = Quaternion(0, 0, 0, 1)
        euler = transform_utils.quaternion_to_euler(quat)
        assert np.isclose(euler.x, 0)
        assert np.isclose(euler.y, 0)
        assert np.isclose(euler.z, 0)

    def test_90_degree_yaw(self) -> None:
        # Create quaternion for 90 degree yaw rotation
        r = R.from_euler("z", np.pi / 2)
        q = r.as_quat()
        quat = Quaternion(q[0], q[1], q[2], q[3])

        euler = transform_utils.quaternion_to_euler(quat)
        assert np.isclose(euler.x, 0)
        assert np.isclose(euler.y, 0)
        assert np.isclose(euler.z, np.pi / 2)

    def test_round_trip_euler_quaternion(self) -> None:
        original_euler = Vector3(0.3, 0.5, 0.7)
        quat = transform_utils.euler_to_quaternion(original_euler)
        recovered_euler = transform_utils.quaternion_to_euler(quat)

        assert np.isclose(recovered_euler.x, original_euler.x, atol=1e-10)
        assert np.isclose(recovered_euler.y, original_euler.y, atol=1e-10)
        assert np.isclose(recovered_euler.z, original_euler.z, atol=1e-10)

    def test_degrees_mode(self) -> None:
        # Create quaternion for 45 degree yaw rotation
        r = R.from_euler("z", 45, degrees=True)
        q = r.as_quat()
        quat = Quaternion(q[0], q[1], q[2], q[3])

        euler = transform_utils.quaternion_to_euler(quat, degrees=True)
        assert np.isclose(euler.x, 0)
        assert np.isclose(euler.y, 0)
        assert np.isclose(euler.z, 45)

    def test_angle_normalization(self) -> None:
        # Test that angles are normalized to [-pi, pi]
        r = R.from_euler("xyz", [3 * np.pi, -3 * np.pi, 2 * np.pi])
        q = r.as_quat()
        quat = Quaternion(q[0], q[1], q[2], q[3])

        euler = transform_utils.quaternion_to_euler(quat)
        assert -np.pi <= euler.x <= np.pi
        assert -np.pi <= euler.y <= np.pi
        assert -np.pi <= euler.z <= np.pi


class TestGetDistance:
    def test_same_pose(self) -> None:
        pose1 = Pose(Vector3(1, 2, 3), Quaternion(0, 0, 0, 1))
        pose2 = Pose(Vector3(1, 2, 3), Quaternion(0.1, 0.2, 0.3, 0.9))
        distance = transform_utils.get_distance(pose1, pose2)
        assert np.isclose(distance, 0)

    def test_vector_distance(self) -> None:
        pose1 = Vector3(1, 2, 3)
        pose2 = Vector3(4, 5, 6)
        distance = transform_utils.get_distance(pose1, pose2)
        assert np.isclose(distance, np.sqrt(3**2 + 3**2 + 3**2))

    def test_distance_x_axis(self) -> None:
        pose1 = Pose(Vector3(0, 0, 0), Quaternion(0, 0, 0, 1))
        pose2 = Pose(Vector3(5, 0, 0), Quaternion(0, 0, 0, 1))
        distance = transform_utils.get_distance(pose1, pose2)
        assert np.isclose(distance, 5)

    def test_distance_y_axis(self) -> None:
        pose1 = Pose(Vector3(0, 0, 0), Quaternion(0, 0, 0, 1))
        pose2 = Pose(Vector3(0, 3, 0), Quaternion(0, 0, 0, 1))
        distance = transform_utils.get_distance(pose1, pose2)
        assert np.isclose(distance, 3)

    def test_distance_z_axis(self) -> None:
        pose1 = Pose(Vector3(0, 0, 0), Quaternion(0, 0, 0, 1))
        pose2 = Pose(Vector3(0, 0, 4), Quaternion(0, 0, 0, 1))
        distance = transform_utils.get_distance(pose1, pose2)
        assert np.isclose(distance, 4)

    def test_3d_distance(self) -> None:
        pose1 = Pose(Vector3(0, 0, 0), Quaternion(0, 0, 0, 1))
        pose2 = Pose(Vector3(3, 4, 0), Quaternion(0, 0, 0, 1))
        distance = transform_utils.get_distance(pose1, pose2)
        assert np.isclose(distance, 5)  # 3-4-5 triangle

    def test_negative_coordinates(self) -> None:
        pose1 = Pose(Vector3(-1, -2, -3), Quaternion(0, 0, 0, 1))
        pose2 = Pose(Vector3(1, 2, 3), Quaternion(0, 0, 0, 1))
        distance = transform_utils.get_distance(pose1, pose2)
        expected = np.sqrt(4 + 16 + 36)  # sqrt(56)
        assert np.isclose(distance, expected)


class TestRetractDistance:
    def test_retract_along_negative_z(self) -> None:
        # Default case: gripper approaches along -z axis
        # Positive distance moves away from the surface (opposite to approach direction)
        target_pose = Pose(Vector3(0, 0, 1), Quaternion(0, 0, 0, 1))
        retracted = transform_utils.offset_distance(target_pose, 0.5)

        # Moving along -z approach vector with positive distance = retracting upward
        # Since approach is -z and we retract (positive distance), we move in +z
        assert np.isclose(retracted.position.x, 0)
        assert np.isclose(retracted.position.y, 0)
        assert np.isclose(retracted.position.z, 0.5)  # 1 + 0.5 * (-1) = 0.5

        # Orientation should remain unchanged
        assert retracted.orientation.x == target_pose.orientation.x
        assert retracted.orientation.y == target_pose.orientation.y
        assert retracted.orientation.z == target_pose.orientation.z
        assert retracted.orientation.w == target_pose.orientation.w

    def test_retract_with_rotation(self) -> None:
        # Test with a rotated pose (90 degrees around x-axis)
        r = R.from_euler("x", np.pi / 2)
        q = r.as_quat()
        target_pose = Pose(Vector3(0, 0, 1), Quaternion(q[0], q[1], q[2], q[3]))

        retracted = transform_utils.offset_distance(target_pose, 0.5)

        # After 90 degree rotation around x, -z becomes +y
        assert np.isclose(retracted.position.x, 0)
        assert np.isclose(retracted.position.y, 0.5)  # Move along +y
        assert np.isclose(retracted.position.z, 1)

    def test_retract_negative_distance(self) -> None:
        # Negative distance should move forward (toward the approach direction)
        target_pose = Pose(Vector3(0, 0, 1), Quaternion(0, 0, 0, 1))
        retracted = transform_utils.offset_distance(target_pose, -0.3)

        # Moving along -z approach vector with negative distance = moving downward
        assert np.isclose(retracted.position.x, 0)
        assert np.isclose(retracted.position.y, 0)
        assert np.isclose(retracted.position.z, 1.3)  # 1 + (-0.3) * (-1) = 1.3

    def test_retract_arbitrary_pose(self) -> None:
        # Test with arbitrary position and rotation
        r = R.from_euler("xyz", [0.1, 0.2, 0.3])
        q = r.as_quat()
        target_pose = Pose(Vector3(5, 3, 2), Quaternion(q[0], q[1], q[2], q[3]))

        distance = 1.0
        retracted = transform_utils.offset_distance(target_pose, distance)

        # Verify the distance between original and retracted is as expected
        # (approximately, due to the approach vector direction)
        T_target = transform_utils.pose_to_matrix(target_pose)
        rotation_matrix = T_target[:3, :3]
        approach_vector = rotation_matrix @ np.array([0, 0, -1])

        expected_x = target_pose.position.x + distance * approach_vector[0]
        expected_y = target_pose.position.y + distance * approach_vector[1]
        expected_z = target_pose.position.z + distance * approach_vector[2]

        assert np.isclose(retracted.position.x, expected_x)
        assert np.isclose(retracted.position.y, expected_y)
        assert np.isclose(retracted.position.z, expected_z)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

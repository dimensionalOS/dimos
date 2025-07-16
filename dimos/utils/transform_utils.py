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
from typing import Tuple, Dict, Any
import logging
from scipy.spatial.transform import Rotation

from dimos.types.vector import Vector
from dimos.msgs.geometry_msgs import Pose, Vector3, Quaternion

logger = logging.getLogger(__name__)


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi] range"""
    return np.arctan2(np.sin(angle), np.cos(angle))


def distance_angle_to_goal_xy(distance: float, angle: float) -> Tuple[float, float]:
    """Convert distance and angle to goal x, y in robot frame"""
    return distance * np.cos(angle), distance * np.sin(angle)


def pose_to_matrix(pose: Pose) -> np.ndarray:
    """
    Convert pose to 4x4 homogeneous transform matrix.

    Args:
        pose: Pose object with position and orientation (quaternion)

    Returns:
        4x4 transformation matrix
    """
    # Extract position
    tx, ty, tz = pose.position.x, pose.position.y, pose.position.z

    # Create rotation matrix from quaternion using scipy
    quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    rotation = Rotation.from_quat(quat)
    R = rotation.as_matrix()

    # Create 4x4 transform
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [tx, ty, tz]

    return T


def matrix_to_pose(T: np.ndarray) -> Pose:
    """
    Convert 4x4 transformation matrix to Pose object.

    Args:
        T: 4x4 transformation matrix

    Returns:
        Pose object with position and orientation (quaternion)
    """
    # Extract position
    pos = Vector3(T[0, 3], T[1, 3], T[2, 3])

    # Extract rotation matrix and convert to quaternion
    R = T[:3, :3]
    rotation = Rotation.from_matrix(R)
    quat = rotation.as_quat()  # Returns [x, y, z, w]

    orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])

    return Pose(pos, orientation)


def apply_transform(pose: Pose, transform_matrix: np.ndarray) -> Pose:
    """
    Apply a transformation matrix to a pose.

    Args:
        pose: Input pose
        transform_matrix: 4x4 transformation matrix to apply

    Returns:
        Transformed pose
    """
    # Convert pose to matrix
    T_pose = pose_to_matrix(pose)

    # Apply transform
    T_result = transform_matrix @ T_pose

    # Convert back to pose
    return matrix_to_pose(T_result)


def optical_to_robot_frame(pose: Pose) -> Pose:
    """
    Convert pose from optical camera frame to robot frame convention.

    Optical Camera Frame (e.g., ZED):
    - X: Right
    - Y: Down
    - Z: Forward (away from camera)

    Robot Frame (ROS/REP-103):
    - X: Forward
    - Y: Left
    - Z: Up

    Args:
        pose: Pose in optical camera frame

    Returns:
        Pose in robot frame
    """
    # Position transformation
    robot_x = pose.position.z  # Forward = Camera Z
    robot_y = -pose.position.x  # Left = -Camera X
    robot_z = -pose.position.y  # Up = -Camera Y

    # Rotation transformation using quaternions
    # First convert quaternion to rotation matrix
    quat_optical = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    R_optical = Rotation.from_quat(quat_optical).as_matrix()

    # Coordinate frame transformation matrix from optical to robot
    # X_robot = Z_optical, Y_robot = -X_optical, Z_robot = -Y_optical
    T_frame = np.array(
        [
            [0, 0, 1],  # X_robot = Z_optical
            [-1, 0, 0],  # Y_robot = -X_optical
            [0, -1, 0],  # Z_robot = -Y_optical
        ]
    )

    # Transform the rotation matrix
    R_robot = T_frame @ R_optical @ T_frame.T

    # Convert back to quaternion
    quat_robot = Rotation.from_matrix(R_robot).as_quat()  # [x, y, z, w]

    return Pose(
        Vector3(robot_x, robot_y, robot_z),
        Quaternion(quat_robot[0], quat_robot[1], quat_robot[2], quat_robot[3]),
    )


def robot_to_optical_frame(pose: Pose) -> Pose:
    """
    Convert pose from robot frame to optical camera frame convention.
    This is the inverse of optical_to_robot_frame.

    Args:
        pose: Pose in robot frame

    Returns:
        Pose in optical camera frame
    """
    # Position transformation (inverse)
    optical_x = -pose.position.y  # Right = -Left
    optical_y = -pose.position.z  # Down = -Up
    optical_z = pose.position.x  # Forward = Forward

    # Rotation transformation using quaternions
    quat_robot = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    R_robot = Rotation.from_quat(quat_robot).as_matrix()

    # Coordinate frame transformation matrix from Robot to optical (inverse of optical to Robot)
    # This is the transpose of the forward transformation
    T_frame_inv = np.array(
        [
            [0, -1, 0],  # X_optical = -Y_robot
            [0, 0, -1],  # Y_optical = -Z_robot
            [1, 0, 0],  # Z_optical = X_robot
        ]
    )

    # Transform the rotation matrix
    R_optical = T_frame_inv @ R_robot @ T_frame_inv.T

    # Convert back to quaternion
    quat_optical = Rotation.from_matrix(R_optical).as_quat()  # [x, y, z, w]

    return Pose(
        Vector3(optical_x, optical_y, optical_z),
        Quaternion(quat_optical[0], quat_optical[1], quat_optical[2], quat_optical[3]),
    )


def yaw_towards_point(position: Vector, target_point: Vector = Vector(0.0, 0.0, 0.0)) -> float:
    """
    Calculate yaw angle from target point to position (away from target).
    This is commonly used for object orientation in grasping applications.
    Assumes robot frame where X is forward and Y is left.

    Args:
        position: Current position in robot frame
        target_point: Reference point (default: origin)

    Returns:
        Yaw angle in radians pointing from target_point to position
    """
    direction = position - target_point
    return np.arctan2(direction.y, direction.x)


def transform_robot_to_map(
    robot_position: Vector, robot_rotation: Vector, position: Vector, rotation: Vector
) -> Tuple[Vector, Vector]:
    """Transform position and rotation from robot frame to map frame.

    Args:
        robot_position: Current robot position in map frame
        robot_rotation: Current robot rotation in map frame
        position: Position in robot frame as Vector (x, y, z)
        rotation: Rotation in robot frame as Vector (roll, pitch, yaw) in radians

    Returns:
        Tuple of (transformed_position, transformed_rotation) where:
            - transformed_position: Vector (x, y, z) in map frame
            - transformed_rotation: Vector (roll, pitch, yaw) in map frame

    Example:
        obj_pos_robot = Vector(1.0, 0.5, 0.0)  # 1m forward, 0.5m left of robot
        obj_rot_robot = Vector(0.0, 0.0, 0.0)  # No rotation relative to robot

        map_pos, map_rot = transform_robot_to_map(robot_position, robot_rotation, obj_pos_robot, obj_rot_robot)
    """
    # Extract robot pose components
    robot_pos = robot_position
    robot_rot = robot_rotation

    # Robot position and orientation in map frame
    robot_x, robot_y, robot_z = robot_pos.x, robot_pos.y, robot_pos.z
    robot_yaw = robot_rot.z  # yaw is rotation around z-axis

    # Position in robot frame
    pos_x, pos_y, pos_z = position.x, position.y, position.z

    # Apply 2D transformation (rotation + translation) for x,y coordinates
    cos_yaw = np.cos(robot_yaw)
    sin_yaw = np.sin(robot_yaw)

    # Transform position from robot frame to map frame
    map_x = robot_x + cos_yaw * pos_x - sin_yaw * pos_y
    map_y = robot_y + sin_yaw * pos_x + cos_yaw * pos_y
    map_z = robot_z + pos_z  # Z translation (assume flat ground)

    # Transform rotation from robot frame to map frame
    rot_roll, rot_pitch, rot_yaw = rotation.x, rotation.y, rotation.z
    map_roll = robot_rot.x + rot_roll  # Add robot's roll
    map_pitch = robot_rot.y + rot_pitch  # Add robot's pitch
    map_yaw_rot = normalize_angle(robot_yaw + rot_yaw)  # Add robot's yaw and normalize

    transformed_position = Vector(map_x, map_y, map_z)
    transformed_rotation = Vector(map_roll, map_pitch, map_yaw_rot)

    return transformed_position, transformed_rotation

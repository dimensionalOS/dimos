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
from typing import Dict, Any, Optional, Tuple, List
from dimos.types.pose import Pose
from dimos.types.vector import Vector
import cv2


def parse_zed_pose(zed_pose_data: Dict[str, Any]) -> Optional[Pose]:
    """
    Parse ZED pose data dictionary into a Pose object.

    Args:
        zed_pose_data: Dictionary from ZEDCamera.get_pose() containing:
            - position: [x, y, z] in meters
            - rotation: [x, y, z, w] quaternion
            - euler_angles: [roll, pitch, yaw] in radians
            - valid: Whether pose is valid

    Returns:
        Pose object with position and rotation, or None if invalid
    """
    if not zed_pose_data or not zed_pose_data.get("valid", False):
        return None

    # Extract position
    position = zed_pose_data.get("position", [0, 0, 0])
    pos_vector = Vector(position[0], position[1], position[2])

    # Extract euler angles (roll, pitch, yaw)
    euler = zed_pose_data.get("euler_angles", [0, 0, 0])
    rot_vector = Vector(euler[0], euler[1], euler[2])  # roll, pitch, yaw

    return Pose(pos_vector, rot_vector)


def pose_to_transform_matrix(pose: Pose) -> np.ndarray:
    """
    Convert pose to 4x4 homogeneous transform matrix.

    Args:
        pose: Pose object with position and rotation (euler angles)

    Returns:
        4x4 transformation matrix
    """
    # Extract position
    tx, ty, tz = pose.pos.x, pose.pos.y, pose.pos.z

    # Extract euler angles
    roll, pitch, yaw = pose.rot.x, pose.rot.y, pose.rot.z

    # Create rotation matrices
    cos_roll, sin_roll = np.cos(roll), np.sin(roll)
    cos_pitch, sin_pitch = np.cos(pitch), np.sin(pitch)
    cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)

    # Roll (X), Pitch (Y), Yaw (Z) - ZYX convention
    R_x = np.array([[1, 0, 0], [0, cos_roll, -sin_roll], [0, sin_roll, cos_roll]])

    R_y = np.array([[cos_pitch, 0, sin_pitch], [0, 1, 0], [-sin_pitch, 0, cos_pitch]])

    R_z = np.array([[cos_yaw, -sin_yaw, 0], [sin_yaw, cos_yaw, 0], [0, 0, 1]])

    R = R_z @ R_y @ R_x

    # Create 4x4 transform
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [tx, ty, tz]

    return T


def transform_matrix_to_pose(T: np.ndarray) -> Pose:
    """
    Convert 4x4 transformation matrix to Pose object.

    Args:
        T: 4x4 transformation matrix

    Returns:
        Pose object with position and rotation (euler angles)
    """
    # Extract position
    pos = Vector(T[0, 3], T[1, 3], T[2, 3])

    # Extract rotation (euler angles from rotation matrix)
    R = T[:3, :3]
    roll = np.arctan2(R[2, 1], R[2, 2])
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))
    yaw = np.arctan2(R[1, 0], R[0, 0])

    rot = Vector(roll, pitch, yaw)

    return Pose(pos, rot)


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
    T_pose = pose_to_transform_matrix(pose)

    # Apply transform
    T_result = transform_matrix @ T_pose

    # Convert back to pose
    return transform_matrix_to_pose(T_result)


def optical_to_robot_convention(pose: Pose) -> Pose:
    """
    Convert pose from ZED camera convention to robot arm convention.

    ZED Camera Coordinates:
    - X: Right
    - Y: Down
    - Z: Forward (away from camera)

    Robot/ROS Coordinates:
    - X: Forward
    - Y: Left
    - Z: Up

    Args:
        pose: Pose in ZED camera convention

    Returns:
        Pose in robot arm convention
    """
    # Position transformation
    robot_x = pose.pos.z  # Forward = ZED Z
    robot_y = -pose.pos.x  # Left = -ZED X
    robot_z = -pose.pos.y  # Up = -ZED Y

    # Rotation transformation using rotation matrices
    # First, create rotation matrix from ZED Euler angles
    roll_zed, pitch_zed, yaw_zed = pose.rot.x, pose.rot.y, pose.rot.z

    # Create rotation matrix for ZED frame (ZYX convention)
    cr, sr = np.cos(roll_zed), np.sin(roll_zed)
    cp, sp = np.cos(pitch_zed), np.sin(pitch_zed)
    cy, sy = np.cos(yaw_zed), np.sin(yaw_zed)

    # Roll (X), Pitch (Y), Yaw (Z) - ZYX convention
    R_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])

    R_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])

    R_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])

    R_zed = R_z @ R_y @ R_x

    # Coordinate frame transformation matrix from ZED to Robot
    # X_robot = Z_zed, Y_robot = -X_zed, Z_robot = -Y_zed
    T_frame = np.array(
        [
            [0, 0, 1],  # X_robot = Z_zed
            [-1, 0, 0],  # Y_robot = -X_zed
            [0, -1, 0],
        ]
    )  # Z_robot = -Y_zed

    # Transform the rotation matrix
    R_robot = T_frame @ R_zed @ T_frame.T

    # Extract Euler angles from robot rotation matrix
    # Using ZYX convention for robot frame as well
    robot_roll = np.arctan2(R_robot[2, 1], R_robot[2, 2])
    robot_pitch = np.arctan2(-R_robot[2, 0], np.sqrt(R_robot[2, 1] ** 2 + R_robot[2, 2] ** 2))
    robot_yaw = np.arctan2(R_robot[1, 0], R_robot[0, 0])

    # Normalize angles to [-π, π]
    robot_roll = np.arctan2(np.sin(robot_roll), np.cos(robot_roll))
    robot_pitch = np.arctan2(np.sin(robot_pitch), np.cos(robot_pitch))
    robot_yaw = np.arctan2(np.sin(robot_yaw), np.cos(robot_yaw))

    return Pose(Vector(robot_x, robot_y, robot_z), Vector(robot_roll, robot_pitch, robot_yaw))


def robot_to_optical_convention(pose: Pose) -> Pose:
    """
    Convert pose from robot arm convention to ZED camera convention.
    This is the inverse of optical_to_robot_convention.

    Args:
        pose: Pose in robot arm convention

    Returns:
        Pose in ZED camera convention
    """
    # Position transformation (inverse)
    zed_x = -pose.pos.y  # Right = -Left
    zed_y = -pose.pos.z  # Down = -Up
    zed_z = pose.pos.x  # Forward = Forward

    # Rotation transformation using rotation matrices
    # First, create rotation matrix from Robot Euler angles
    roll_robot, pitch_robot, yaw_robot = pose.rot.x, pose.rot.y, pose.rot.z

    # Create rotation matrix for Robot frame (ZYX convention)
    cr, sr = np.cos(roll_robot), np.sin(roll_robot)
    cp, sp = np.cos(pitch_robot), np.sin(pitch_robot)
    cy, sy = np.cos(yaw_robot), np.sin(yaw_robot)

    # Roll (X), Pitch (Y), Yaw (Z) - ZYX convention
    R_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])

    R_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])

    R_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])

    R_robot = R_z @ R_y @ R_x

    # Coordinate frame transformation matrix from Robot to ZED (inverse of ZED to Robot)
    # This is the transpose of the forward transformation
    T_frame_inv = np.array(
        [
            [0, -1, 0],  # X_zed = -Y_robot
            [0, 0, -1],  # Y_zed = -Z_robot
            [1, 0, 0],
        ]
    )  # Z_zed = X_robot

    # Transform the rotation matrix
    R_zed = T_frame_inv @ R_robot @ T_frame_inv.T

    # Extract Euler angles from ZED rotation matrix
    # Using ZYX convention for ZED frame as well
    zed_roll = np.arctan2(R_zed[2, 1], R_zed[2, 2])
    zed_pitch = np.arctan2(-R_zed[2, 0], np.sqrt(R_zed[2, 1] ** 2 + R_zed[2, 2] ** 2))
    zed_yaw = np.arctan2(R_zed[1, 0], R_zed[0, 0])

    # Normalize angles
    zed_roll = np.arctan2(np.sin(zed_roll), np.cos(zed_roll))
    zed_pitch = np.arctan2(np.sin(zed_pitch), np.cos(zed_pitch))
    zed_yaw = np.arctan2(np.sin(zed_yaw), np.cos(zed_yaw))

    return Pose(Vector(zed_x, zed_y, zed_z), Vector(zed_roll, zed_pitch, zed_yaw))


def calculate_yaw_to_origin(position: Vector) -> float:
    """
    Calculate yaw angle to point away from origin (0,0,0)
    Assumes robot frame where X is forward and Y is left.

    Args:
        position: Current position in robot frame

    Returns:
        Yaw angle in radians to point away from origin
    """
    return np.arctan2(position.y, position.x)


def estimate_object_depth(
    depth_image: np.ndarray, segmentation_mask: Optional[np.ndarray], bbox: List[float]
) -> float:
    """
    Estimate object depth dimension using segmentation mask and depth data.
    Optimized for real-time performance.

    Args:
        depth_image: Depth image in meters
        segmentation_mask: Binary segmentation mask for the object
        bbox: Bounding box [x1, y1, x2, y2]

    Returns:
        Estimated object depth in meters
    """
    x1, y1, x2, y2 = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])

    # Quick bounds check
    if x2 <= x1 or y2 <= y1:
        return 0.05

    # Extract depth ROI once
    roi_depth = depth_image[y1:y2, x1:x2]

    if segmentation_mask is not None and segmentation_mask.size > 0:
        # Extract mask ROI efficiently
        mask_roi = (
            segmentation_mask[y1:y2, x1:x2]
            if segmentation_mask.shape != roi_depth.shape
            else segmentation_mask
        )

        # Fast mask application using boolean indexing
        valid_mask = mask_roi > 0
        if np.sum(valid_mask) > 10:  # Early exit if not enough points
            masked_depths = roi_depth[valid_mask]

            # Fast percentile calculation using numpy's optimized functions
            depth_90 = np.percentile(masked_depths, 90)
            depth_10 = np.percentile(masked_depths, 10)
            depth_range = depth_90 - depth_10

            # Clamp to reasonable bounds with single operation
            return np.clip(depth_range, 0.02, 0.5)

    # Fast fallback using area calculation
    bbox_area = (x2 - x1) * (y2 - y1)

    # Vectorized area-based estimation
    if bbox_area > 10000:
        return 0.15
    elif bbox_area > 5000:
        return 0.10
    else:
        return 0.05

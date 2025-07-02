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

from dimos.types.vector import Vector

logger = logging.getLogger(__name__)


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi] range"""
    return np.arctan2(np.sin(angle), np.cos(angle))


def distance_angle_to_goal_xy(distance: float, angle: float) -> Tuple[float, float]:
    """Convert distance and angle to goal x, y in robot frame"""
    return distance * np.cos(angle), distance * np.sin(angle)


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

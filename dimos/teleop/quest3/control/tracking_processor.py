#!/usr/bin/env python3
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

"""
VR controller tracking processor for Quest3 teleoperation.

Processes VR controller tracking data, applies coordinate transformations,
and returns robot-space poses and gripper values.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = setup_logger()

# Coordinate frame transformation from VR to robot
VR_TO_ROBOT_FRAME = np.array(
    [
        [0, 0, -1, 0],  # Robot X-axis = -VR Z-axis (flip forward/back)
        [-1, 0, 0, 0],  # Robot Y-axis = -VR X-axis (flip left/right)
        [0, 1, 0, 0],  # Robot Z-axis = +VR Y-axis (keep up direction)
        [0, 0, 0, 1],  # Homogeneous coordinate
    ],
    dtype=np.float32,
)


class TrackingProcessor:
    """Processes VR controller tracking data to robot-space poses.

    Handles:
    - VR to robot coordinate frame transformation
    - Controller orientation alignment
    - Safety constraints (Z position clamping) (optional)
    """

    def __init__(self) -> None:
        """Initialize tracking processor."""
        # Current poses (4x4 transformation matrices)
        self.left_wrist_pose = np.eye(4, dtype=np.float32)
        self.right_wrist_pose = np.eye(4, dtype=np.float32)

        # Current gripper values (0.0 = open, 1.0 = closed)
        self.left_gripper_value = 0.0
        self.right_gripper_value = 0.0

    def process_tracking_message(
        self, event: dict[str, Any]
    ) -> tuple[NDArray[np.float32], NDArray[np.float32], float, float] | None:
        """Process VR tracking message and return robot-space poses.

        Args:
            event: Dictionary with 'type', 'left', and 'right' controller data

        Returns:
            Tuple of (left_pose, right_pose, left_gripper, right_gripper) or None if invalid
        """
        tracking_type = event.get("type")
        if tracking_type != "controller":  # TODO: handle hand tracking type
            logger.debug("Ignoring non-controller tracking type: %s", tracking_type)
            return None

        # Process both controllers
        for side in ["left", "right"]:
            tracking_data = event.get(side)
            if tracking_data is not None:
                if not isinstance(tracking_data, dict):
                    logger.warning("Invalid tracking data format for %s", side)
                    continue

                self._process_controller(tracking_data, side)

        # Get controller poses
        left_pose = self.left_wrist_pose.copy()
        right_pose = self.right_wrist_pose.copy()

        return left_pose, right_pose, self.left_gripper_value, self.right_gripper_value

    def _process_controller(self, tracking_data: dict[str, Any], side: str) -> None:
        """Process single controller's tracking data.

        Args:
            tracking_data: Controller data with 'targetLocation' and 'trigger'
            side: 'left' or 'right'
        """
        # Process target location (pose)
        if "targetLocation" in tracking_data:
            self._process_target_location(tracking_data["targetLocation"], side)

        # Process gripper (trigger)
        if side == "left":
            self.left_gripper_value = self._extract_gripper_value(tracking_data)
        else:
            self.right_gripper_value = self._extract_gripper_value(tracking_data)

    def _process_target_location(self, target_location: list[float], side: str) -> None:
        """Process controller pose from VR space to robot space.

        Args:
            target_location: Flat 16-element transformation matrix (column-major)
            side: 'left' or 'right'
        """
        # Convert to 4x4 matrix
        target_matrix_flat = np.array(target_location, dtype=np.float32)
        if target_matrix_flat.size != 16:
            logger.error("Invalid targetLocation size: %d, expected 16", target_matrix_flat.size)
            return

        # Reshape and transpose (column-major from JS to row-major)
        target_matrix = target_matrix_flat.reshape(4, 4).T

        # Rotate controller 90° around Z-axis for gripper alignment
        direction = -1 if side == "right" else 1
        rotation = Rotation.from_euler("z", 90 * direction, degrees=True)
        target_matrix[:3, :3] = target_matrix[:3, :3] @ rotation.as_matrix()

        # Apply VR to robot frame transformation
        wrist_mat = VR_TO_ROBOT_FRAME @ target_matrix

        # Store the pose
        if side == "left":
            self.left_wrist_pose = wrist_mat
        else:
            self.right_wrist_pose = wrist_mat

    def _extract_gripper_value(self, tracking_data: dict[str, Any]) -> float:
        """Extract and validate gripper value from tracking data.

        Args:
            tracking_data: Controller data containing 'trigger'

        Returns:
            Gripper value clamped to [0.0, 1.0]
        """
        gripper_value = tracking_data.get("trigger", 0.0)
        try:
            gripper_value = float(gripper_value)
            return max(0.0, min(1.0, gripper_value))
        except (ValueError, TypeError):
            logger.warning("Invalid trigger value: %s", gripper_value)
            return 0.0

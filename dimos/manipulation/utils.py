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

"""Manipulation utility functions."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from numpy.typing import NDArray


def pose_from_xyzrpy(
    x: float,
    y: float,
    z: float,
    roll: float,
    pitch: float,
    yaw: float,
) -> NDArray[np.float64]:
    """Create a 4x4 homogeneous transformation matrix from position and Euler angles.

    Args:
        x: X position in meters
        y: Y position in meters
        z: Z position in meters
        roll: Rotation about X axis in radians
        pitch: Rotation about Y axis in radians
        yaw: Rotation about Z axis in radians

    Returns:
        4x4 homogeneous transformation matrix
    """
    # Rotation matrices
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    # Rotation matrix (ZYX Euler angles: yaw-pitch-roll)
    # R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    R = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )

    # Build 4x4 homogeneous transform
    pose = np.eye(4)
    pose[:3, :3] = R
    pose[:3, 3] = [x, y, z]

    return pose


def xyzrpy_from_pose(pose: NDArray[np.float64]) -> tuple[float, float, float, float, float, float]:
    """Extract position and Euler angles from a 4x4 homogeneous transformation matrix.

    Args:
        pose: 4x4 homogeneous transformation matrix

    Returns:
        Tuple of (x, y, z, roll, pitch, yaw)
    """
    x, y, z = pose[:3, 3]

    # Extract Euler angles from rotation matrix (ZYX convention)
    R = pose[:3, :3]

    # Handle gimbal lock
    if abs(R[2, 0]) >= 1.0 - 1e-6:
        # Gimbal lock: pitch = +/- 90 degrees
        yaw = 0.0
        if R[2, 0] < 0:
            pitch = np.pi / 2
            roll = np.arctan2(R[0, 1], R[0, 2])
        else:
            pitch = -np.pi / 2
            roll = np.arctan2(-R[0, 1], -R[0, 2])
    else:
        pitch = np.arcsin(-R[2, 0])
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])

    return float(x), float(y), float(z), float(roll), float(pitch), float(yaw)

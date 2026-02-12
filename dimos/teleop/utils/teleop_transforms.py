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

"""Teleop transform utilities for VR coordinate transforms."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from scipy.spatial.transform import Rotation as R  # type: ignore[import-untyped]

from dimos.msgs.geometry_msgs import PoseStamped
from dimos.utils.transform_utils import matrix_to_pose, pose_to_matrix

if TYPE_CHECKING:
    from numpy.typing import NDArray

# Coordinate frame transformation from VR (WebXR) to robot frame
# WebXR: X=right, Y=up, Z=back (towards user)
# Robot: X=forward, Y=left, Z=up
VR_TO_ROBOT_FRAME: NDArray[np.float64] = np.array(
    [
        [0, 0, -1, 0],  # Robot X = -VR Z (forward)
        [-1, 0, 0, 0],  # Robot Y = -VR X (left)
        [0, 1, 0, 0],  # Robot Z = +VR Y (up)
        [0, 0, 0, 1],
    ],
    dtype=np.float64,
)


def webxr_to_robot(
    pose_stamped: PoseStamped,
    is_left_controller: bool = True,
) -> PoseStamped:
    """Transform VR controller pose to robot coordinate frame.

    Args:
        pose_stamped: PoseStamped from VR controller in WebXR frame.
        is_left_controller: True for left controller (+90 deg Z rotation),
                           False for right controller (-90 deg Z rotation).

    Returns:
        PoseStamped in robot frame (preserves original ts and frame_id).
    """
    vr_matrix = pose_to_matrix(pose_stamped)

    # Apply controller alignment rotation
    # Left controller rotates +90 deg around Z, right rotates -90 deg
    direction = 1 if is_left_controller else -1
    z_rotation = R.from_euler("z", 90 * direction, degrees=True).as_matrix()
    vr_matrix[:3, :3] = vr_matrix[:3, :3] @ z_rotation

    # Apply VR to robot frame transformation
    robot_matrix = VR_TO_ROBOT_FRAME @ vr_matrix
    robot_pose = matrix_to_pose(robot_matrix)

    return PoseStamped(
        position=robot_pose.position,
        orientation=robot_pose.orientation,
        ts=pose_stamped.ts,
        frame_id=pose_stamped.frame_id,
    )


__all__ = ["VR_TO_ROBOT_FRAME", "webxr_to_robot"]

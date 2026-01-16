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

if TYPE_CHECKING:
    from dimos_lcm.geometry_msgs import Transform as LCMTransform
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


def transform_vr_to_robot(
    transform: LCMTransform,
    is_left_controller: bool = True,
) -> NDArray[np.float64]:
    """Transform VR controller pose to robot coordinate frame.

    Args:
        transform: LCM Transform from VR controller.
        is_left_controller: True for left controller (+90° Z rotation),
                           False for right controller (-90° Z rotation).

    Returns:
        4x4 transformation matrix in robot frame.
    """
    # Convert LCM Transform to 4x4 matrix
    t = transform.translation
    q = transform.rotation
    rot_matrix = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

    vr_matrix = np.eye(4, dtype=np.float64)
    vr_matrix[:3, :3] = rot_matrix
    vr_matrix[:3, 3] = [t.x, t.y, t.z]

    # Apply controller alignment rotation
    # Left controller rotates +90° around Z, right rotates -90°
    direction = 1 if is_left_controller else -1
    z_rotation = R.from_euler("z", 90 * direction, degrees=True).as_matrix()
    vr_matrix[:3, :3] = vr_matrix[:3, :3] @ z_rotation

    # Apply VR to robot frame transformation
    return VR_TO_ROBOT_FRAME @ vr_matrix

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

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3

if TYPE_CHECKING:
    from dimos_lcm.geometry_msgs import Transform as LCMTransform
    from numpy.typing import NDArray

    from dimos.msgs.geometry_msgs.Pose import Pose

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


def pose_to_pose_stamped(
    delta_pose: Pose,
    initial_robot_pose: Pose | None = None,
    frame_id: str = "None",
    ts: float | None = None,
) -> PoseStamped:
    """Convert delta pose to PoseStamped (target = initial + delta)."""
    if initial_robot_pose is not None:
        target_pose = initial_robot_pose + delta_pose
    else:
        target_pose = delta_pose

    return PoseStamped(
        ts=ts if ts is not None else 0.0,
        frame_id=frame_id,
        position=target_pose.position,
        orientation=target_pose.orientation,
    )


def pose_to_twist_stamped(
    delta_pose: Pose,
    linear_scale: float = 1.0,
    angular_scale: float = 1.0,
    max_linear: float = 0.5,
    max_angular: float = 1.0,
    frame_id: str = "",
    ts: float | None = None,
) -> TwistStamped:
    """Convert delta pose to TwistStamped velocity command."""

    def clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    linear = Vector3(
        clamp(delta_pose.position.x * linear_scale, max_linear),
        clamp(delta_pose.position.y * linear_scale, max_linear),
        clamp(delta_pose.position.z * linear_scale, max_linear),
    )

    euler = delta_pose.orientation.to_euler()
    angular = Vector3(
        clamp(euler.x * angular_scale, max_angular),
        clamp(euler.y * angular_scale, max_angular),
        clamp(euler.z * angular_scale, max_angular),
    )

    return TwistStamped(
        ts=ts if ts is not None else 0.0,
        frame_id=frame_id,
        linear=linear,
        angular=angular,
    )

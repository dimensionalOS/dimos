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

"""Teleop transform utilities for converting delta poses to robot commands."""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Any

import numpy as np
from scipy.spatial.transform import Rotation as R  # type: ignore[import-untyped]

from dimos.msgs.geometry_msgs import Pose, PoseStamped, Twist, Vector3
from dimos.msgs.std_msgs import Bool
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos_lcm.geometry_msgs import Transform as LCMTransform
    from numpy.typing import NDArray

logger = setup_logger()

# Index mapping: output type → available indices
OUTPUT_TYPE_INDICES: dict[type, list[int]] = {
    PoseStamped: [0, 1],
    Twist: [2, 3],
}

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


def compute_active_indices(output_types: list[type]) -> list[int]:
    """Compute active indices from output types.
    Example:
        [PoseStamped, Twist] → [0, 2]
        [Twist, PoseStamped] → [2, 0]
        [PoseStamped, PoseStamped] → [0, 1]
        [Twist, Twist] → [2, 3]
    """
    indices: list[int] = []
    used_indices: set[int] = set()

    for output_type in output_types:
        available = OUTPUT_TYPE_INDICES.get(output_type, [])
        for idx in available:
            if idx not in used_indices:
                indices.append(idx)
                used_indices.add(idx)
                break
        else:
            raise ValueError(f"No available index for output type {output_type.__name__}")

    return indices


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


def transform_delta(
    delta_pose: Pose,
    trigger_value: float,
    output_type: type,
    initial_robot_pose: Pose | None = None,
    linear_scale: float = 1.0,
    angular_scale: float = 1.0,
    max_linear_velocity: float = 0.5,
    max_angular_velocity: float = 1.0,
    gripper_threshold: float = 0.5,
) -> tuple[PoseStamped | Twist | None, Bool | None]:
    """Transform delta pose to robot command based on output type.

    Args:
        delta_pose: Delta from teleop device (current - initial).
        trigger_value: Trigger value (0.0-1.0).
        output_type: PoseStamped or Twist.
        initial_robot_pose: Initial robot pose (required for PoseStamped).
        linear_scale: Scale factor for linear velocity (Twist only).
        angular_scale: Scale factor for angular velocity (Twist only).
        max_linear_velocity: Max linear velocity clamp (Twist only).
        max_angular_velocity: Max angular velocity clamp (Twist only).
        gripper_threshold: Trigger threshold for gripper open/close.

    Returns:
        Tuple of (command, trigger_bool).
    """
    trigger_bool = Bool(data=trigger_value > gripper_threshold)

    if output_type == PoseStamped:
        return _to_pose_stamped(delta_pose, initial_robot_pose), trigger_bool
    elif output_type == Twist:
        return _to_twist(
            delta_pose, linear_scale, angular_scale, max_linear_velocity, max_angular_velocity
        ), trigger_bool
    else:
        logger.warning(f"Unknown output type: {output_type}")
        return None, trigger_bool


def _to_pose_stamped(delta_pose: Pose, initial_robot_pose: Pose | None) -> PoseStamped:
    """Convert delta pose to PoseStamped (target = initial + delta)."""
    if initial_robot_pose is not None:
        target_pose = initial_robot_pose + delta_pose
    else:
        target_pose = delta_pose
    return PoseStamped(
        ts=time.time(),
        frame_id="teleop_target",
        position=target_pose.position,
        orientation=target_pose.orientation,
    )


def _to_twist(
    delta_pose: Pose,
    linear_scale: float,
    angular_scale: float,
    max_linear: float,
    max_angular: float,
) -> Twist:
    """Convert delta pose to Twist velocity command."""

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
    return Twist(linear=linear, angular=angular)


def parse_pose_from_dict(  # type: ignore[type-arg]
    output_type: type, data: dict[str, Any] | None = None
) -> Pose | None:
    """Parse a Pose from a dictionary based on output type.

    Args:
        output_type: PoseStamped or Twist.
        data: Dict with position and orientation keys.
            {"position": {"x": float, "y": float, "z": float},
             "orientation": {"x": float, "y": float, "z": float, "w": float}}

    Returns:
        Parsed Pose for PoseStamped, None for Twist.
    """
    if output_type == Twist:
        return None

    if output_type == PoseStamped:
        if data is None:
            return Pose()

        from dimos.msgs.geometry_msgs import Quaternion

        pos = data.get("position", {})
        ori = data.get("orientation", {})

        pose = Pose(
            position=Vector3(
                pos.get("x", 0.0),
                pos.get("y", 0.0),
                pos.get("z", 0.0),
            ),
        )
        if ori:
            pose.orientation = Quaternion(
                ori.get("x", 0.0),
                ori.get("y", 0.0),
                ori.get("z", 0.0),
                ori.get("w", 1.0),
            )
        return pose

    return None

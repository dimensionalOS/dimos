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

"""Deterministic WebXR body-skeleton conversion for SONIC SMPL mode."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Final

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.control.tasks.g1_sonic_wbc_task.sonic_pipeline import (
    DEFAULT_ANGLES_ONNX,
    NUM_JOINTS,
    WRIST_ONNX_INDICES,
)
from dimos.teleop.webxr.body_tracking import BodyTrackingSnapshot

# Standard SMPL 24-joint order. The PICO body role order follows the same
# skeleton; the WebXR names are the device-neutral boundary exposed by DimOS.
SMPL_WEBXR_JOINTS: Final[tuple[str, ...]] = (
    "hips",
    "left-upper-leg",
    "right-upper-leg",
    "spine-lower",
    "left-lower-leg",
    "right-lower-leg",
    "spine-middle",
    "left-foot-ankle",
    "right-foot-ankle",
    "spine-upper",
    "left-foot-ball",
    "right-foot-ball",
    "neck",
    "left-shoulder",
    "right-shoulder",
    "head",
    "left-arm-upper",
    "right-arm-upper",
    "left-arm-lower",
    "right-arm-lower",
    "left-hand-wrist",
    "right-hand-wrist",
    "left-hand-palm",
    "right-hand-palm",
)

SMPL_PARENTS: Final[tuple[int, ...]] = (
    -1,
    0,
    0,
    0,
    1,
    2,
    3,
    4,
    5,
    6,
    7,
    8,
    9,
    9,
    9,
    12,
    13,
    14,
    16,
    17,
    18,
    19,
    20,
    21,
)

# Per-role rotation from the WebXR joint frame to the corresponding SMPL
# joint frame, aligned with SMPL_WEBXR_JOINTS. PICO's standardized body roles
# currently expose the same rest axes, so every correction is identity. Keep
# the table explicit: a headset/runtime-specific axis change belongs here,
# rather than in the live retargeting logic.
SMPL_REST_BASIS_XYZW: Final[tuple[tuple[float, float, float, float], ...]] = (
    (0.0, 0.0, 0.0, 1.0),  # hips
    (0.0, 0.0, 0.0, 1.0),  # left-upper-leg
    (0.0, 0.0, 0.0, 1.0),  # right-upper-leg
    (0.0, 0.0, 0.0, 1.0),  # spine-lower
    (0.0, 0.0, 0.0, 1.0),  # left-lower-leg
    (0.0, 0.0, 0.0, 1.0),  # right-lower-leg
    (0.0, 0.0, 0.0, 1.0),  # spine-middle
    (0.0, 0.0, 0.0, 1.0),  # left-foot-ankle
    (0.0, 0.0, 0.0, 1.0),  # right-foot-ankle
    (0.0, 0.0, 0.0, 1.0),  # spine-upper
    (0.0, 0.0, 0.0, 1.0),  # left-foot-ball
    (0.0, 0.0, 0.0, 1.0),  # right-foot-ball
    (0.0, 0.0, 0.0, 1.0),  # neck
    (0.0, 0.0, 0.0, 1.0),  # left-shoulder
    (0.0, 0.0, 0.0, 1.0),  # right-shoulder
    (0.0, 0.0, 0.0, 1.0),  # head
    (0.0, 0.0, 0.0, 1.0),  # left-arm-upper
    (0.0, 0.0, 0.0, 1.0),  # right-arm-upper
    (0.0, 0.0, 0.0, 1.0),  # left-arm-lower
    (0.0, 0.0, 0.0, 1.0),  # right-arm-lower
    (0.0, 0.0, 0.0, 1.0),  # left-hand-wrist
    (0.0, 0.0, 0.0, 1.0),  # right-hand-wrist
    (0.0, 0.0, 0.0, 1.0),  # left-hand-palm
    (0.0, 0.0, 0.0, 1.0),  # right-hand-palm
)

# WebXR: +X right, +Y up, -Z forward. SONIC: +X forward, +Y left, +Z up.
WEBXR_TO_SONIC: Final[NDArray[np.float64]] = np.array(
    [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
    dtype=np.float64,
)

_WRIST_LIMITS: Final[NDArray[np.float64]] = np.array([1.972, 1.615, 1.615], dtype=np.float64)


class IncompleteBodyPoseError(ValueError):
    """A WebXR snapshot cannot produce a complete SONIC reference."""


@dataclass(frozen=True)
class RetargetedSonicFrame:
    """One packed-message-equivalent SONIC protocol-v3 frame."""

    fields: dict[str, NDArray[Any]]


class WebXRSonicRetargeter:
    """Convert complete WebXR snapshots into root-local SONIC SMPL frames."""

    def __init__(self) -> None:
        self._previous_joint_pos: NDArray[np.float32] | None = None
        self._previous_time: float | None = None

    @staticmethod
    def missing_joints(snapshot: BodyTrackingSnapshot) -> tuple[str, ...]:
        joints = snapshot.joints
        if joints is None:
            return SMPL_WEBXR_JOINTS
        return tuple(name for name in SMPL_WEBXR_JOINTS if name not in joints)

    @classmethod
    def is_complete(cls, snapshot: BodyTrackingSnapshot) -> bool:
        return not cls.missing_joints(snapshot)

    def reset(self) -> None:
        self._previous_joint_pos = None
        self._previous_time = None

    def retarget(
        self,
        snapshot: BodyTrackingSnapshot,
        *,
        frame_index: int,
        t_now: float,
    ) -> RetargetedSonicFrame:
        missing = self.missing_joints(snapshot)
        if missing:
            raise IncompleteBodyPoseError(f"missing WebXR body joints: {', '.join(missing)}")
        assert snapshot.joints is not None

        positions = np.empty((24, 3), dtype=np.float64)
        world_rotations: list[Rotation] = []
        for index, name in enumerate(SMPL_WEBXR_JOINTS):
            pose = snapshot.joints[name]
            position = WEBXR_TO_SONIC @ np.asarray(pose.position, dtype=np.float64)
            quaternion = np.asarray(pose.orientation, dtype=np.float64)
            norm = float(np.linalg.norm(quaternion))
            if not np.isfinite(position).all() or not np.isfinite(quaternion).all() or norm < 1e-8:
                raise IncompleteBodyPoseError(f"invalid WebXR body pose for {name!r}")
            quaternion /= norm
            webxr_rotation = Rotation.from_quat(quaternion)
            sonic_matrix = (
                WEBXR_TO_SONIC
                @ webxr_rotation.as_matrix()
                @ WEBXR_TO_SONIC.T
                @ Rotation.from_quat(SMPL_REST_BASIS_XYZW[index]).as_matrix()
            )
            positions[index] = position
            world_rotations.append(Rotation.from_matrix(sonic_matrix))

        root_position = positions[0].copy()
        root_rotation = world_rotations[0]
        root_matrix = root_rotation.as_matrix()
        heading = np.arctan2(root_matrix[1, 0], root_matrix[0, 0])
        heading_inverse = Rotation.from_euler("z", -heading)
        root_local_positions = heading_inverse.apply(positions - root_position)

        # SONIC's native message carries 21 SMPL body rotations: all joints
        # after the root through the wrists, excluding the two terminal hands.
        body_pose = np.empty((21, 3), dtype=np.float64)
        local_rotations: list[Rotation] = [Rotation.identity()]
        for index in range(1, 24):
            parent_rotation = world_rotations[SMPL_PARENTS[index]]
            local_rotation = parent_rotation.inv() * world_rotations[index]
            local_rotations.append(local_rotation)
            if index <= 21:
                body_pose[index - 1] = local_rotation.as_rotvec()

        left_wrist = np.clip(local_rotations[20].as_euler("XYZ"), -_WRIST_LIMITS, _WRIST_LIMITS)
        right_wrist = np.clip(local_rotations[21].as_euler("XYZ"), -_WRIST_LIMITS, _WRIST_LIMITS)
        joint_pos = DEFAULT_ANGLES_ONNX.astype(np.float32, copy=True)
        joint_pos[WRIST_ONNX_INDICES] = np.asarray(
            [
                left_wrist[0],
                right_wrist[0],
                left_wrist[1],
                right_wrist[1],
                left_wrist[2],
                right_wrist[2],
            ],
            dtype=np.float32,
        )
        joint_vel = np.zeros(NUM_JOINTS, dtype=np.float32)
        if self._previous_joint_pos is not None and self._previous_time is not None:
            dt = t_now - self._previous_time
            if dt > 1e-6:
                joint_vel[WRIST_ONNX_INDICES] = (
                    joint_pos[WRIST_ONNX_INDICES] - self._previous_joint_pos[WRIST_ONNX_INDICES]
                ) / dt
        self._previous_joint_pos = joint_pos.copy()
        self._previous_time = t_now

        root_quaternion = root_rotation.as_quat()
        fields: dict[str, NDArray[Any]] = {
            "frame_index": np.array([frame_index], dtype=np.int64),
            "joint_pos": joint_pos.reshape(1, NUM_JOINTS),
            "joint_vel": joint_vel.reshape(1, NUM_JOINTS),
            "body_quat_w": np.array([[root_quaternion[3], *root_quaternion[:3]]], dtype=np.float32),
            "smpl_joints": root_local_positions.astype(np.float32).reshape(1, 24, 3),
            "smpl_pose": body_pose.astype(np.float32).reshape(1, 21, 3),
        }
        return RetargetedSonicFrame(fields=fields)

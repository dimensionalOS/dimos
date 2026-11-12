# Copyright 2026 Dimensional Inc.
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

"""Inference contract of the frozen coordinated R1 PolicyIK extension.

Provenance: Tinaliu0123/PolicyIK 22f3a5c and Humanoid Control Lab upper003/004.
This is the two-hand extension, not the published single-arm task.
"""

from dataclasses import dataclass
import math
from typing import cast

import numpy as np
from numpy.typing import NDArray

from dimos.control.policyik.actor import PolicyIKActor
from dimos.control.policyik.kinematics import ToolKinematics

FloatArray = NDArray[np.float32]
DT_NS = 2_083_333
CONTROL_STEPS = 16
PERIOD_NS = DT_NS * CONTROL_STEPS
POSITION_TOLERANCE = 0.001
ORIENTATION_TOLERANCE = math.radians(0.2)


def conjugate(q: FloatArray) -> FloatArray:
    return np.concatenate((q[..., :1], -q[..., 1:]), axis=-1)


def multiply(a: FloatArray, b: FloatArray) -> FloatArray:
    aw, av, bw, bv = a[..., :1], a[..., 1:], b[..., :1], b[..., 1:]
    return np.concatenate(
        (aw * bw - np.sum(av * bv, axis=-1, keepdims=True), aw * bv + bw * av + np.cross(av, bv)),
        axis=-1,
    )


def rotate(q: FloatArray, v: FloatArray) -> FloatArray:
    uv = 2 * np.cross(q[..., 1:], v)
    return v + q[..., :1] * uv + np.cross(q[..., 1:], uv)


def rotation_error(current: FloatArray, goal: FloatArray) -> FloatArray:
    relative = multiply(conjugate(current), goal)
    relative /= np.maximum(np.linalg.norm(relative, axis=-1, keepdims=True), 1e-12)
    relative = np.where(relative[..., :1] < 0, -relative, relative)
    length = np.linalg.norm(relative[..., 1:], axis=-1, keepdims=True)
    angle = 2 * np.arctan2(length, np.maximum(relative[..., :1], 0))
    return cast(
        "FloatArray",
        relative[..., 1:] * np.where(length > 1e-8, angle / np.maximum(length, 1e-12), 2),
    )


def pose_features(position: FloatArray, quaternion: FloatArray, goals: FloatArray) -> FloatArray:
    points = np.eye(3, dtype=np.float32) * 0.30
    offsets = (
        goals[:, None, :3]
        + rotate(goals[:, None, 3:], points)
        - position[:, None, :]
        - rotate(quaternion[:, None, :], points)
    )
    dp = rotate(conjugate(quaternion), goals[:, :3] - position)
    dr = rotation_error(quaternion, goals[:, 3:])
    return np.concatenate(
        (
            goals[:, :3],
            offsets.reshape(2, 9),
            np.clip(dp / 0.01, -1, 1),
            np.clip(dr / math.radians(20), -1, 1),
            np.clip(dp / 0.0005, -1, 1),
            np.clip(dr / math.radians(0.25), -1, 1),
        ),
        axis=-1,
    )


@dataclass(frozen=True)
class ControlResult:
    reference: FloatArray
    action: FloatArray
    observation: FloatArray
    position_error: FloatArray
    orientation_error: FloatArray
    tool_position: FloatArray
    tool_quaternion: FloatArray


class CoordinatedPolicyIK:
    """One caller owns this state; reset only on a new physical episode.

    Changing goals resets progress history, but preserves the joint reference
    accumulator and previous action. Neither is replaced by measured q each tick.
    """

    def __init__(self, actor: PolicyIKActor, kinematics: ToolKinematics) -> None:
        self.actor = actor
        self.kinematics = kinematics
        self.reset()

    def reset(self) -> None:
        self.reference: FloatArray | None = None
        self.previous_action = np.zeros(18, dtype=np.float32)
        self.previous_goals: FloatArray | None = None
        self.best = np.zeros(2, dtype=np.float32)
        self.step = 0

    def compute(self, q: FloatArray, qd: FloatArray, goals: FloatArray) -> ControlResult:
        position, quaternion = self.kinematics.poses(q)
        return self.compute_with_poses(q, qd, goals, position, quaternion)

    def compute_with_poses(
        self,
        q: FloatArray,
        qd: FloatArray,
        goals: FloatArray,
        position: FloatArray,
        quaternion: FloatArray,
    ) -> ControlResult:
        """Explicit pose entry point permits parity checks against native traces."""
        if q.shape != (18,) or qd.shape != (18,) or goals.shape != (2, 7):
            raise ValueError("Expected 18 measured joints and two xyz/wxyz goals")
        if not all(np.isfinite(x).all() for x in (q, qd, goals, position, quaternion)):
            raise ValueError("Nonfinite policy input")
        if not np.allclose(np.linalg.norm(goals[:, 3:], axis=-1), 1, atol=1e-5):
            raise ValueError("Goal quaternions must be normalized wxyz")
        ep = np.linalg.norm(goals[:, :3] - position, axis=-1)
        er = np.linalg.norm(rotation_error(quaternion, goals[:, 3:]), axis=-1)
        rho = np.maximum(ep / POSITION_TOLERANCE, er / ORIENTATION_TOLERANCE)
        changed = (
            np.ones(2, dtype=bool)
            if self.previous_goals is None
            else np.any(goals != self.previous_goals, axis=-1)
        )
        # Native observation history resets while episode_length_buf <= 1.
        before = np.where(changed | (self.step <= 1), rho, self.best)
        history = np.stack(
            (np.clip(before / 20, 0, 1), np.clip((rho - before) / 20, 0, 1)), axis=-1
        )
        self.best = np.minimum(before, rho)
        self.previous_goals = goals.copy()
        command = np.zeros_like(q) if self.reference is None else self.reference - q
        observation = np.concatenate(
            (
                q,
                qd,
                np.concatenate(
                    (pose_features(position, quaternion, goals), history), axis=-1
                ).ravel(),
                command,
                self.previous_action,
            )
        ).astype(np.float32)
        action = self.actor(observation)
        gp = 0.1 + 0.9 * np.clip((ep - 0.0005) / (0.00931 - 0.0005), 0, 1)
        gr = 0.1 + 0.9 * np.clip((er - math.radians(0.415)) / math.radians(2.328 - 0.415), 0, 1)
        gain = np.maximum(gp, gr).max()
        anchor = q if self.reference is None else self.reference
        self.reference = np.clip(
            anchor + np.float32(0.0625) * gain * action,
            self.kinematics.limits[:, 0],
            self.kinematics.limits[:, 1],
        ).astype(np.float32)
        self.previous_action = action.copy()
        self.step += 1
        return ControlResult(
            self.reference.copy(), action, observation, ep, er, position, quaternion
        )

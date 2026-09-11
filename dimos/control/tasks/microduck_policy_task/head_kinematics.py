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

"""MicroDuck head FK and gaze IK in the upstream trunk/cv2 frames.

This is a small Python port of ``microduck/kinematics/src/head.rs``. The rest
transforms come from the pinned alpha MJCF. Policy-space limits are narrower
than mechanical travel, so gaze results are clamped to the trained envelope.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import NDArray

_Vec3 = NDArray[np.float64]
_Quat = NDArray[np.float64]

# rest position, rest quaternion (wxyz), then a rotation about local +z
_HEAD_CHAIN: tuple[tuple[tuple[float, ...], tuple[float, ...]], ...] = (
    ((0.026, 0.0145, 0.0324215), (0.0, 0.0, 0.707107, -0.707107)),
    ((0.0, -0.05, 0.0), (0.0, 1.0, 0.0, 0.0)),
    ((0.0, 0.0186931, -0.0145), (0.0, 0.0, -0.707107, -0.707107)),
    ((-0.0179, 0.0, 0.0145), (0.707107, 0.0, -0.707107, 0.0)),
)
_CAMERA_POS = np.asarray((0.0155, -9.13778e-05, -0.0733), dtype=np.float64)
_CAMERA_QUAT = np.asarray((0.707107, 0.0, 0.707107, 0.0), dtype=np.float64)
_SITE_TO_CV2 = np.asarray((0.5, -0.5, 0.5, -0.5), dtype=np.float64)

# neck_pitch, head_pitch, head_yaw, head_roll command envelopes
HEAD_COMMAND_LOWER = np.asarray((-1.10, -1.10, -1.40, -0.31), dtype=np.float64)
HEAD_COMMAND_UPPER = np.asarray((1.10, 1.10, 1.40, 0.31), dtype=np.float64)


@dataclass(frozen=True)
class Gaze:
    joints: tuple[float, float, float, float]
    clamped: bool


def _quat_normalized(q: _Quat) -> _Quat:
    norm = float(np.linalg.norm(q))
    return np.asarray((1.0, 0.0, 0.0, 0.0), dtype=np.float64) if norm < 1e-12 else q / norm


def _quat_mul(a: _Quat, b: _Quat) -> _Quat:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.asarray(
        (
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ),
        dtype=np.float64,
    )


def _quat_rotate(q: _Quat, vector: _Vec3) -> _Vec3:
    q = _quat_normalized(q)
    xyz = q[1:]
    t = 2.0 * np.cross(xyz, vector)
    return np.asarray(vector + q[0] * t + np.cross(xyz, t), dtype=np.float64)


def _z_rotation(angle: float) -> _Quat:
    sine, cosine = math.sin(0.5 * angle), math.cos(0.5 * angle)
    return np.asarray((cosine, 0.0, 0.0, sine), dtype=np.float64)


def camera_in_trunk_cv2(joints: tuple[float, float, float, float]) -> tuple[_Vec3, _Quat]:
    """Return camera position and cv2-axis quaternion in the trunk frame."""

    position = np.zeros(3, dtype=np.float64)
    quaternion = np.asarray((1.0, 0.0, 0.0, 0.0), dtype=np.float64)
    for angle, (rest_position, rest_quaternion) in zip(joints, _HEAD_CHAIN, strict=True):
        position += _quat_rotate(quaternion, np.asarray(rest_position, dtype=np.float64))
        quaternion = _quat_mul(quaternion, _quat_normalized(np.asarray(rest_quaternion)))
        quaternion = _quat_mul(quaternion, _z_rotation(angle))

    position += _quat_rotate(quaternion, _CAMERA_POS)
    quaternion = _quat_mul(quaternion, _quat_normalized(_CAMERA_QUAT))
    quaternion = _quat_mul(quaternion, _SITE_TO_CV2)
    return position, _quat_normalized(quaternion)


def look_at(target_in_trunk: tuple[float, float, float], neck_pitch: float = 0.0) -> Gaze:
    """Point the camera toward a trunk-frame target with damped 2-DOF IK."""

    target = np.asarray(target_in_trunk, dtype=np.float64)
    joints = np.asarray((neck_pitch, 0.0, 0.0, 0.0), dtype=np.float64)
    joints = np.clip(joints, HEAD_COMMAND_LOWER, HEAD_COMMAND_UPPER)

    tolerance = 1e-4
    step_h = 1e-5
    damping = 1e-3
    max_step = 0.7

    def pointing_error(values: NDArray[np.float64]) -> NDArray[np.float64]:
        position, quaternion = camera_in_trunk_cv2(tuple(float(v) for v in values))  # type: ignore[arg-type]
        delta = target - position
        camera_delta = _quat_rotate(
            np.asarray((quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3])),
            delta,
        )
        flat = math.hypot(float(camera_delta[0]), float(camera_delta[2]))
        return np.asarray(
            (
                math.atan2(float(camera_delta[0]), float(camera_delta[2])),
                math.atan2(float(camera_delta[1]), flat),
            ),
            dtype=np.float64,
        )

    residual = math.inf
    for _ in range(30):
        error = pointing_error(joints)
        residual = float(np.max(np.abs(error)))
        if residual < tolerance:
            break

        jacobian = np.empty((2, 2), dtype=np.float64)
        for column, joint_index in enumerate((1, 2)):
            probe = joints.copy()
            probe[joint_index] += step_h
            jacobian[:, column] = (pointing_error(probe) - error) / step_h

        lhs = jacobian.T @ jacobian + damping * np.eye(2)
        rhs = -(jacobian.T @ error)
        try:
            step = np.linalg.solve(lhs, rhs)
        except np.linalg.LinAlgError:
            break
        norm = float(np.linalg.norm(step))
        if norm > max_step:
            step *= max_step / norm
        joints[1:3] += step
        joints = np.clip(joints, HEAD_COMMAND_LOWER, HEAD_COMMAND_UPPER)

    # Re-evaluate after the final update; upstream reports whether the answer
    # still misses, regardless of whether travel or geometry caused the miss.
    residual = float(np.max(np.abs(pointing_error(joints))))
    return Gaze(
        joints=tuple(float(value) for value in joints),  # type: ignore[arg-type]
        clamped=residual >= tolerance,
    )

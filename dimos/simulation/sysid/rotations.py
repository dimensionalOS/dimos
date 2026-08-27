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

"""Quaternion/matrix helpers shared by ingest, replay and the backend.

Ported verbatim from the identified instrument: replays are compared against
numbers produced by exactly these operations, so a library swap here would
silently change every published residual.
"""

from __future__ import annotations

import numpy as np


def quat_to_mat(q: np.ndarray) -> np.ndarray:
    """wxyz quaternion(s) to rotation matrix/matrices; normalises first."""
    q = np.asarray(q, float)
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    m = np.empty((*q.shape[:-1], 3, 3))
    m[..., 0, 0] = 1 - 2 * (y * y + z * z)
    m[..., 0, 1] = 2 * (x * y - w * z)
    m[..., 0, 2] = 2 * (x * z + w * y)
    m[..., 1, 0] = 2 * (x * y + w * z)
    m[..., 1, 1] = 1 - 2 * (x * x + z * z)
    m[..., 1, 2] = 2 * (y * z - w * x)
    m[..., 2, 0] = 2 * (x * z - w * y)
    m[..., 2, 1] = 2 * (y * z + w * x)
    m[..., 2, 2] = 1 - 2 * (x * x + y * y)
    return m


def mat_to_quat(m: np.ndarray) -> np.ndarray:
    """One rotation matrix to a wxyz quaternion (Shepperd's method)."""
    t = np.trace(m)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        return np.array(
            [0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
        )
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(m[i, i] - m[j, j] - m[k, k] + 1.0) * 2
    q = np.empty(4)
    q[0] = (m[k, j] - m[j, k]) / s
    q[i + 1] = 0.25 * s
    q[j + 1] = (m[j, i] + m[i, j]) / s
    q[k + 1] = (m[k, i] + m[i, k]) / s
    return q


def strip_yaw(rot: np.ndarray) -> np.ndarray:
    """Keep only the gravity-referenced part of an attitude.

    The IMU's roll and pitch are absolute; its yaw drifts and the room's yaw is
    arbitrary on flat ground, so measured attitudes enter the sim yaw-zeroed.
    """
    fwd = rot[:, 0].copy()
    yaw = np.arctan2(fwd[1], fwd[0])
    cz, sz = np.cos(-yaw), np.sin(-yaw)
    rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    out: np.ndarray = rz @ rot
    return out


def rotation_angle(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Geodesic angle (rad) between batched rotation matrices ``a`` and ``b``."""
    rel = np.einsum("nij,nkj->nik", a, b)
    cos = np.clip((np.trace(rel, axis1=1, axis2=2) - 1) / 2, -1, 1)
    out: np.ndarray = np.arccos(cos)
    return out


def yaw_of(quat: np.ndarray) -> np.ndarray:
    """Heading angle from (n, 4) wxyz quaternions."""
    w, x, y, z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
    yaw: np.ndarray = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return yaw


def pitch_roll_of(quat: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Pitch and roll from (n, 4) wxyz quaternions, ZYX convention.

    Only their *std* is comparable sim-to-real: a constant error in the room
    calibration or the mount shifts the mean but not the spread. Unlike
    anything derived from position, these are immune to the tracker
    translation — rotation carries no lever arm.
    """
    w, x, y, z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
    pitch = -np.arcsin(np.clip(2 * (x * z - w * y), -1.0, 1.0))
    roll = np.arctan2(2 * (y * z + w * x), 1 - 2 * (x * x + y * y))
    return pitch, roll


def yaw_anchor(sim_rot: np.ndarray, real_rot: np.ndarray) -> np.ndarray:
    """The pure-yaw rotation taking a recorded pose into the sim's world.

    Room and sim share gravity, so a ghost anchor must never carry pitch or
    roll: with ``sim @ real.T`` the instruments' few degrees of attitude
    disagreement tilt the WORLD, and the ghost climbs or sinks along its path.
    """
    d = float(np.arctan2(sim_rot[1, 0], sim_rot[0, 0]) - np.arctan2(real_rot[1, 0], real_rot[0, 0]))
    c, s_ = np.cos(d), np.sin(d)
    return np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])

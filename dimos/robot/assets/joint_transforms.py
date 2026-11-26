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

"""Where each joint of a robot puts its child link, for every joint at once."""

from __future__ import annotations

from collections.abc import Mapping

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.robot.assets.model import JointDescription

_POSABLE = ("fixed", "revolute", "continuous", "prismatic")


def joint_position(joint: JointDescription, positions: Mapping[str, float]) -> float:
    """The position ``joint`` sits at, following its mimic leader; a joint not named sits at zero."""
    if joint.mimic is not None:
        leader, multiplier, offset = joint.mimic
        return multiplier * positions.get(leader, 0.0) + offset
    return positions.get(joint.name, 0.0)


class JointTransforms:
    """Fixed and single-DOF joints as arrays, so every parent -> child transform is worked out in one step."""

    def __init__(self, joints: tuple[JointDescription, ...]) -> None:
        unknown = sorted({joint.type for joint in joints if joint.type not in _POSABLE})
        if unknown:
            raise ValueError(f"joints of type {unknown} have no single position")
        self.joints = joints
        self.links = [(joint.parent_link, joint.child_link) for joint in joints]
        origins = np.array([joint.origin() for joint in joints]).reshape(-1, 4, 4)
        self._origin_rotation = origins[:, :3, :3]
        self._origin_translation = origins[:, :3, 3]
        self._origin_quaternion = _quaternions(self._origin_rotation)
        axes = np.array([joint.axis for joint in joints], dtype=np.float64).reshape(-1, 3)
        self._axis = axes / np.linalg.norm(axes, axis=1, keepdims=True)
        self._turns = np.array([j.type in ("revolute", "continuous") for j in joints], dtype=bool)
        self._slides = np.array([j.type == "prismatic" for j in joints], dtype=bool)

    def at(self, positions: Mapping[str, float]) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Each joint's translation and x, y, z, w quaternion (w >= 0) at ``positions``; joints not named sit at zero."""
        at = np.array([joint_position(joint, positions) for joint in self.joints], dtype=np.float64)
        half = np.where(self._turns, at, 0.0) / 2.0
        motion = np.concatenate([self._axis * np.sin(half)[:, None], np.cos(half)[:, None]], axis=1)
        rotations = _canonical(_multiply(self._origin_quaternion, motion))
        slid = self._axis * np.where(self._slides, at, 0.0)[:, None]
        translations = self._origin_translation + np.einsum(
            "nij,nj->ni", self._origin_rotation, slid
        )
        return translations, rotations


def _multiply(a: NDArray[np.float64], b: NDArray[np.float64]) -> NDArray[np.float64]:
    """Row-wise Hamilton product of x, y, z, w quaternions."""
    ax, ay, az, aw = a.T
    bx, by, bz, bw = b.T
    return np.stack(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ],
        axis=1,
    )


def _canonical(quaternions: NDArray[np.float64]) -> NDArray[np.float64]:
    """The same rotations with w >= 0."""
    return np.where(quaternions[:, 3:4] < 0.0, -quaternions, quaternions)


def _quaternions(rotations: NDArray[np.float64]) -> NDArray[np.float64]:
    """x, y, z, w quaternions of rotation matrices, once at setup."""
    if not len(rotations):
        return np.zeros((0, 4))
    return _canonical(np.asarray(Rotation.from_matrix(rotations).as_quat()).reshape(-1, 4))

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

"""Independent SEW-Mimic paper reimplementation for verified YXZ-YZYX arms.

Algorithms 1/2 and Appendix C of https://arxiv.org/html/2602.01632v1.
No author implementation is reused. Matches signed joint-axis directions and
hand orientation, not wrist position. No collision or dynamics guarantees.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import product
from typing import TypeAlias, cast

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

Array: TypeAlias = NDArray[np.float64]


def unit(v: Array) -> Array:
    v = np.asarray(v, dtype=float)
    if v.shape != (3,) or not np.isfinite(v).all() or np.linalg.norm(v) < 1e-9:
        raise ValueError("Missing or degenerate bone direction")
    return v / np.linalg.norm(v)


def rotation(axis: Array, angle: float) -> Array:
    return cast("Array", Rotation.from_rotvec(axis * angle).as_matrix())


def rotation_error(a: Array, b: Array) -> float:
    return float(Rotation.from_matrix(a.T @ b).magnitude())


def equivalents(angle: float, lower: float, upper: float) -> list[float]:
    return [
        angle + 2 * np.pi * k
        for k in range(
            int(np.ceil((lower - angle) / (2 * np.pi))),
            int(np.floor((upper - angle) / (2 * np.pi))) + 1,
        )
    ]


def align_pair(a: Array, b: Array, v: Array, target: Array, seed: Array) -> list[Array]:
    """Analytic two-axis alignment via scalar circle/plane intersection (SP4/SP1)."""
    a, b, v, target = unit(a), unit(b), unit(v), unit(target)
    c = float(a @ b * (b @ v))
    cosine = float(a @ v) - c
    sine = float(a @ np.cross(b, v))
    radius = float(np.hypot(cosine, sine))
    d = float(a @ target) - c
    if radius < 1e-10:
        if abs(d) > 1e-8:
            return []
        betas = [float(seed[1])]
    elif abs(d) > radius + 1e-8:
        return []
    else:
        phase = np.arctan2(sine, cosine)
        offset = np.arccos(np.clip(d / radius, -1, 1))
        betas = [float(phase + offset), float(phase - offset)]
    result = []
    for beta in betas:
        p = rotation(b, beta) @ v
        pp, tt = p - a * (a @ p), target - a * (a @ target)
        alpha = (
            float(seed[0])
            if np.linalg.norm(pp) < 1e-9
            else float(np.arctan2(a @ np.cross(pp, tt), pp @ tt))
        )
        if np.linalg.norm(rotation(a, alpha) @ p - target) < 1e-7:
            result.append(np.array([alpha, beta]))
    return result


@dataclass(frozen=True)
class SewArmTarget:
    shoulder: Array
    elbow: Array
    wrist: Array
    hand_rotation: Array

    def directions(self) -> tuple[Array, Array]:
        h = np.asarray(self.hand_rotation)
        if (
            h.shape != (3, 3)
            or not np.isfinite(h).all()
            or not np.allclose(h.T @ h, np.eye(3), atol=1e-6)
            or np.linalg.det(h) < 0
        ):
            raise ValueError("Hand orientation must be a proper rotation")
        return unit(self.elbow - self.shoulder), unit(self.wrist - self.elbow)


class SewArmSolver:
    """Exact signed-axis solution; finite limits reject rather than clip targets."""

    def __init__(self, lower: Array, upper: Array) -> None:
        self.lower = np.asarray(lower, dtype=float)
        self.upper = np.asarray(upper, dtype=float)
        if (
            self.lower.shape != (7,)
            or self.upper.shape != (7,)
            or not np.isfinite([self.lower, self.upper]).all()
            or np.any(self.lower >= self.upper)
        ):
            raise ValueError("Expected seven finite ordered joint limits")
        self.axes = np.array(
            [[0, 1, 0], [1, 0, 0], [0, 0, 1], [0, 1, 0], [0, 0, 1], [0, 1, 0], [1, 0, 0]],
            dtype=float,
        )

    def _choose(self, candidates: list[Array], start: int, seed: Array) -> Array:
        legal: list[Array] = []
        for candidate in candidates:
            options = [
                equivalents(float(x), float(self.lower[start + i]), float(self.upper[start + i]))
                for i, x in enumerate(candidate)
            ]
            legal.extend(np.array(q) for q in product(*options))
        if not legal:
            raise ValueError(f"No limit-compliant SEW branch at joint {start + 1}")
        return min(legal, key=lambda q: float(np.abs(q - seed).sum()))

    def features(self, q: Array) -> tuple[Array, Array, Array]:
        r = np.eye(3)
        upper = lower = np.zeros(3)
        for i, (axis, angle) in enumerate(zip(self.axes, q, strict=True)):
            r = r @ rotation(axis, float(angle))
            if i == 1:
                upper = r @ np.array([0.0, 0.0, -1.0])
            if i == 3:
                lower = r @ np.array([0.0, 0.0, -1.0])
        return upper, lower, r

    def solve(self, target: SewArmTarget, seed: Array) -> Array:
        u, l = target.directions()
        seed = np.asarray(seed, dtype=float)
        if seed.shape != (7,) or not np.isfinite(seed).all():
            raise ValueError("Expected seven finite seed angles")
        q = seed.copy()
        v = np.array([0.0, 0.0, -1.0])  # R1's joint axes point opposite distal limbs.
        q[:2] = self._choose(align_pair(self.axes[0], self.axes[1], v, u, q[:2]), 0, q[:2])
        r = rotation(self.axes[0], q[0]) @ rotation(self.axes[1], q[1])
        q[2:4] = self._choose(align_pair(self.axes[2], self.axes[3], v, r.T @ l, q[2:4]), 2, q[2:4])
        r = r @ rotation(self.axes[2], q[2]) @ rotation(self.axes[3], q[3])
        wrist = r.T @ target.hand_rotation
        # ZYX wrist. Its configured pitch limits exclude +/- pi/2 gimbal lock.
        if abs(abs(wrist[2, 0]) - 1) < 1e-9:
            raise ValueError("Wrist gimbal singularity")
        y = float(np.arcsin(np.clip(-wrist[2, 0], -1, 1)))
        z = float(np.arctan2(wrist[1, 0], wrist[0, 0]))
        x = float(np.arctan2(wrist[2, 1], wrist[2, 2]))
        q[4:] = self._choose(
            [np.array([z, y, x]), np.array([z + np.pi, np.pi - y, x + np.pi])], 4, q[4:]
        )
        return q

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

"""Planar slab kinematics in the sagittal plane (x forward, z up, hinge axis at origin).

A slab is a rigid box hanging from the hinge; its foot is a flat 2c-deep bottom that
rolls over its leading/trailing edge when tilted. "Foot position" means where the foot's
flat center sits (or would sit) on the ground, so it stays fixed while the foot rolls.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import NDArray

from tars_sdk.model.params import Params


@dataclass(frozen=True)
class SlabGeometry:
    pivot_to_bottom: float  # hinge axis to foot bottom with slide retracted (m)
    half_depth: float  # half the foot depth along x (m)
    slide_travel: float  # max slide extension (m)
    slab_y: tuple[float, ...]  # lateral offset of each slab (m), slab 1 first

    @classmethod
    def from_params(cls, p: Params) -> SlabGeometry:
        return cls(
            pivot_to_bottom=p.pivot_height,
            half_depth=p.slab_depth / 2,
            slide_travel=p.slide_travel,
            slab_y=tuple(p.slab_y(i) for i in range(1, p.n_slabs + 1)),
        )


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def rot_y(theta: float, x: float, z: float) -> tuple[float, float]:
    """Rotate (x, z) by theta about +Y."""
    c, s = math.cos(theta), math.sin(theta)
    return x * c + z * s, -x * s + z * c


def foot_ik(g: SlabGeometry, dx: float, h: float) -> tuple[float, float]:
    """Slab angle and slide that put a planted foot's flat center dx ahead of the hinge,
    with the ground h below the hinge. Returns (theta, slide); slide is not clamped."""
    c = g.half_depth
    e = c if dx <= 0 else -c  # contact edge in slab frame: toe when the foot is behind
    ex = dx + e  # contact edge x relative to the hinge
    r = math.sqrt(max(h * h + ex * ex - c * c, 0.0))
    theta = wrap_angle(math.atan2(ex, -h) - math.atan2(e, -r))
    return theta, r - g.pivot_to_bottom


def foot_fk(g: SlabGeometry, theta: float, slide: float) -> tuple[float, float, float]:
    """Lowest foot edge for a slab at world pitch theta. Returns (edge_x, edge_z, center_x)."""
    r = g.pivot_to_bottom + slide
    e = g.half_depth if theta >= 0 else -g.half_depth
    x, z = rot_y(theta, e, -r)
    return x, z, x - e


def quat_to_mat(q: NDArray[np.float64]) -> NDArray[np.float64]:
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def heading(R: NDArray[np.float64]) -> float:
    """Yaw of the hub axle. Stays well-defined when the hub pitches through 90 deg (roll mode)."""
    return math.atan2(-R[0, 1], R[1, 1])


def yaw_pitch(q: NDArray[np.float64]) -> tuple[float, float]:
    R = quat_to_mat(q)
    yaw = heading(R)
    cy, sy = math.cos(yaw), math.sin(yaw)
    fwd = np.array([cy, sy, 0.0])  # heading direction; pitch = rotation of body x about the axle
    x_b = R[:, 0]
    return yaw, math.atan2(-x_b[2], float(fwd @ x_b))

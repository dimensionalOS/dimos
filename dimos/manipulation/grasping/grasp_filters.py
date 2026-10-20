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

"""Proposer-independent checks of grasp poses against what the camera saw around the object.

Poses are TCP poses (approach along +z, jaws closing along +y). The gripper is a few boxes
sampled on their surfaces, the xArm gripper open: two fingers ending at the TCP and the body
behind them. The checks are as GraspGenX's own collision filter (gripper samples near a scene
point collide) but in NumPy, so they cover any proposer, in the stack or served.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

# (centre, size) per box in the TCP frame, metres: the open fingers, then the body.
XARM_GRIPPER_BOXES = (
    ((0.0, 0.0495, -0.0325), (0.024, 0.010, 0.065)),
    ((0.0, -0.0495, -0.0325), (0.024, 0.010, 0.065)),
    ((0.0, 0.0, -0.1175), (0.060, 0.110, 0.105)),
)
# A gripper sample this close to a scene point collides; the scene is thinned to 5 mm cells.
COLLISION_M = 0.008
# A grasp whose lowest gripper sample is below the support plus this would hit it.
SUPPORT_MARGIN_M = 0.003


def box_surface_points(
    boxes: tuple[
        tuple[tuple[float, float, float], tuple[float, float, float]], ...
    ] = XARM_GRIPPER_BOXES,
    step: float = 0.005,
) -> NDArray[np.float64]:
    """Points every ``step`` on the faces of axis-aligned boxes."""
    out = []
    for centre, size in boxes:
        c, s = np.asarray(centre), np.asarray(size)
        axes = [np.linspace(-h, h, max(2, round(2 * h / step) + 1)) for h in s / 2]
        for k in range(3):
            i, j = [a for a in range(3) if a != k]
            u, v = np.meshgrid(axes[i], axes[j])
            for side in (-1.0, 1.0):
                face = np.zeros((u.size, 3))
                face[:, i], face[:, j], face[:, k] = u.ravel(), v.ravel(), side * s[k] / 2
                out.append(face + c)
    return np.unique(np.round(np.vstack(out), 6), axis=0)


def pose_matrices(
    positions: NDArray[np.float64], quaternions_xyzw: NDArray[np.float64]
) -> NDArray[np.float64]:
    """(K, 4, 4) transforms from positions (K, 3) and unit quaternions (K, 4), x y z w."""
    x, y, z, w = quaternions_xyzw.T
    m = np.zeros((len(positions), 4, 4))
    m[:, 0, 0] = 1 - 2 * (y * y + z * z)
    m[:, 0, 1] = 2 * (x * y - z * w)
    m[:, 0, 2] = 2 * (x * z + y * w)
    m[:, 1, 0] = 2 * (x * y + z * w)
    m[:, 1, 1] = 1 - 2 * (x * x + z * z)
    m[:, 1, 2] = 2 * (y * z - x * w)
    m[:, 2, 0] = 2 * (x * z - y * w)
    m[:, 2, 1] = 2 * (y * z + x * w)
    m[:, 2, 2] = 1 - 2 * (x * x + y * y)
    m[:, :3, 3] = positions
    m[:, 3, 3] = 1.0
    return m


def gripper_in_world(
    poses: NDArray[np.float64], gripper: NDArray[np.float64]
) -> NDArray[np.float64]:
    """(K, M, 3): the gripper samples placed at each pose."""
    return np.einsum("kij,mj->kmi", poses[:, :3, :3], gripper) + poses[:, None, :3, 3]


def support_height(scene: NDArray[np.floating], below: float) -> float | None:
    """The height of the densest 5 mm layer of scene points under ``below``: the table or shelf
    the object stands on. None when fewer than 50 points are under it."""
    z = scene[:, 2][scene[:, 2] < below]
    if len(z) < 50:
        return None
    edges = np.arange(z.min(), z.max() + 0.01, 0.005)
    counts, edges = np.histogram(z, bins=edges)
    k = int(np.argmax(counts))
    layer = z[(z >= edges[k]) & (z <= edges[k + 1])]
    return float(np.median(layer))


def colliding(
    poses: NDArray[np.float64],
    scene: NDArray[np.floating],
    gripper: NDArray[np.float64],
    threshold: float = COLLISION_M,
) -> NDArray[np.bool_]:
    """(K,) True where any gripper sample lies within ``threshold`` of a scene point."""
    from scipy.spatial import cKDTree

    if not len(poses) or not len(scene):
        return np.zeros(len(poses), dtype=bool)
    placed = gripper_in_world(poses, gripper)
    distance, _ = cKDTree(scene).query(placed.reshape(-1, 3), distance_upper_bound=threshold)
    return np.isfinite(distance).reshape(len(poses), -1).any(axis=1)


def below_support(
    poses: NDArray[np.float64],
    support: float,
    gripper: NDArray[np.float64],
    margin: float = SUPPORT_MARGIN_M,
) -> NDArray[np.bool_]:
    """(K,) True where the gripper's lowest sample goes below the support plus ``margin``."""
    if not len(poses):
        return np.zeros(0, dtype=bool)
    return gripper_in_world(poses, gripper)[:, :, 2].min(axis=1) < support + margin


def tilt_degrees(poses: NDArray[np.float64]) -> NDArray[np.float64]:
    """(K,) angle of each approach (+z) from straight down, degrees."""
    approach_z = poses[:, 2, 2]
    return np.degrees(np.arccos(np.clip(-approach_z, -1.0, 1.0)))

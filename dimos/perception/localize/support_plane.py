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

"""RANSAC fit of the dominant horizontal support surface from window keyframes."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import NDArray

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.memory.type.observation import Observation
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.perception.localize.rig import Rig

logger = setup_logger()

PLANE_DISTANCE = 0.01  # m - RANSAC inlier distance
MIN_HORIZONTAL_DOT = 0.90  # |normal . z| for a plane to count as horizontal
PLANE_INLIER_QUANTILE = 0.15  # anchor plane offset near the base of the inlier slab
PLANE_SEED = 0  # so one window always fits the same plane


@dataclass
class SupportPlane:
    """Plane coefficients (a, b, c, d), normal up."""

    coefficients: tuple[float, float, float, float]

    def height_above(self, points: np.ndarray) -> np.ndarray:
        a, b, c, d = self.coefficients
        heights: np.ndarray = points @ np.array([a, b, c]) + d
        return heights


def window_points(
    rig: Rig, keyframes: list[Observation[Image]], frame: str, frames: int = 5
) -> NDArray[np.float64] | None:
    """Up to ``frames`` keyframes of one camera merged at 1 cm; None under 500 points."""
    if not keyframes:
        return None
    clouds = []
    for obs in keyframes[:: max(1, len(keyframes) // frames)][:frames]:
        cloud = rig.backdrop(obs.ts, frame)
        if cloud is not None:
            clouds.append(cloud.voxel_downsample(0.01))
    if not clouds:
        return None
    merged = clouds[0]
    for cloud in clouds[1:]:
        merged = merged + cloud
    points = np.asarray(merged.pointcloud.points)
    return points if len(points) >= 500 else None


def peel_planes(
    points: NDArray[np.float64], min_up_dot: float
) -> list[tuple[NDArray[np.float64], NDArray[np.float64]]]:
    """Up to four RANSAC planes with ``|n . z| >= min_up_dot``: (a, b, c, d) normal up, inliers."""
    import open3d as o3d

    o3d.utility.random.seed(PLANE_SEED)
    remaining = o3d.geometry.PointCloud()
    remaining.points = o3d.utility.Vector3dVector(points)
    found: list[tuple[NDArray[np.float64], NDArray[np.float64]]] = []
    for _ in range(4):
        if len(remaining.points) < 500:
            break
        model, inlier_idx = remaining.segment_plane(
            distance_threshold=PLANE_DISTANCE, ransac_n=3, num_iterations=1000, probability=1.0
        )
        model = np.array(model)
        if abs(model[2]) >= min_up_dot:
            found.append(
                (model if model[2] >= 0 else -model, np.asarray(remaining.points)[inlier_idx])
            )
        remaining = remaining.select_by_index(inlier_idx, invert=True)
    return found


def anchored(model: NDArray[np.float64], inliers: NDArray[np.float64]) -> SupportPlane:
    """The plane through the base of its inlier slab.

    Thin objects (< 1 cm) sitting on the support surface fall inside the RANSAC
    inlier slab and bias the least-squares offset upward; anchoring the offset
    at the lower quantile of the inliers keeps the plane on the support surface.
    """
    d = -float(np.quantile(inliers @ model[:3], PLANE_INLIER_QUANTILE))
    return SupportPlane(coefficients=(float(model[0]), float(model[1]), float(model[2]), d))


def fit_support_plane(
    rig: Rig, keyframes: list[Observation[Image]], frame: str
) -> SupportPlane | None:
    """Dominant near-horizontal plane in one camera's keyframes; non-horizontal
    candidates are peeled off."""
    points = window_points(rig, keyframes, frame)
    if points is None:
        return None
    planes = peel_planes(points, MIN_HORIZONTAL_DOT)
    if not planes:
        logger.warning("support plane: no horizontal plane found in backdrop")
        return None
    model, inliers = max(planes, key=lambda plane: len(plane[1]))
    return anchored(model, inliers)

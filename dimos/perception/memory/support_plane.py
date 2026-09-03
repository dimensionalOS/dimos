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

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.memory.type.observation import Observation
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.perception.memory.rig import Rig

logger = setup_logger()

PLANE_DISTANCE = 0.01  # m - RANSAC inlier distance for depth-camera clouds
PLANE_DISTANCE_CLOUD = 0.03  # m - registered lidar is noisier
MIN_HORIZONTAL_DOT = 0.90  # |normal . z| for a plane to count as horizontal
PLANE_SEED = 0  # so one window always fits the same plane


@dataclass
class SupportPlane:
    """Plane coefficients (a, b, c, d), normal up, plus the inlier footprint."""

    coefficients: tuple[float, float, float, float]
    footprint_hull: np.ndarray  # (K, 2) convex hull of inlier (x, y), world
    inlier_count: int

    def height_above(self, points: np.ndarray) -> np.ndarray:
        a, b, c, d = self.coefficients
        heights: np.ndarray = points @ np.array([a, b, c]) + d
        return heights


def fit_support_plane(rig: Rig, keyframes: list[Observation[Image]]) -> SupportPlane | None:
    """Dominant near-horizontal plane; non-horizontal candidates are peeled off."""
    if not keyframes:
        return None

    clouds = []
    for obs in keyframes[:: max(1, len(keyframes) // 5)][:5]:
        cloud = rig.backdrop(obs.ts)
        if cloud is not None:
            clouds.append(cloud.voxel_downsample(0.01))
    if not clouds:
        return None

    merged = clouds[0]
    for cloud in clouds[1:]:
        merged = merged + cloud
    points = np.asarray(merged.pointcloud.points)
    if len(points) < 500:
        return None

    import open3d as o3d

    o3d.utility.random.seed(PLANE_SEED)
    distance = PLANE_DISTANCE if rig.depth is not None else PLANE_DISTANCE_CLOUD
    remaining = o3d.geometry.PointCloud()
    remaining.points = o3d.utility.Vector3dVector(points)
    best: tuple[np.ndarray, np.ndarray] | None = None
    for _ in range(4):
        if len(remaining.points) < 500:
            break
        model, inlier_idx = remaining.segment_plane(
            distance_threshold=distance, ransac_n=3, num_iterations=1000, probability=1.0
        )
        inliers = np.asarray(remaining.points)[inlier_idx]
        if abs(model[2]) >= MIN_HORIZONTAL_DOT and (best is None or len(inliers) > len(best[1])):
            best = (np.array(model), inliers)
        remaining = remaining.select_by_index(inlier_idx, invert=True)

    if best is None:
        logger.warning("support plane: no horizontal plane found in backdrop")
        return None

    model, inliers = best
    if model[2] < 0:
        model = -model
    from scipy.spatial import ConvexHull

    xy = inliers[:, :2]
    return SupportPlane(
        coefficients=(float(model[0]), float(model[1]), float(model[2]), float(model[3])),
        footprint_hull=xy[ConvexHull(xy).vertices],
        inlier_count=len(inliers),
    )

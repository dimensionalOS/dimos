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

"""Support-surface fit and the scope predicate derived from it.

The plane is RANSAC-fit from the window's own frames - nothing scene-specific
is passed in and no caller supplies coordinates. On a wrist-camera rig over a
workspace the dominant horizontal plane is the tabletop; on a mobile rig it
is the floor. The plane's inlier footprint is the workspace; the scope
predicate accepts a support when its cloud sits in a band above the plane and
its footprint intersects the plane footprint.
"""

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
PLANE_SEED = 0  # RANSAC is seeded so one window always fits the same plane
FOOTPRINT_DILATE_M = 0.03


@dataclass
class SupportPlane:
    """A horizontal support surface: plane coefficients plus inlier footprint."""

    # Plane (a, b, c, d): a*x + b*y + c*z + d = 0, normal pointing up (+z).
    coefficients: tuple[float, float, float, float]
    footprint_hull: np.ndarray  # (K, 2) convex hull of inlier (x, y), world
    inlier_count: int

    @property
    def normal(self) -> np.ndarray:
        return np.array(self.coefficients[:3])

    def height_above(self, points: np.ndarray) -> np.ndarray:
        """Signed height of (N, 3) world points above the plane."""
        a, b, c, d = self.coefficients
        heights: np.ndarray = points @ np.array([a, b, c]) + d
        return heights

    def footprint_contains(self, points_xy: np.ndarray) -> np.ndarray:
        """Boolean mask: which (N, 2) world XY points fall inside the (dilated) hull."""
        from matplotlib.path import Path as MplPath

        hull = self.footprint_hull
        center = hull.mean(axis=0)
        offsets = hull - center
        norms = np.linalg.norm(offsets, axis=1, keepdims=True)
        dilated = hull + offsets / np.maximum(norms, 1e-9) * FOOTPRINT_DILATE_M
        return MplPath(dilated).contains_points(points_xy)


def fit_support_plane(rig: Rig, keyframes: list[Observation[Image]]) -> SupportPlane | None:
    """Fit the dominant near-horizontal plane from a handful of window keyframes.

    Non-horizontal dominant planes (a wall, a screen) are peeled off and the
    fit repeats on the remainder. Among horizontal candidates the one with
    the most inliers wins.
    """
    if not keyframes:
        return None

    picks = keyframes[:: max(1, len(keyframes) // 5)][:5]
    clouds = []
    for obs in picks:
        cloud = rig.backdrop(obs.ts)
        if cloud is None:
            continue
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

    # Unseeded, the plane fit lands on a different set of inliers each run
    o3d.utility.random.seed(PLANE_SEED)

    distance = PLANE_DISTANCE if rig.depth is not None else PLANE_DISTANCE_CLOUD
    remaining = o3d.geometry.PointCloud()
    remaining.points = o3d.utility.Vector3dVector(points)
    best: tuple[np.ndarray, np.ndarray] | None = None  # (coefficients, inlier points)
    for _ in range(4):
        if len(remaining.points) < 500:
            break
        model, inlier_idx = remaining.segment_plane(
            distance_threshold=distance, ransac_n=3, num_iterations=1000, probability=1.0
        )
        inliers = np.asarray(remaining.points)[inlier_idx]
        normal = np.array(model[:3])
        if abs(normal[2]) >= MIN_HORIZONTAL_DOT:
            if best is None or len(inliers) > len(best[1]):
                best = (np.array(model), inliers)
        remaining = remaining.select_by_index(inlier_idx, invert=True)

    if best is None:
        logger.warning("support plane: no horizontal plane found in backdrop")
        return None

    model, inliers = best
    if model[2] < 0:  # normal points up
        model = -model
    from scipy.spatial import ConvexHull

    xy = inliers[:, :2]
    hull = ConvexHull(xy)
    return SupportPlane(
        coefficients=(float(model[0]), float(model[1]), float(model[2]), float(model[3])),
        footprint_hull=xy[hull.vertices],
        inlier_count=len(inliers),
    )

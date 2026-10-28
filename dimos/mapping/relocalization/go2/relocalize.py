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

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

import numpy as np

if TYPE_CHECKING:
    import open3d as o3d  # type: ignore[import-untyped]

# (voxel_size, total RANSAC runs at that scale). 0.8m is the coarsest, cheapest
# scale; it provides anchor candidates that don't need as many restarts.
SCALE_PLAN: list[tuple[float, int]] = [
    (0.2, 8),
    (0.3, 8),
    (0.8, 1),
]
RANSAC_ITERS = 500_000  # RANSAC iteration budget per scale
FINE_VOXEL = 0.1  # voxel for the final ICP refinement
RERANK_DIST = FINE_VOXEL * 1.5  # inlier dist for fine-scale candidate scoring
GRAVITY_TILT_MAX_DEG = 10.0  # reject candidates whose z-axis tilts more than this


def _preprocess(
    pcd: o3d.geometry.PointCloud, voxel_size: float
) -> tuple[o3d.geometry.PointCloud, Any]:
    """Downsample, estimate normals, compute FPFH descriptors."""
    import open3d as o3d

    _reg = o3d.pipelines.registration
    down = pcd.voxel_down_sample(voxel_size)
    down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    fpfh = _reg.compute_fpfh_feature(
        down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100),
    )
    return down, fpfh


@dataclass(frozen=True)
class Target:
    """A global map prepared once: per-scale downsample and FPFH, and the fine cloud for ICP."""

    scales: dict[float, tuple[o3d.geometry.PointCloud, Any]]
    fine: o3d.geometry.PointCloud


def prepare(global_map: o3d.geometry.PointCloud) -> Target:
    import open3d as o3d

    fine = global_map.voxel_down_sample(FINE_VOXEL)
    fine.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=FINE_VOXEL * 2, max_nn=30))
    return Target({vs: _preprocess(global_map, vs) for vs, _ in SCALE_PLAN}, fine)


def _ransac(
    src_down: o3d.geometry.PointCloud,
    tgt_down: o3d.geometry.PointCloud,
    src_fpfh: Any,
    tgt_fpfh: Any,
    voxel_size: float,
) -> Any:
    """Open3D feature-matching RANSAC. Returns a RegistrationResult.

    Docs:
      https://www.open3d.org/docs/latest/python_api/open3d.registration.registration_ransac_based_on_feature_matching.html
    """
    import open3d as o3d

    _reg = o3d.pipelines.registration
    dist = voxel_size * 1.5
    return _reg.registration_ransac_based_on_feature_matching(
        src_down,
        tgt_down,
        src_fpfh,
        tgt_fpfh,
        mutual_filter=True,
        max_correspondence_distance=dist,
        estimation_method=_reg.TransformationEstimationPointToPoint(False),
        ransac_n=3,
        checkers=[
            _reg.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            _reg.CorrespondenceCheckerBasedOnDistance(dist),
        ],
        criteria=_reg.RANSACConvergenceCriteria(RANSAC_ITERS, 0.995),
    )


def _gravity_tilt_deg(T: np.ndarray) -> float:
    """Angle (deg) between the transform's z-axis and world z-up."""
    z_world = T[:3, :3] @ np.array([0.0, 0.0, 1.0])
    return float(np.degrees(np.arccos(np.clip(z_world[2], -1.0, 1.0))))


def _wall_subset(cloud: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    import open3d as o3d

    nrm = np.asarray(cloud.normals)
    mask = np.abs(nrm[:, 2]) < 0.7  # roughly horizontal
    if mask.sum() < 100:
        return cloud  # too sparse -> fall back to full cloud
    sub = o3d.geometry.PointCloud()
    sub.points = o3d.utility.Vector3dVector(np.asarray(cloud.points)[mask])
    sub.normals = o3d.utility.Vector3dVector(nrm[mask])
    return sub


def wall_fitness(
    target: Target,
    local_map: o3d.geometry.PointCloud,
    T: np.ndarray,
) -> float:
    """What `relocalize` would score placement ``T`` as: wall-only fitness after its stage 2 polish."""
    import open3d as o3d

    _reg = o3d.pipelines.registration
    src_fine = local_map.voxel_down_sample(FINE_VOXEL)
    src_fine.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=FINE_VOXEL * 2, max_nn=30)
    )
    tgt_fine = target.fine
    tukey = _reg.TransformationEstimationPointToPlane(_reg.TukeyLoss(k=RERANK_DIST))
    r = _reg.registration_icp(
        _wall_subset(src_fine),
        _wall_subset(tgt_fine),
        RERANK_DIST,
        T,
        tukey,
        _reg.ICPConvergenceCriteria(max_iteration=70),
    )
    return float(r.fitness)


def relocalize(
    target: Target,
    local_map: o3d.geometry.PointCloud,
) -> tuple[np.ndarray, float]:
    """Return the 4x4 placing ``local_map`` in the ``target`` map, and its wall fitness.

    Multi-scale FPFH+RANSAC candidates, gravity-filtered, re-ranked by wall-only
    fine-scale fitness (RANSAC's own fitness passes wrong-room candidates), then ICP.
    """
    import open3d as o3d

    _reg = o3d.pipelines.registration

    # Fine downsample once, for both candidate scoring and the final ICP.
    src_fine = local_map.voxel_down_sample(FINE_VOXEL)
    src_fine.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=FINE_VOXEL * 2, max_nn=30)
    )
    tgt_fine = target.fine

    candidates: list[np.ndarray] = []  # 4x4 transforms
    for vs, n_runs in SCALE_PLAN:
        src_down, src_fpfh = _preprocess(local_map, vs)
        tgt_down, tgt_fpfh = target.scales[vs]
        for _ in range(n_runs):
            # Successive calls advance Open3D's RNG state, so each restart explores a new sample.
            result = _ransac(src_down, tgt_down, src_fpfh, tgt_fpfh, vs)
            candidates.append(np.asarray(result.transformation))

    # Each candidate also flipped 180 deg about the cloud's own xy centroid: same place, opposite heading.
    src_pts = np.asarray(src_fine.points)
    c_body = np.array([src_pts[:, 0].mean(), src_pts[:, 1].mean(), 0.0])
    rz180 = np.diag([-1.0, -1.0, 1.0])
    t_body_flip = np.eye(4)
    t_body_flip[:3, :3] = rz180
    t_body_flip[:3, 3] = c_body - rz180 @ c_body  # = (2*Cx, 2*Cy, 0)
    candidates = candidates + [T @ t_body_flip for T in candidates]

    # Gravity filter; fall back to all if everything is tilted (degenerate clouds).
    upright = [T for T in candidates if _gravity_tilt_deg(T) <= GRAVITY_TILT_MAX_DEG]
    pool = upright if upright else candidates

    # Score and polish on walls only: floors fit any yaw, so they would hide a flipped candidate.
    src_walls = _wall_subset(src_fine)
    tgt_walls = _wall_subset(tgt_fine)

    # Stage 1: rank all candidates by WALL-only fine-scale fitness.
    def fine_fitness(T: np.ndarray) -> float:
        r = _reg.evaluate_registration(src_walls, tgt_walls, RERANK_DIST, T)
        return float(r.fitness)

    top_k = sorted(pool, key=fine_fitness, reverse=True)[:10]

    # Stage 2: ICP each of the top 10 on walls, keep the one whose walls align best.
    tukey = _reg.TransformationEstimationPointToPlane(_reg.TukeyLoss(k=RERANK_DIST))
    polished: list[tuple[float, np.ndarray]] = []
    for T0 in top_k:
        r = _reg.registration_icp(
            src_walls,
            tgt_walls,
            RERANK_DIST,
            T0,
            tukey,
            _reg.ICPConvergenceCriteria(max_iteration=70),
        )
        polished.append((float(r.fitness), np.asarray(r.transformation)))
    best_fit, best_T = max(polished, key=lambda fT: fT[0])

    # Stage 3: final ICP on full clouds, incl. floor/ceiling
    final = _reg.registration_icp(
        src_fine,
        tgt_fine,
        RERANK_DIST,
        best_T,
        tukey,
        _reg.ICPConvergenceCriteria(max_iteration=50),
    )
    return np.asarray(final.transformation), best_fit

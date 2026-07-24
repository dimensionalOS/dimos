#!/usr/bin/env python3
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

from typing import Any

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]

_reg = o3d.pipelines.registration

# (voxel_size m, RANSAC runs at that scale); 0.8m is the coarsest/cheapest anchor
# scale, needing fewer restarts.
SCALE_PLAN: list[tuple[float, int]] = [
    (0.2, 8),
    (0.3, 8),
    (0.8, 1),
]
RANSAC_ITERS = 500_000  # RANSAC iteration budget per scale
FINE_VOXEL = 0.1  # m, voxel for the final ICP refinement
RERANK_DIST = FINE_VOXEL * 1.5  # inlier dist for fine-scale candidate scoring
GRAVITY_TILT_MAX_DEG = 10.0  # reject candidates whose z-axis tilts more than this
# Min wall points (per cloud, post fine-voxel downsample) to attempt the wall-only
# rerank. ARBITRARY / UNTUNED: inherited from the old silent-fallback check.
MIN_WALL_POINTS = 100


class InsufficientWallEvidenceError(ValueError):
    """Too few wall points for the wall-only rerank to discriminate yaw (flat floors are rotationally symmetric); refuse rather than degrade."""


class NoUprightCandidateError(ValueError):
    """Every candidate tilts past ``gravity_tilt_max_deg``; no valid winner, so refuse rather than admit a gravity-violating pose."""


def _preprocess(
    pcd: o3d.geometry.PointCloud, voxel_size: float
) -> tuple[o3d.geometry.PointCloud, Any]:
    """Downsample, estimate normals, compute FPFH descriptors."""
    down = pcd.voxel_down_sample(voxel_size)
    down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    # FPFH descriptors. normal radius = 2*voxel, feature radius = 5*voxel per the
    # Open3D global-registration tutorial; FPFH: Rusu, Blodow, Beetz 2009,
    # https://doi.org/10.1109/ROBOT.2009.5152473
    fpfh = _reg.compute_fpfh_feature(
        down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100),
    )
    return down, fpfh


# Per-process cache of the global map's downsampled cloud + FPFH features; a worker
# reuses one global map across all its frames, so the first call pays and the rest are free.
_GLOBAL_CACHE: dict[tuple[str, float, int], Any] = {}


def _global_preprocess(
    global_map: o3d.geometry.PointCloud, voxel_size: float
) -> tuple[o3d.geometry.PointCloud, Any]:
    key = ("ransac", voxel_size, len(global_map.points))
    cached = _GLOBAL_CACHE.get(key)
    if cached is None:
        cached = _preprocess(global_map, voxel_size)
        _GLOBAL_CACHE[key] = cached
    return cached  # type: ignore[no-any-return]


def _global_fine(global_map: o3d.geometry.PointCloud, voxel_size: float) -> o3d.geometry.PointCloud:
    key = ("fine", voxel_size, len(global_map.points))
    cached = _GLOBAL_CACHE.get(key)
    if cached is None:
        down = global_map.voxel_down_sample(voxel_size)
        down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30)
        )
        cached = down
        _GLOBAL_CACHE[key] = cached
    return cached  # type: ignore[no-any-return]


def _ransac(
    src_down: o3d.geometry.PointCloud,
    tgt_down: o3d.geometry.PointCloud,
    src_fpfh: Any,
    tgt_fpfh: Any,
    voxel_size: float,
) -> Any:
    """Open3D feature-matching RANSAC. Returns a RegistrationResult."""
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
            _reg.CorrespondenceCheckerBasedOnEdgeLength(0.9),  # edge-length similarity ratio
            _reg.CorrespondenceCheckerBasedOnDistance(dist),
        ],
        criteria=_reg.RANSACConvergenceCriteria(RANSAC_ITERS, 0.995),  # 0.995 = confidence
    )


def _gravity_tilt_deg(T: np.ndarray) -> float:
    """Angle (deg) between the transform's z-axis and world z-up."""
    z_world = T[:3, :3] @ np.array([0.0, 0.0, 1.0])
    return float(np.degrees(np.arccos(np.clip(z_world[2], -1.0, 1.0))))


def _wall_subset(cloud: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    """Roughly-horizontal-normal subset (walls); excludes floor/ceiling points whose vertical normals fit any yaw and mask a 180° flip."""
    nrm = np.asarray(cloud.normals)
    mask = np.abs(nrm[:, 2]) < 0.7  # |n_z|<0.7 => normal within ~44 deg of horizontal
    sub = o3d.geometry.PointCloud()
    sub.points = o3d.utility.Vector3dVector(np.asarray(cloud.points)[mask])
    sub.normals = o3d.utility.Vector3dVector(nrm[mask])
    return sub


def generate_ransac_candidates(
    global_map: o3d.geometry.PointCloud,
    local_map: o3d.geometry.PointCloud,
) -> list[np.ndarray]:
    """Multi-scale x multi-restart FPFH+RANSAC candidate transforms placing ``local_map``
    into ``global_map``, plus centroid-aware 180° yaw flips: the unranked pool for
    ``refine_candidates``."""
    # Fine downsample of local_map, used only for the yaw-flip centroid below.
    src_fine_pts = np.asarray(local_map.voxel_down_sample(FINE_VOXEL).points)

    candidates: list[np.ndarray] = []  # 4x4 transforms
    for vs, n_runs in SCALE_PLAN:
        src_down, src_fpfh = _preprocess(local_map, vs)
        tgt_down, tgt_fpfh = _global_preprocess(global_map, vs)
        for _ in range(n_runs):
            # Successive calls advance Open3D's per-frame-seeded RNG, so each restart differs.
            result = _ransac(src_down, tgt_down, src_fpfh, tgt_fpfh, vs)
            candidates.append(np.asarray(result.transformation))

    # Centroid-aware yaw flip: add each candidate rotated 180° about the cloud's OWN
    # xy-centroid, not body origin -- a naive `T @ Rz_180` about body origin flings the
    # cloud across the world when lidar coverage isn't robot-centered ("same place, opposite heading").
    c_body = np.array([src_fine_pts[:, 0].mean(), src_fine_pts[:, 1].mean(), 0.0])
    rz180 = np.diag([-1.0, -1.0, 1.0])
    t_body_flip = np.eye(4)
    t_body_flip[:3, :3] = rz180
    t_body_flip[:3, 3] = c_body - rz180 @ c_body  # = (2*Cx, 2*Cy, 0)
    return candidates + [T @ t_body_flip for T in candidates]


def refine_candidates(
    global_map: o3d.geometry.PointCloud,
    local_map: o3d.geometry.PointCloud,
    candidates: list[np.ndarray],
    gravity_tilt_max_deg: float = GRAVITY_TILT_MAX_DEG,
) -> tuple[np.ndarray, float, int]:
    """Judge a pool of candidate local_map->global_map transforms and refine the winner:
    gravity-filter, rerank by WALL-only fine inlier ratio, wall ICP polish, final full-cloud
    ICP. Returns ``(T, fitness, winning_index)`` -- the index lets pooled-prior callers attribute the win."""
    # Fine downsample once — used for both candidate scoring and the final ICP.
    src_fine = local_map.voxel_down_sample(FINE_VOXEL)
    src_fine.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=FINE_VOXEL * 2, max_nn=30)
    )
    tgt_fine = _global_fine(global_map, FINE_VOXEL)

    # Gravity filter. An all-tilted pool is REFUSED, not resurrected -- a tilted winner is
    # a rotationally-symmetric-floor mis-solve, not a valid pose.
    indexed = list(enumerate(candidates))
    upright = [item for item in indexed if _gravity_tilt_deg(item[1]) <= gravity_tilt_max_deg]
    if not upright:
        raise NoUprightCandidateError(
            f"no candidate within the gravity gate: {len(indexed)} candidate(s), "
            f"none within {gravity_tilt_max_deg} deg -- refusing"
        )
    pool = upright

    # WALL-ONLY clouds score + polish; the FULL clouds drive the final refinement,
    # preserving the gravity anchor and inlier density in the output.
    src_walls = _wall_subset(src_fine)
    tgt_walls = _wall_subset(tgt_fine)
    n_src_walls, n_tgt_walls = len(src_walls.points), len(tgt_walls.points)
    if n_src_walls < MIN_WALL_POINTS or n_tgt_walls < MIN_WALL_POINTS:
        # No silent full-cloud fallback: floors-only scoring is rotation-blind.
        raise InsufficientWallEvidenceError(
            f"insufficient wall evidence: submap walls={n_src_walls}, "
            f"map walls={n_tgt_walls} < {MIN_WALL_POINTS} — skipping solve"
        )

    # Stage 1: rank all candidates by WALL-only fine-scale fitness.
    def fine_fitness(item: tuple[int, np.ndarray]) -> float:
        r = _reg.evaluate_registration(src_walls, tgt_walls, RERANK_DIST, item[1])
        return float(r.fitness)

    top_k = sorted(pool, key=fine_fitness, reverse=True)[:10]

    # Stage 2: moderate-distance ICP on each top-10, on WALL clouds so wall correspondences
    # (not floors) drive yaw+xy. Tukey biweight down-weights far correspondences so gross
    # outliers can't drag the fit. https://www.open3d.org/docs/latest/tutorial/pipelines/robust_kinematics.html
    tukey = _reg.TransformationEstimationPointToPlane(_reg.TukeyLoss(k=RERANK_DIST))
    polished: list[tuple[int, float, np.ndarray]] = []
    for i, T0 in top_k:
        r = _reg.registration_icp(
            src_walls,
            tgt_walls,
            RERANK_DIST,
            T0,
            tukey,
            _reg.ICPConvergenceCriteria(max_iteration=70),
        )
        polished.append((i, float(r.fitness), np.asarray(r.transformation)))
    winning_index, best_fit, best_T = max(polished, key=lambda item: item[1])

    # Stage 3: final ICP on full clouds, incl. floor/ceiling
    final = _reg.registration_icp(
        src_fine,
        tgt_fine,
        RERANK_DIST,
        best_T,
        tukey,
        _reg.ICPConvergenceCriteria(max_iteration=50),
    )
    return np.asarray(final.transformation), best_fit, winning_index


def relocalize(
    global_map: o3d.geometry.PointCloud,
    local_map: o3d.geometry.PointCloud,
    gravity_tilt_max_deg: float = GRAVITY_TILT_MAX_DEG,
) -> tuple[np.ndarray, float]:
    """Estimate the 4x4 transform placing ``local_map`` into ``global_map`` via RANSAC +
    the shared judge. module.py's ``_relocalize`` relies on this (T, fitness) signature."""
    candidates = generate_ransac_candidates(global_map, local_map)
    T, fitness, _winning_index = refine_candidates(
        global_map,
        local_map,
        candidates,
        gravity_tilt_max_deg=gravity_tilt_max_deg,
    )
    return T, fitness

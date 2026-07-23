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

# (voxel_size, total RANSAC runs at that scale). 0.8m is the coarsest, cheapest
# scale; it provides anchor candidates that don't need as many restarts.
SCALE_PLAN: list[tuple[float, int]] = [
    (0.2, 8),
    (0.3, 8),
    (0.8, 1),
]
RANSAC_ITERS = 500_000  # RANSAC iteration budget per scale
FINE_VOXEL = 0.1  # m, voxel for the final ICP refinement
RERANK_DIST = FINE_VOXEL * 1.5  # inlier dist for fine-scale candidate scoring
GRAVITY_TILT_MAX_DEG = 10.0  # reject candidates whose z-axis tilts more than this
# Minimum wall points (per cloud, post fine-voxel downsample) to attempt the
# wall-only rerank. ARBITRARY / UNTUNED: inherited verbatim from the old
# silent-fallback check; never observed to fire in offline replay of the
# hk_village1..6 recordings, so it has never been calibrated against real
# sparse-wall data.
MIN_WALL_POINTS = 100


class InsufficientWallEvidenceError(ValueError):
    """Too few wall (horizontal-normal) points for the wall-only rerank to mean
    anything. Floors/ceilings are rotationally symmetric, so full-cloud scoring
    would be rotation-blind -- a 180-deg flip scores as well as the truth. Refuse
    loudly rather than degrade; ``_try_relocalize`` catches, logs, and skips."""


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


# Per-process cache of the global map's downsampled cloud + FPFH features and the
# fine-voxel cloud. A worker reuses one global map across all its frames, so the
# first call pays the cost and the rest get it free. Allowed per program.md:
# caching FPFH across calls is fine within one run (fresh state per process).
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
    """Roughly-horizontal-normal subset. Floor/ceiling points have vertical
    normals; they fit equally well in any yaw rotation (flat planes are
    rotationally symmetric), so a 180°-flipped candidate can hide its wall
    misalignment behind perfect floor alignment if they're left in.

    Returns the subset unconditionally, even when it is tiny/empty; the
    sparse-wall refusal (MIN_WALL_POINTS) lives in ``refine_candidates``."""
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
    """Multi-scale x multi-restart FPFH+RANSAC candidate transforms placing
    ``local_map`` into ``global_map``, plus their centroid-aware 180° yaw
    flips. This is the RANSAC relocalization prior's raw proposal pool --
    unfiltered, unranked, un-refined; ``refine_candidates`` is the judge.
    """
    # Fine downsample of local_map, used only for the yaw-flip centroid below.
    # refine_candidates recomputes this for its own scoring -- cheap and
    # deterministic, so it costs nothing observable but keeps the contract clean.
    src_fine_pts = np.asarray(local_map.voxel_down_sample(FINE_VOXEL).points)

    candidates: list[np.ndarray] = []  # 4x4 transforms
    for vs, n_runs in SCALE_PLAN:
        src_down, src_fpfh = _preprocess(local_map, vs)
        tgt_down, tgt_fpfh = _global_preprocess(global_map, vs)
        for _ in range(n_runs):
            # Successive calls advance Open3D's RNG state (seeded per-frame in
            # run.py), so each restart explores a different sample sequence.
            result = _ransac(src_down, tgt_down, src_fpfh, tgt_fpfh, vs)
            candidates.append(np.asarray(result.transformation))

    # Centroid-aware yaw flip: for every candidate add the variant rotated 180°
    # around the cloud's OWN xy-centroid, not body origin. A naive `T @ Rz_180`
    # rotates about body origin and flings the cloud across the world when lidar
    # coverage isn't centered on the robot; rotating about the centroid keeps it in
    # place -- "same place, opposite heading" for an indoor submap.
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
    """Judge a pool of candidate local_map->global_map transforms and refine the
    winner. The single referee every prior's candidates go through: gravity-filter,
    rerank by WALL-only fine-scale inlier ratio, per-candidate wall ICP polish, then
    final full-cloud ICP. No source is trusted -- a candidate wins by fitness or not
    at all. The rerank catches z-degenerate and wrong-room busts: at FINE_VOXEL a
    5m-off candidate has ~0 inliers even when its proposer reported it fit.

    Returns ``(T, fitness, winning_index)``, ``winning_index`` being the position in
    ``candidates`` that survived to the final polish -- callers pooling multiple
    priors use it to attribute which prior won.
    """
    # Fine downsample once — used for both candidate scoring and the final ICP.
    src_fine = local_map.voxel_down_sample(FINE_VOXEL)
    src_fine.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=FINE_VOXEL * 2, max_nn=30)
    )
    tgt_fine = _global_fine(global_map, FINE_VOXEL)

    # Gravity filter; fall back to all if everything is tilted (degenerate clouds).
    # Every shipped fire pools ONE source, so a global gate IS the per-source gate.
    indexed = list(enumerate(candidates))
    upright = [item for item in indexed if _gravity_tilt_deg(item[1]) <= gravity_tilt_max_deg]
    pool = upright if upright else indexed

    # WALL-ONLY clouds for scoring + polish; the FULL clouds drive the final
    # refinement, preserving the gravity anchor and inlier density in the output.
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

    # Stage 2: moderate-distance ICP on each top-10, on WALL clouds -- wall
    # correspondences drive yaw and xy, so the winner is the one whose walls align,
    # not whose floors agree. Tukey biweight M-estimator down-weights far
    # correspondences so a few gross outliers can't drag the fit. https://www.open3d.org/docs/latest/tutorial/pipelines/robust_kinematics.html
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
    """Estimate the 4x4 transform placing ``local_map`` into ``global_map``.

    RANSAC candidate generation feeding the shared judge (``refine_candidates``).
    module.py's ``_relocalize`` relies on this (T, fitness) signature.
    ``gravity_tilt_max_deg`` defaults to the module constant, so callers that don't
    override it get today's behavior.
    (#2137's offline eval entrypoint shares only the (global_map, local_map)
    convention; it returns the bare 4x4, not this tuple.)
    """
    candidates = generate_ransac_candidates(global_map, local_map)
    T, fitness, _winning_index = refine_candidates(
        global_map,
        local_map,
        candidates,
        gravity_tilt_max_deg=gravity_tilt_max_deg,
    )
    return T, fitness

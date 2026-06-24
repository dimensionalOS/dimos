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

"""Pure pose/trajectory math for loop-closure evaluation.

Drift injection + correction and ground-truth(-free) trajectory metrics, all
operating on plain arrays and lookup callables (no db reads) so they can be
shared across the eval drivers.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.navigation.jnav.utils.voxel_map import VoxelMap

# (ts, x, y, z, qx, qy, qz, qw) per keyframe
GraphPose = tuple[float, float, float, float, float, float, float, float]
# time -> nearest pose: 3-vec [x,y,z] (PoseLookup) or 7-vec +quat (PoseLookup7)
PoseLookup = Callable[[float], "np.ndarray | None"]
PoseLookup7 = Callable[[float], "np.ndarray | None"]


def trajectory_lookup(times: np.ndarray, positions: np.ndarray, tolerance: float) -> PoseLookup:
    """A ``time -> [x, y, z]`` nearest-sample lookup over a (ts, position) trajectory."""
    if len(times) == 0:
        return lambda _timestamp: None

    def lookup(timestamp: float) -> np.ndarray | None:
        index = int(np.argmin(np.abs(times - timestamp)))
        if abs(float(times[index]) - timestamp) > tolerance:
            return None
        return np.asarray(positions[index], dtype=np.float64)

    return lookup


def graph_lookup(graph: list[tuple[float, float, float, float]]) -> PoseLookup:
    """Nearest-keyframe lookup over an optimized pose graph (no tolerance).

    Keyframes only spawn on motion, so a parked robot legitimately maps to an
    old keyframe — that keyframe IS its position (within the keyframe delta).
    Unbounded nearest-in-time is sound when the graph is never truncated."""
    times = np.asarray([node[0] for node in graph], dtype=np.float64)
    positions = np.asarray([[node[1], node[2], node[3]] for node in graph], dtype=np.float64)
    return trajectory_lookup(times, positions, float("inf"))


# Voxel-agreement sampling: cap per-scan points so the map fits in memory
# without holding the whole recording at once.
VOXEL_SIZE_M = 0.2
VOXEL_MAX_POINTS_PER_SCAN = 4000


def drift_offset(
    timestamp: float, t0: float, drift_per_sec: list[float] | np.ndarray
) -> np.ndarray:
    """World translation injected at ``timestamp`` (grows linearly from ``t0``)."""
    return np.asarray(drift_per_sec, dtype=np.float64) * (timestamp - t0)


def has_drift(drift_per_sec: list[float] | np.ndarray) -> bool:
    return bool(np.any(np.asarray(drift_per_sec, dtype=np.float64)))


def drifted_lookup(
    base_lookup: Callable[[float], np.ndarray | None],
    drift_per_sec: list[float],
    drift_t0: float,
) -> Callable[[float], np.ndarray | None]:
    """Wrap a pose lookup so its xyz gets the same drift the module was fed.

    Pass-through when drift is zero. Works for both the 3-vec (xyz) and 7-vec
    (xyz+quat) lookups — only the first three components are shifted."""
    if not has_drift(drift_per_sec):
        return base_lookup
    drift = np.asarray(drift_per_sec, dtype=np.float64)

    def lookup(timestamp: float) -> np.ndarray | None:
        pose = base_lookup(timestamp)
        if pose is None:
            return None
        shifted = np.array(pose, dtype=np.float64)
        shifted[:3] += drift * (timestamp - drift_t0)
        return shifted

    return lookup


def pose7_lookup(times: np.ndarray, poses: np.ndarray, tolerance: float) -> PoseLookup7:
    """time -> nearest [x,y,z,qx,qy,qz,qw], or None past the tolerance."""

    def lookup(timestamp: float) -> np.ndarray | None:
        if len(times) == 0:
            return None
        index = int(np.argmin(np.abs(times - timestamp)))
        if abs(float(times[index]) - timestamp) > tolerance:
            return None
        return np.asarray(poses[index], dtype=np.float64)

    return lookup


def drift_delta_lookup(
    graph: list[GraphPose], raw_lookup: PoseLookup7
) -> Callable[[float], tuple[np.ndarray, np.ndarray] | None]:
    """time -> nearest keyframe's drift correction (R_delta, t_delta).

    The delta is computed at the KEYFRAME's own timestamp —
    T_corrected(kf) * T_raw(kf_ts)^-1 — so raw and corrected poses describe the
    same instant. (Comparing a keyframe pose against the raw pose at a nearby
    scan's timestamp snaps scans onto keyframes and fakes a tighter map: an
    identity correction must yield an identity delta.)"""
    times: list[float] = []
    rotations: list[np.ndarray] = []
    translations: list[np.ndarray] = []
    for node in graph:
        raw_pose = raw_lookup(node[0])
        if raw_pose is None:
            continue
        rotation_raw = Rotation.from_quat(raw_pose[3:7]).as_matrix()
        rotation_corrected = Rotation.from_quat(node[4:8]).as_matrix()
        rotation_delta = rotation_corrected @ rotation_raw.T
        translation_delta = np.asarray(node[1:4]) - rotation_delta @ raw_pose[:3]
        times.append(node[0])
        rotations.append(rotation_delta)
        translations.append(translation_delta)
    times_array = np.asarray(times, dtype=np.float64)

    def lookup(timestamp: float) -> tuple[np.ndarray, np.ndarray] | None:
        if len(times_array) == 0:
            return None
        index = int(np.argmin(np.abs(times_array - timestamp)))
        return rotations[index], translations[index]

    return lookup


def rigid_align_rmse(source: np.ndarray, target: np.ndarray) -> float:
    """Absolute trajectory error: RMSE of ``source`` to ``target`` after a
    best-fit rigid (rotation+translation, no scale) alignment (Kabsch/Umeyama).
    Both are (N, 3). The alignment removes the gauge freedom — two trajectories
    of the same shape in different world frames score 0."""
    if len(source) < 3:
        return 0.0
    source_centroid = source.mean(axis=0)
    target_centroid = target.mean(axis=0)
    source_centered = source - source_centroid
    target_centered = target - target_centroid
    covariance = source_centered.T @ target_centered
    u_matrix, _, vt_matrix = np.linalg.svd(covariance)
    # Reflection fix so the result is a proper rotation (det = +1).
    reflection = np.sign(np.linalg.det(vt_matrix.T @ u_matrix.T))
    correction = np.diag([1.0, 1.0, reflection])
    rotation = vt_matrix.T @ correction @ u_matrix.T
    aligned = (rotation @ source_centered.T).T + target_centroid
    return float(np.sqrt(np.mean(np.sum((aligned - target) ** 2, axis=1))))


def trajectory_recovery_error(
    graph: list[GraphPose],
    gt_lookup: Callable[[float], np.ndarray | None],
    drift_per_sec: list[float],
    drift_t0: float,
) -> dict[str, float] | None:
    """Drift-recovery ATE: how close the module's corrected keyframe trajectory
    gets to the *un-drifted* ground-truth, vs the drifted input it was given.

    Only meaningful with injected drift (then GT = the un-drifted odom the drift
    was added to). This is the right metric where tag/voxel agreement is weak —
    e.g. KITTI's long single-loop trajectory. Returns None when drift is off or
    too few keyframes resolve. ``trajectory_improvement`` = fraction of the drift
    ATE removed (1.0 = perfect recovery, 0 = no help, negative = worse)."""
    if not has_drift(drift_per_sec):
        return None
    drift = np.asarray(drift_per_sec, dtype=np.float64)
    corrected_points: list[list[float]] = []
    gt_points: list[np.ndarray] = []
    drifted_points: list[np.ndarray] = []
    for node in graph:
        timestamp = node[0]
        gt_pose = gt_lookup(timestamp)
        if gt_pose is None:
            continue
        gt_xyz = np.asarray(gt_pose, dtype=np.float64)[:3]
        corrected_points.append([node[1], node[2], node[3]])
        gt_points.append(gt_xyz)
        drifted_points.append(gt_xyz + drift * (timestamp - drift_t0))
    if len(gt_points) < 3:
        return None
    gt_array = np.asarray(gt_points)
    drifted_ate = rigid_align_rmse(np.asarray(drifted_points), gt_array)
    corrected_ate = rigid_align_rmse(np.asarray(corrected_points), gt_array)
    improvement = (drifted_ate - corrected_ate) / drifted_ate if drifted_ate > 1e-9 else 0.0
    return {
        "drifted_ate_m": drifted_ate,
        "corrected_ate_m": corrected_ate,
        "trajectory_improvement": improvement,
    }


def lidar_voxel_agreement(
    scans: Iterable[tuple[float, np.ndarray]],
    raw_lookup: PoseLookup7,
    graph: list[GraphPose],
    *,
    voxel_size: float = VOXEL_SIZE_M,
    max_points_per_scan: int = VOXEL_MAX_POINTS_PER_SCAN,
    drift_per_sec: list[float] | None = None,
    drift_t0: float = 0.0,
) -> dict[str, Any]:
    """Occupied-voxel counts of the lidar map, raw vs corrected.

    ``scans`` yields ``(timestamp, points)`` where ``points`` is an (N, 3+) array
    registered in the raw odom world frame. Each scan is re-anchored by its
    nearest keyframe's drift correction (see `drift_delta_lookup`), so a good
    correction collapses doubled walls and the corrected map occupies fewer
    voxels. ``improvement`` is the fractional voxel drop (positive = tighter).
    When ``drift_per_sec`` is set the raw scans are first shifted into the same
    drifted world the module solved in."""
    drift = np.asarray(drift_per_sec or [0.0, 0.0, 0.0], dtype=np.float64)
    apply_drift = has_drift(drift)
    delta_lookup = drift_delta_lookup(graph, raw_lookup)
    raw_clouds: list[np.ndarray] = []
    corrected_clouds: list[np.ndarray] = []
    used = 0
    for timestamp, scan_points in scans:
        delta = delta_lookup(timestamp)
        if delta is None or raw_lookup(timestamp) is None:
            continue
        rotation_delta, translation_delta = delta
        points = np.asarray(scan_points, dtype=np.float64)[:, :3]
        if len(points) > max_points_per_scan:
            points = points[:: -(-len(points) // max_points_per_scan)]
        if apply_drift:
            points = points + drift * (timestamp - drift_t0)
        raw_clouds.append(points)
        corrected_clouds.append(points @ rotation_delta.T + translation_delta)
        used += 1

    if not raw_clouds:
        return {"status": "skipped: no scans matched both pose sources"}
    raw_voxels = VoxelMap.from_points(np.vstack(raw_clouds), voxel_size).count
    corrected_voxels = VoxelMap.from_points(np.vstack(corrected_clouds), voxel_size).count
    improvement = (raw_voxels - corrected_voxels) / raw_voxels if raw_voxels else 0.0
    return {
        "status": "ok",
        "raw_voxels": raw_voxels,
        "corrected_voxels": corrected_voxels,
        "improvement": improvement,
        "voxel_size_m": voxel_size,
        "scans_used": used,
    }

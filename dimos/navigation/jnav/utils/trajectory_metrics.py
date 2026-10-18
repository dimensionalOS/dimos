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
# time -> nearest pose: 3-vec [x,y,z] (PoseLookup) or 7-vec +quat (PoseQuatLookup)
PoseLookup = Callable[[float], "np.ndarray | None"]
PoseQuatLookup = Callable[[float], "np.ndarray | None"]
# time -> nearest keyframe's drift correction (R_delta, t_delta)
DeltaLookup = Callable[[float], "tuple[np.ndarray, np.ndarray] | None"]


def matrix_from_pose(pose: np.ndarray) -> np.ndarray:
    """4x4 homogeneous transform from a [x, y, z, qx, qy, qz, qw] pose."""
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(pose[3:7]).as_matrix()
    matrix[:3, 3] = pose[:3]
    return matrix


def nearest_index(times: np.ndarray, timestamp: float) -> int:
    """Index of the sample nearest in time (``times`` sorted ascending)."""
    index = int(np.searchsorted(times, timestamp))
    index = min(max(index, 0), len(times) - 1)
    if index > 0 and abs(float(times[index - 1]) - timestamp) < abs(
        float(times[index]) - timestamp
    ):
        index -= 1
    return index


def thin_pairs_by_path_section(
    pairs: Iterable[tuple[int, int]], positions: np.ndarray, section_length: float
) -> list[tuple[int, int]]:
    """Keep at most one index pair per ``section_length`` meters of path.

    ``positions`` (N, 3) are poses in path order; each pose is bucketed by cumulative arc
    length, and every bucket may participate in only one kept pair — greedy in the given
    pair order, so sort ``pairs`` best-first. ``section_length <= 0`` keeps everything.
    """
    pairs = list(pairs)
    if section_length <= 0:
        return pairs
    steps = np.linalg.norm(np.diff(np.asarray(positions, float), axis=0), axis=1)
    arc_length = np.concatenate([[0.0], np.cumsum(steps)])
    sections = (arc_length // section_length).astype(int)
    used: set[int] = set()
    kept: list[tuple[int, int]] = []
    for first, second in pairs:
        if int(sections[first]) in used or int(sections[second]) in used:
            continue
        used.add(int(sections[first]))
        used.add(int(sections[second]))
        kept.append((first, second))
    return kept


# Voxel-agreement sampling: cap per-scan points so the map fits in memory
# without holding the whole recording at once.
VOXEL_SIZE_M = 0.2
VOXEL_MAX_POINTS_PER_SCAN = 4000


def pose_lookup(times: np.ndarray, poses: np.ndarray, tolerance: float) -> PoseQuatLookup:
    """time -> nearest [x,y,z,qx,qy,qz,qw], or None past the tolerance."""

    def lookup(timestamp: float) -> np.ndarray | None:
        if len(times) == 0:
            return None
        index = int(np.argmin(np.abs(times - timestamp)))
        if abs(float(times[index]) - timestamp) > tolerance:
            return None
        return np.asarray(poses[index], dtype=np.float64)

    return lookup


def drift_delta_lookup(graph: list[GraphPose], raw_lookup: PoseQuatLookup) -> DeltaLookup:
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


def deform_path(
    times: np.ndarray, poses: np.ndarray, delta_lookup: DeltaLookup
) -> tuple[np.ndarray, np.ndarray]:
    """Raw odom trajectory + its plain-stretch corrected counterpart (Nx3 each)."""
    raw_path = poses[:, :3].astype(np.float64)
    corrected_points = []
    kept_raw = []
    for timestamp, position in zip(times, raw_path, strict=True):
        delta = delta_lookup(float(timestamp))
        if delta is None:
            continue
        rotation_delta, translation_delta = delta
        corrected_points.append(rotation_delta @ position + translation_delta)
        kept_raw.append(position)
    if not corrected_points:
        return raw_path, np.empty((0, 3))
    return np.array(kept_raw), np.array(corrected_points)


def lidar_voxel_agreement(
    scans: Iterable[tuple[float, np.ndarray]],
    raw_lookup: PoseQuatLookup,
    graph: list[GraphPose],
    *,
    voxel_size: float = VOXEL_SIZE_M,
    max_points_per_scan: int = VOXEL_MAX_POINTS_PER_SCAN,
) -> dict[str, Any]:
    """Occupied-voxel counts of the lidar map, raw vs corrected.

    ``scans`` yields ``(timestamp, points)`` where ``points`` is an (N, 3+) array
    registered in the raw odom world frame. Each scan is re-anchored by its
    nearest keyframe's drift correction (see `drift_delta_lookup`), so a good
    correction collapses doubled walls and the corrected map occupies fewer
    voxels. ``improvement`` is the fractional voxel drop (positive = tighter)."""
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

# Copyright 2025-2026 Dimensional Inc.
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

"""Map-building utilities: constants, voxel packing, floor calibration, raycasting."""

from __future__ import annotations

import time

import numpy as np
from scipy.ndimage import sobel
from scipy.spatial import cKDTree

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DEPTH_MM_THRESHOLD = 100
MILLIMETERS_PER_METER = 1000.0

VOFF  = np.int64(100_000)
VMASK = np.int64(0x3FFFF)

# Optical frame (X=right, Y=down, Z=depth) → camera_link (X=fwd, Y=left, Z=up)
R_OPT_TO_LINK = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float32)

RAYCAST_MIN_RAY_VOXELS = 2
# Voxels of margin held back from each ray's surface hit before it's allowed to
# clear existing map voxels as "free space". Was 1.5 (~3cm at 2cm voxels) when
# raycasting was first tried and removed for carving away thin real obstacles —
# too thin a margin lets a ray aimed at a wall behind a chair leg clear the leg
# itself. 4.0 (~8cm) matches the fat-voxel insertion guard's own tolerance.
RAYCAST_SURFACE_MARGIN = 4.0

# Full 9×9×9 neighbourhood (dx,dy,dz ∈ [-4..4], 729 entries).
# ±4 voxels = ±8 cm per axis. Rationale:
#   - Centroid ICP against a slightly noisy map gives ~7-8 cm depth error per step.
#   - Madgwick lag during rotation at 2 m range reaches ~7 cm for 2° orientation error.
#   - EMA steady-state lag at 8 cm keyframe step ≈ 3.4 cm (adds in depth direction).
#   - A noisy accumulated map makes ICP worse → more layers → even worse ICP (feedback).
#   - Blocking new observations within 8 cm of existing keeps the map single-layer,
#     which keeps the ICP reference clean and breaks the degradation feedback loop.
FAT_SHIFTS = np.array(
    [(dx * (1 << 36)) + (dy * (1 << 18)) + dz
     for dx in range(-4, 5)
     for dy in range(-4, 5)
     for dz in range(-4, 5)],
    dtype=np.int64,
)


def _pack(vkeys: np.ndarray) -> np.ndarray:
    v = (vkeys.astype(np.int64) + VOFF) & VMASK
    return (v[:, 0] << np.int64(36)) | (v[:, 1] << np.int64(18)) | v[:, 2]


def _voxel_dedup(points: np.ndarray, vox_size: float) -> np.ndarray:
    """Keep one point per occupied voxel (first occurrence)."""
    voxel_indices = np.floor(points / vox_size).astype(np.int32)
    _, first_index_per_voxel = np.unique(_pack(voxel_indices), return_index=True)
    return points[first_index_per_voxel]


def _gradient_mask(depth: np.ndarray, threshold: float) -> np.ndarray:
    valid   = np.isfinite(depth)
    depth_f = np.where(valid, depth, 0.0).astype(np.float64)
    grad    = np.hypot(sobel(depth_f, axis=1), sobel(depth_f, axis=0))
    return valid & (grad < threshold)  # type: ignore[no-any-return]


def _preprocess_depth(raw_depth: np.ndarray, min_depth: float, max_depth: float) -> np.ndarray:
    """Normalize to meters (mm heuristic) and NaN-out invalid/out-of-range pixels."""
    depth = raw_depth
    if hasattr(depth, "get"):
        depth = depth.get()
    if depth.ndim == 3:
        depth = depth[:, :, 0]
    depth = depth.astype(np.float32)
    valid_depth = depth[depth > 0]
    is_millimeters = len(valid_depth) > 0 and np.median(valid_depth) > DEPTH_MM_THRESHOLD
    if is_millimeters:
        depth /= MILLIMETERS_PER_METER
    out_of_range = (depth <= 0) | (depth < min_depth) | (depth > max_depth)
    depth[out_of_range] = np.nan
    return depth


def _intrinsics_from_camera_info(
    camera_info: object | None, height: int, width: int
) -> tuple[float, float, float, float]:
    """Returns (focal_x, focal_y, principal_x, principal_y); a rough guess if camera_info is None."""
    if camera_info is not None:
        k_matrix = camera_info.get_K_matrix()  # type: ignore[attr-defined]
        return (
            float(k_matrix[0, 0]),
            float(k_matrix[1, 1]),
            float(k_matrix[0, 2]),
            float(k_matrix[1, 2]),
        )
    focal = float(max(height, width)) / 2.0
    return focal, focal, width / 2.0, height / 2.0


def _unproject(
    depth: np.ndarray,
    valid: np.ndarray,
    principal_x: float,
    principal_y: float,
    focal_x: float,
    focal_y: float,
) -> np.ndarray:
    """Valid depth pixels -> optical-frame XYZ (X=right, Y=down, Z=depth)."""
    height, width = depth.shape
    u_grid, v_grid = np.meshgrid(np.arange(width, dtype=np.float32), np.arange(height, dtype=np.float32))
    depth_values = depth[valid]
    return np.column_stack([
        (u_grid[valid] - principal_x) * depth_values / focal_x,
        (v_grid[valid] - principal_y) * depth_values / focal_y,
        depth_values,
    ]).astype(np.float32)


def _format_perf_log(
    img_ts: float,
    t_start: float,
    t_gradient: float,
    t_unproject: float,
    t_odom_start: float,
    t_odom_end: float,
    t_publish: float,
    t_end: float,
    point_count: int,
) -> str:
    return (
        f"StereoPointCloud perf: lag={time.time() - img_ts:.1f}s "
        f"total={(t_end - t_start) * 1000:.0f}ms "
        f"gradient={(t_gradient - t_start) * 1000:.0f}ms "
        f"unproject={(t_unproject - t_gradient) * 1000:.0f}ms "
        f"odom={(t_odom_end - t_odom_start) * 1000:.0f}ms "
        f"publish={(t_publish - t_odom_end) * 1000:.0f}ms "
        f"traj={(t_end - t_publish) * 1000:.0f}ms "
        f"points={point_count}"
    )


def _isolation_filter(pts: np.ndarray, radius: float, min_neighbors: int = 2) -> np.ndarray:
    """Keep only points that have at least min_neighbors within radius — kills flying pixels."""
    if len(pts) <= min_neighbors:
        return np.ones(len(pts), dtype=bool)
    counts = cKDTree(pts).query_ball_point(pts, r=radius, return_length=True)
    return np.asarray(counts) > min_neighbors


def _raycast_free_keys(surface_pts: np.ndarray, vox_size: float, n_rays: int = 400, origin: np.ndarray | None = None) -> np.ndarray:
    """Free-space voxel keys on rays from origin to surface_pts (world frame)."""
    origin = np.zeros(3, dtype=np.float32) if origin is None else np.asarray(origin, dtype=np.float32)
    n = min(len(surface_pts), n_rays)
    if n == 0:
        return np.empty(0, dtype=np.int64)
    idx   = np.random.choice(len(surface_pts), n, replace=False) if len(surface_pts) > n else np.arange(n)
    pts   = surface_pts[idx]
    vecs  = pts - origin
    dists = np.linalg.norm(vecs, axis=1)
    ok    = dists > vox_size * RAYCAST_MIN_RAY_VOXELS
    vecs, dists = vecs[ok], dists[ok]
    if not len(vecs):
        return np.empty(0, dtype=np.int64)
    dirs      = vecs / dists[:, None]
    max_steps = int(np.ceil(dists.max() / vox_size))
    t_vals    = np.arange(max_steps, dtype=np.float32) * vox_size
    t_end     = (dists - vox_size * RAYCAST_SURFACE_MARGIN)[:, None]
    valid_m   = (t_vals[None, :] >= 0) & (t_vals[None, :] < t_end)
    ray_pts   = origin + dirs[:, None, :] * t_vals[None, :, None]
    vk_flat   = np.floor(ray_pts.reshape(-1, 3) / vox_size).astype(np.int32)
    return np.unique(_pack(vk_flat)[valid_m.ravel()])


class _FloorCalibrator:
    """Histogram-based one-shot floor height estimator (Z=up).

    Feed :meth:`update` with GRAVITY-ALIGNED camera-centered points
    (``xyz_world - t``, i.e. ``xyz_cam @ R.T``) — in that frame the floor is a
    horizontal plane and its z-histogram is sharp regardless of mount pitch.
    """

    SKIP_FRAMES      = 30
    CALIB_FRAMES     = 30
    BIN_M            = 0.02
    FLOOR_MAX_Z      = -0.30
    FLOOR_MIN_Z      = -3.0
    FLOOR_MAX_DIST_M = 4.0
    FLOOR_MIN_PTS    = 200
    # mode-search window above the lowest significant bin (3 bins = 6 cm)
    CLUSTER_BINS     = 3

    def __init__(self) -> None:
        self._frame   = 0
        self._samples: list[float] = []
        self.floor_z:    float | None = None
        self.cam_height: float | None = None

    @property
    def ready(self) -> bool:
        return self.floor_z is not None

    def update(self, xyz_cam: np.ndarray) -> None:
        self._frame += 1
        if self.ready or self._frame <= self.SKIP_FRAMES:
            return
        z    = xyz_cam[:, 2]
        dist = np.linalg.norm(xyz_cam[:, :2], axis=1)
        mask = (z < self.FLOOR_MAX_Z) & (z > self.FLOOR_MIN_Z) & (dist < self.FLOOR_MAX_DIST_M)
        if mask.sum() < self.FLOOR_MIN_PTS:
            return
        z_floor = z[mask]
        lo, hi  = z_floor.min(), z_floor.max()
        if hi - lo < self.BIN_M:
            return
        bins          = np.arange(lo, hi + self.BIN_M, self.BIN_M)
        counts, edges = np.histogram(z_floor, bins=bins)
        significant   = np.where(counts >= self.FLOOR_MIN_PTS)[0]
        if not len(significant):
            return
        # FIX: take the DOMINANT bin within the lowest significant cluster
        # instead of the lowest significant bin. The lowest bin sits on the
        # noise skirt of the floor peak (and is pulled further down by
        # hole-filling/spatial-filter artifacts below the true floor), which
        # biased the floor estimate 1-2 cm low — enough to leak floor points
        # past the global_floor_margin cut.
        window = significant[significant <= significant[0] + self.CLUSTER_BINS]
        best   = int(window[np.argmax(counts[window])])
        self._samples.append(float(edges[best] + self.BIN_M / 2))
        if len(self._samples) >= self.CALIB_FRAMES:
            self.floor_z    = float(np.median(self._samples))
            self.cam_height = -self.floor_z
            logger.info(f"StereoPointCloud: floor calibrated — camera {self.cam_height:.3f} m above floor")

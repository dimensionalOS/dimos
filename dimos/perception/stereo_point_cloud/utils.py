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

import numpy as np
from scipy.ndimage import sobel
from scipy.spatial import cKDTree

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_VOFF  = np.int64(100_000)
_VMASK = np.int64(0x3FFFF)

# Optical frame (X=right, Y=down, Z=depth) → camera_link (X=fwd, Y=left, Z=up)
_R_OPT_TO_LINK = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float32)

_RAYCAST_MIN_RAY_VOXELS = 2
_RAYCAST_SURFACE_MARGIN = 1.5

# Full 7×7×7 neighbourhood (dx,dy,dz ∈ [-3..3], 343 entries).
# Axis-aligned-only checks miss diagonal drift: a 2° Madgwick lag at 2 m places the
# same surface ~7 cm off in a combined x+y direction that no single-axis shift catches.
# ±3 voxels = ±6 cm per axis — catches diagonal drift up to ~10 cm, handles most
# Madgwick orientation error during typical rotation speeds.
_FAT_SHIFTS = np.array(
    [(dx * (1 << 36)) + (dy * (1 << 18)) + dz
     for dx in range(-3, 4)
     for dy in range(-3, 4)
     for dz in range(-3, 4)],
    dtype=np.int64,
)


def _pack(vkeys: np.ndarray) -> np.ndarray:
    v = (vkeys.astype(np.int64) + _VOFF) & _VMASK
    return (v[:, 0] << np.int64(36)) | (v[:, 1] << np.int64(18)) | v[:, 2]


def _gradient_mask(depth: np.ndarray, threshold: float) -> np.ndarray:
    valid   = np.isfinite(depth)
    depth_f = np.where(valid, depth, 0.0).astype(np.float64)
    grad    = np.hypot(sobel(depth_f, axis=1), sobel(depth_f, axis=0))
    return valid & (grad < threshold)


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
    ok    = dists > vox_size * _RAYCAST_MIN_RAY_VOXELS
    vecs, dists = vecs[ok], dists[ok]
    if not len(vecs):
        return np.empty(0, dtype=np.int64)
    dirs      = vecs / dists[:, None]
    max_steps = int(np.ceil(dists.max() / vox_size))
    t_vals    = np.arange(max_steps, dtype=np.float32) * vox_size
    t_end     = (dists - vox_size * _RAYCAST_SURFACE_MARGIN)[:, None]
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

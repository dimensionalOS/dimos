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

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_VOFF  = np.int64(100_000)
_VMASK = np.int64(0x3FFFF)

# Optical frame (X=right, Y=down, Z=depth) → camera_link (X=fwd, Y=left, Z=up)
_R_OPT_TO_LINK = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float32)

_RAYCAST_MIN_RAY_VOXELS = 2
_RAYCAST_SURFACE_MARGIN = 1.5


def _pack(vkeys: np.ndarray) -> np.ndarray:
    v = (vkeys.astype(np.int64) + _VOFF) & _VMASK
    return (v[:, 0] << np.int64(36)) | (v[:, 1] << np.int64(18)) | v[:, 2]


def _gradient_mask(depth: np.ndarray, threshold: float) -> np.ndarray:
    valid   = np.isfinite(depth)
    depth_f = np.where(valid, depth, 0.0).astype(np.float64)
    grad    = np.hypot(sobel(depth_f, axis=1), sobel(depth_f, axis=0))
    return valid & (grad < threshold)


def _raycast_free_keys(surface_pts: np.ndarray, vox_size: float, n_rays: int = 400) -> np.ndarray:
    """Free-space voxel keys on rays from camera origin (rotation-only frame) to surface_pts."""
    origin = np.zeros(3, dtype=np.float32)
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
    """Histogram-based one-shot floor height estimator in camera_link frame (Z=up)."""

    SKIP_FRAMES      = 30
    CALIB_FRAMES     = 30
    BIN_M            = 0.02
    FLOOR_MAX_Z      = -0.30
    FLOOR_MAX_DIST_M = 4.0
    FLOOR_MIN_PTS    = 200

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
        mask = (z < self.FLOOR_MAX_Z) & (dist < self.FLOOR_MAX_DIST_M)
        if mask.sum() < self.FLOOR_MIN_PTS:
            return
        z_floor = z[mask]
        lo, hi  = z_floor.min(), z_floor.max()
        if hi - lo < self.BIN_M:
            return
        bins          = np.arange(lo, hi + self.BIN_M, self.BIN_M)
        counts, edges = np.histogram(z_floor, bins=bins)
        peak          = np.argmax(counts)
        self._samples.append(float(edges[peak] + self.BIN_M / 2))
        if len(self._samples) >= self.CALIB_FRAMES:
            self.floor_z    = float(np.median(self._samples))
            self.cam_height = -self.floor_z
            logger.info(f"StereoPointCloud: floor calibrated — camera {self.cam_height:.3f} m above floor")

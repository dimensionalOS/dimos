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
    """Height-agnostic floor estimator (Z=up) — works for ANY mount height.

    Feed :meth:`update` with GRAVITY-ALIGNED camera-centered points
    (``xyz_world - t``, i.e. ``xyz_cam @ R.T``) — in that frame the floor is a
    horizontal plane and its z-histogram is sharp regardless of mount pitch.

    Per frame it takes the dominant bin of the LOWEST significant z-histogram
    cluster and then validates that the slice at that height is actually
    floor-like:

    - **area**: covered XY footprint ≥ MIN_AREA_M2 — rejects small horizontal
      patches (box tops, shelf edges);
    - **spread**: minor-axis std of the slice ≥ MIN_MINOR_STD — rejects thin
      lines, which is what the base of a WALL looks like in a z-slice (the
      false lock that plagued histogram-only calibration).

    Calibration completes when CALIB_FRAMES validated samples agree within
    STABLE_STD. Afterwards detection keeps running: if a stable validated
    plane appears more than RECAL_DROP *below* the current floor, it is
    adopted (the floor is by definition the lowest valid horizontal plane —
    this self-corrects a lock onto a table when the real floor was initially
    out of view). Upward revisions are never applied.
    """

    SKIP_FRAMES      = 15
    CALIB_FRAMES     = 30
    BIN_M            = 0.02
    MIN_CAM_HEIGHT   = 0.15   # supports low mounts (was fixed at 0.30)
    MAX_CAM_HEIGHT   = 4.0
    FLOOR_MAX_DIST_M = 6.0
    FLOOR_MIN_PTS    = 100
    # upward scan: window size (bins) and max slice validations per frame
    SCAN_WINDOW_BINS = 5   # 10 cm windows
    MAX_SLICE_TESTS  = 10
    # floor-likeness validation of the candidate z-slice
    SLICE_TOL        = 0.025  # slice half-thickness around candidate z
    MIN_AREA_M2      = 0.30   # min covered XY footprint (10 cm cells)
    MIN_MINOR_STD    = 0.10   # min minor-axis std — rejects line-like slices
    ABOVE_BAND_LO    = 0.12   # vertical-context band starts this far above z0
    # convergence / recalibration
    STABLE_STD       = 0.03
    RECAL_DROP       = 0.08

    def __init__(self) -> None:
        self._frame   = 0
        self._samples: list[float] = []
        self.floor_z:    float | None = None
        self.cam_height: float | None = None
        # diagnostic: floor-band candidate count of the last update()
        self.last_candidates: int = 0

    @property
    def ready(self) -> bool:
        return self.floor_z is not None

    def _detect(self, xyz_cam: np.ndarray) -> float | None:
        """Validated floor height for one frame, or None."""
        z    = xyz_cam[:, 2]
        dist = np.linalg.norm(xyz_cam[:, :2], axis=1)
        mask = (
            (z < -self.MIN_CAM_HEIGHT)
            & (z > -self.MAX_CAM_HEIGHT)
            & (dist < self.FLOOR_MAX_DIST_M)
        )
        self.last_candidates = int(mask.sum())
        if self.last_candidates < self.FLOOR_MIN_PTS:
            return None
        pts = xyz_cam[mask]
        zf  = pts[:, 2]
        lo, hi = float(zf.min()), float(zf.max())
        if hi - lo < self.BIN_M:
            return None
        bins          = np.arange(lo, hi + self.BIN_M, self.BIN_M)
        counts, edges = np.histogram(zf, bins=bins)
        significant   = np.where(counts >= self.FLOOR_MIN_PTS)[0]
        if not len(significant):
            return None
        # Scan upward from the LOWEST significant bin in fixed windows,
        # validating each window's dominant bin, and return the first z that
        # passes the floor-likeness checks. Windowing (rather than gap-based
        # clustering) matters because a wall produces significant bins at
        # EVERY height — merging everything into one cluster — while its
        # slices are rejected below; the first horizontal surface above it
        # (floor or table) must still be reachable. The test count is bounded.
        i = 0
        tested = 0
        while i < len(significant) and tested < self.MAX_SLICE_TESTS:
            w = significant[
                (significant >= significant[i])
                & (significant < significant[i] + self.SCAN_WINDOW_BINS)
            ]
            best = int(w[np.argmax(counts[w])])
            z0   = float(edges[best] + self.BIN_M / 2)
            if self._slice_is_floorlike(pts, z0):
                return z0
            tested += 1
            i = max(
                int(np.searchsorted(significant, significant[i] + self.SCAN_WINDOW_BINS)),
                i + 1,
            )
        return None

    def _slice_is_floorlike(self, pts: np.ndarray, z0: float) -> bool:
        """Is the z-slice at z0 an exposed horizontal surface (floor-like)?

        1. Vertical context: cells with points directly ABOVE them are
           removed first — that is what a wall looks like in any z-slice
           (its own column covers it from above), and it also strips wall
           contamination from a slice that crosses a real surface.
        2. The REMAINING open patch must be big (area) and 2-D (spread) —
           rejecting small horizontal patches (box tops) and line-like
           leftovers. The spread threshold scales with range because depth
           noise (~0.0035 d²) fattens a far wall's base line.
        """
        sl = pts[np.abs(pts[:, 2] - z0) < self.SLICE_TOL]
        if len(sl) < self.FLOOR_MIN_PTS:
            return False
        cell_idx = (sl[:, :2] / 0.1).astype(np.int32)
        cell_key = cell_idx[:, 0].astype(np.int64) * np.int64(1_000_003) + cell_idx[:, 1]
        # Above-band starts at +0.12 m so the surface's own depth noise (which
        # grows ~0.0035 d² at range) cannot mark its cells as covered, and a
        # cell needs ≥2 points above to count — single noise outliers don't
        # qualify, while a wall column always has plenty at every height.
        above = pts[(pts[:, 2] > z0 + self.ABOVE_BAND_LO) & (pts[:, 2] < z0 + 0.7)]
        if len(above):
            a_idx = (above[:, :2] / 0.1).astype(np.int32)
            a_all = a_idx[:, 0].astype(np.int64) * np.int64(1_000_003) + a_idx[:, 1]
            a_key, a_cnt = np.unique(a_all, return_counts=True)
            a_key = a_key[a_cnt >= 2]
            if len(a_key):
                open_mask = ~np.isin(cell_key, a_key)
                sl, cell_key = sl[open_mask], cell_key[open_mask]
        if len(sl) < self.FLOOR_MIN_PTS:
            return False
        if len(np.unique(cell_key)) * 0.01 < self.MIN_AREA_M2:
            return False
        xy  = sl[:, :2].astype(np.float64)
        c   = xy - xy.mean(axis=0)
        cov = (c.T @ c) / len(c)
        ev  = np.linalg.eigvalsh(cov)  # ascending
        # 1-sigma range scaling: the vertical-context subtraction above is the
        # primary wall rejector; this only needs to catch residual noise
        # lines without rejecting the thin grazing floor ring a HIGH-mounted
        # camera legitimately sees (the gradient mask limits far floor).
        d_med     = float(np.median(np.linalg.norm(sl[:, :2], axis=1)))
        minor_min = max(self.MIN_MINOR_STD, 0.0035 * d_med * d_med)
        return float(np.sqrt(max(ev[0], 0.0))) >= minor_min

    def update(self, xyz_cam: np.ndarray) -> None:
        self._frame += 1
        if self._frame <= self.SKIP_FRAMES or len(xyz_cam) == 0:
            return
        z0 = self._detect(xyz_cam)
        if z0 is None:
            return
        self._samples.append(z0)
        if len(self._samples) > self.CALIB_FRAMES:
            self._samples.pop(0)
        if len(self._samples) < self.CALIB_FRAMES:
            return
        s = np.asarray(self._samples)
        if float(s.std()) > self.STABLE_STD:
            return
        med = float(np.median(s))
        if not self.ready:
            self.floor_z    = med
            self.cam_height = -med
            logger.info(
                f"StereoPointCloud: floor calibrated — camera "
                f"{self.cam_height:.3f} m above floor"
            )
        elif med < self.floor_z - self.RECAL_DROP:
            # a lower stable validated plane — the previous lock was a raised
            # surface (e.g. a table); the real floor is below
            logger.warning(
                f"StereoPointCloud: found a lower floor plane "
                f"({-med:.3f} m vs {self.cam_height:.3f} m) — recalibrating"
            )
            self.floor_z    = med
            self.cam_height = -med

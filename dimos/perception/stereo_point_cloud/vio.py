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

"""Visual-inertial odometry: Madgwick AHRS + rotation-decoupled ICP."""

from __future__ import annotations

import threading

import numpy as np
from scipy.spatial import cKDTree


_MAX_DT_S        = 0.1   # don't integrate more than 100ms at once — big gaps would blow up the filter
_ACCEL_MIN_NORM  = 0.5   # skip accel correction if reading is basically zero (noise or free-fall)
_MIN_ICP_INLIERS = 30    # need at least 30 matched point pairs to trust the ICP translation update
_SMOOTH_ALPHA    = 0.7   # EMA weight for ICP output — reduces ±2cm centroid oscillation to ±5mm,
                          # keeping the same 2cm world-voxel key for a stationary surface.


class MadgwickFilter:
    """Gyro + accel → orientation R (camera_link → world, Z-up). beta=0.033."""

    def __init__(self, beta: float = 0.033) -> None:
        self._q      = np.array([1., 0., 0., 0.], dtype=np.float64)
        self._beta   = beta
        self._t_prev: float | None = None

    def update(self, gyro: np.ndarray, accel: np.ndarray, t: float) -> None:
        if self._t_prev is None:
            self._t_prev = t
            return
        dt = min(t - self._t_prev, _MAX_DT_S)
        self._t_prev = t
        if dt <= 0:
            return
        q0, q1, q2, q3 = self._q
        gx, gy, gz = gyro.astype(np.float64)
        a_n = np.linalg.norm(accel)
        if a_n > _ACCEL_MIN_NORM:
            ax, ay, az = accel.astype(np.float64) / a_n
            f1 = 2*(q1*q3 - q0*q2) - ax
            f2 = 2*(q0*q1 + q2*q3) - ay
            f3 = 2*(0.5 - q1**2 - q2**2) - az
            J  = np.array([
                [-2*q2,  2*q3, -2*q0, 2*q1],
                [ 2*q1,  2*q0,  2*q3, 2*q2],
                [    0, -4*q1, -4*q2,    0],
            ])
            grad = J.T @ np.array([f1, f2, f3])
            gn   = np.linalg.norm(grad)
            if gn > 1e-9:
                grad /= gn
        else:
            grad = np.zeros(4)
        q_dot = 0.5 * np.array([
            -q1*gx - q2*gy - q3*gz,
             q0*gx + q2*gz - q3*gy,
             q0*gy - q1*gz + q3*gx,
             q0*gz + q1*gy - q2*gx,
        ]) - self._beta * grad
        self._q = self._q + q_dot * dt
        self._q /= np.linalg.norm(self._q) + 1e-12

    @property
    def R(self) -> np.ndarray:
        q0, q1, q2, q3 = self._q
        return np.array([
            [1-2*(q2**2+q3**2), 2*(q1*q2-q0*q3),   2*(q1*q3+q0*q2)  ],
            [2*(q1*q2+q0*q3),   1-2*(q1**2+q3**2), 2*(q2*q3-q0*q1)  ],
            [2*(q1*q3-q0*q2),   2*(q2*q3+q0*q1),   1-2*(q1**2+q2**2)],
        ], dtype=np.float32)


class PointCloudOdometry:
    """Rotation-decoupled ICP for translation t. Takes R from Madgwick, converges in 3-4 iters.

    Owns a bounded, self-growing reference buffer (not just the last frame) so
    a single low-overlap frame can't permanently lose tracking.
    """

    ITERS        = 4
    MAX_DIST     = 0.40
    MIN_PTS      = 50       # minimum points needed to run ICP at all
    N_SRC        = 1_500
    N_DST        = 8_000
    REF_MAX_PTS  = 50_000        # cap on the reference buffer
    REF_STEP_M   = 0.08          # grow the reference buffer once moved this far...
    REF_STEP_RAD = np.deg2rad(8) # ...or rotated this much (pure rotation still needs a refresh)

    def __init__(self) -> None:
        self._t          = np.zeros(3, dtype=np.float32)
        self._lock       = threading.Lock()
        self._ref_pts: np.ndarray | None = None
        self._last_ref_t: np.ndarray | None = None
        self._last_ref_R: np.ndarray | None = None

    @property
    def t(self) -> np.ndarray:
        with self._lock:
            return self._t.copy()

    def update(self, xyz_cam: np.ndarray, R: np.ndarray) -> np.ndarray:
        pts_ga = (xyz_cam @ R.T).astype(np.float32)
        with self._lock:
            t_est = self._t.copy()
            ref   = self._ref_pts

        if ref is None or len(pts_ga) < self.MIN_PTS:
            t_new = t_est
        else:
            n_src = min(self.N_SRC, len(pts_ga))
            n_dst = min(self.N_DST, len(ref))
            src   = pts_ga[np.random.choice(len(pts_ga), n_src, replace=False)]
            dst   = ref[np.random.choice(len(ref), n_dst, replace=False)]
            tree  = cKDTree(dst)
            for _ in range(self.ITERS):
                dists, idx = tree.query(src + t_est, k=1, workers=1)
                mask = dists < self.MAX_DIST
                if mask.sum() < _MIN_ICP_INLIERS:
                    break
                delta = dst[idx[mask]].mean(axis=0) - (src[mask] + t_est).mean(axis=0)
                t_est = (t_est + 0.7 * delta).astype(np.float32)
            t_new = t_est

        with self._lock:
            t_smooth = (_SMOOTH_ALPHA * t_new + (1.0 - _SMOOTH_ALPHA) * self._t).astype(np.float32)
            self._t  = t_smooth

        moved = self._last_ref_t is None or float(np.linalg.norm(t_smooth - self._last_ref_t)) > self.REF_STEP_M
        if not moved and self._last_ref_R is not None:
            cos_a = float((np.trace(R @ self._last_ref_R.T) - 1.0) / 2.0)
            moved = float(np.arccos(np.clip(cos_a, -1.0, 1.0))) > self.REF_STEP_RAD

        if moved:
            world_pts = (pts_ga + t_smooth).astype(np.float32)
            with self._lock:
                self._last_ref_t = t_smooth.copy()
                self._last_ref_R = R.copy()
                self._ref_pts = (
                    np.vstack([self._ref_pts, world_pts]) if self._ref_pts is not None else world_pts.copy()
                )
                if len(self._ref_pts) > self.REF_MAX_PTS:
                    keep = np.random.choice(len(self._ref_pts), self.REF_MAX_PTS, replace=False)
                    self._ref_pts = self._ref_pts[keep]

        return t_smooth

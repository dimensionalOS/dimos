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


_MAX_DT_S        = 0.1  # don't integrate more than 100ms at once — big gaps would blow up the filter
_ACCEL_MIN_NORM  = 0.5  # skip accel correction if reading is basically zero (noise or free-fall)
_MIN_ICP_INLIERS = 30   # need at least 30 matched point pairs to trust the ICP translation update


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
    """Rotation-decoupled ICP for translation t. Takes R from Madgwick, converges in 3-4 iters."""

    ITERS    = 4
    MAX_DIST = 0.40
    MIN_PTS  = 50     # minimum points needed to run ICP at all
    N_SRC    = 1_500
    N_DST    = 8_000
    N_STORE  = 10_000

    def __init__(self) -> None:
        self._t          = np.zeros(3, dtype=np.float32)
        self._lock       = threading.Lock()
        self._prev_world: np.ndarray | None = None

    @property
    def t(self) -> np.ndarray:
        with self._lock:
            return self._t.copy()

    def update(self, xyz_cam: np.ndarray, R: np.ndarray, map_ref: np.ndarray | None = None) -> np.ndarray:
        pts_ga = (xyz_cam @ R.T).astype(np.float32)
        with self._lock:
            t_est = self._t.copy()
        # Prefer the accumulated map as ICP reference — it covers the full explored area
        # and gives far better convergence than a single previous frame, especially when
        # the camera moves to a new region with no overlap to the last frame.
        ref = map_ref if (map_ref is not None and len(map_ref) >= self.MIN_PTS) else self._prev_world
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
        n_st  = min(self.N_STORE, len(pts_ga))
        idx_s = np.random.choice(len(pts_ga), n_st, replace=False)
        self._prev_world = (pts_ga[idx_s] + t_new).astype(np.float32)
        with self._lock:
            self._t = t_new
        return t_new

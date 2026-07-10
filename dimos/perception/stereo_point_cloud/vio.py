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

import bisect
import threading
import time
from collections import deque

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp

from dimos.perception.stereo_point_cloud.utils import _pack


_MAX_DT_S        = 0.1   # don't integrate more than 100ms at once — big gaps would blow up the filter
_ACCEL_MIN_NORM  = 0.5   # skip accel correction if reading is basically zero (noise or free-fall)
_MIN_ICP_INLIERS = 30    # need at least 30 matched point pairs to trust the ICP translation update
_SMOOTH_ALPHA    = 0.7   # EMA weight for ICP output — reduces ±2cm centroid oscillation to ±5mm
_ICP_STEP_GAIN   = 0.7   # fraction of each iteration's centroid offset applied per step


def _quat_wxyz_to_xyzw(q: np.ndarray) -> list[float]:
    return [q[1], q[2], q[3], q[0]]


class MadgwickFilter:
    """Gyro + accel → orientation R (camera_link → world, Z-up). beta=0.033.

    dt integration uses the IMU's own hardware timestamps (self-consistent).
    History for R_at() is indexed separately by wall-clock time, since callers
    (e.g. a depth frame's img.ts) are timestamped on that clock, not the IMU's.
    """

    HISTORY_LEN = 100  # ~250ms at 400Hz gyro — enough to bracket any depth-frame latency

    def __init__(self, beta: float = 0.033) -> None:
        self._quat        = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self._beta         = beta
        self._prev_time_s: float | None = None
        self._history: deque[tuple[float, np.ndarray]] = deque(maxlen=self.HISTORY_LEN)

    def update(self, gyro: np.ndarray, accel: np.ndarray, time_s: float) -> None:
        if self._prev_time_s is None:
            self._prev_time_s = time_s
            self._history.append((time.time(), self._quat.copy()))
            return
        dt = min(time_s - self._prev_time_s, _MAX_DT_S)
        self._prev_time_s = time_s
        if dt <= 0:
            return

        q0, q1, q2, q3 = self._quat
        gyro_x, gyro_y, gyro_z = gyro.astype(np.float64)
        accel_norm = np.linalg.norm(accel)
        if accel_norm > _ACCEL_MIN_NORM:
            accel_x, accel_y, accel_z = accel.astype(np.float64) / accel_norm
            f1 = 2 * (q1 * q3 - q0 * q2) - accel_x
            f2 = 2 * (q0 * q1 + q2 * q3) - accel_y
            f3 = 2 * (0.5 - q1**2 - q2**2) - accel_z
            jacobian = np.array([
                [-2 * q2,  2 * q3, -2 * q0, 2 * q1],
                [ 2 * q1,  2 * q0,  2 * q3, 2 * q2],
                [      0, -4 * q1, -4 * q2,      0],
            ])
            gradient = jacobian.T @ np.array([f1, f2, f3])
            gradient_norm = np.linalg.norm(gradient)
            if gradient_norm > 1e-9:
                gradient /= gradient_norm
        else:
            gradient = np.zeros(4)

        quat_dot = 0.5 * np.array([
            -q1 * gyro_x - q2 * gyro_y - q3 * gyro_z,
             q0 * gyro_x + q2 * gyro_z - q3 * gyro_y,
             q0 * gyro_y - q1 * gyro_z + q3 * gyro_x,
             q0 * gyro_z + q1 * gyro_y - q2 * gyro_x,
        ]) - self._beta * gradient
        self._quat = self._quat + quat_dot * dt
        self._quat /= np.linalg.norm(self._quat) + 1e-12
        self._history.append((time.time(), self._quat.copy()))

    @property
    def R(self) -> np.ndarray:
        return self._to_matrix(self._quat)

    def R_at(self, wall_clock_ts: float) -> np.ndarray:
        """Orientation SLERP-interpolated to a past wall-clock timestamp."""
        if len(self._history) < 2:
            return self.R
        timestamps = [sample[0] for sample in self._history]
        if wall_clock_ts <= timestamps[0]:
            return self._to_matrix(self._history[0][1])
        if wall_clock_ts >= timestamps[-1]:
            return self._to_matrix(self._history[-1][1])
        i = bisect.bisect_left(timestamps, wall_clock_ts)
        t0, q0 = self._history[i - 1]
        t1, q1 = self._history[i]
        if t1 <= t0:
            return self._to_matrix(q1)
        rotations = Rotation.from_quat([_quat_wxyz_to_xyzw(q0), _quat_wxyz_to_xyzw(q1)])
        return Slerp([t0, t1], rotations)([wall_clock_ts]).as_matrix()[0].astype(np.float32)

    @staticmethod
    def _to_matrix(q: np.ndarray) -> np.ndarray:
        q0, q1, q2, q3 = q
        return np.array([
            [1 - 2 * (q2**2 + q3**2), 2 * (q1 * q2 - q0 * q3),     2 * (q1 * q3 + q0 * q2)  ],
            [2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1**2 + q3**2),     2 * (q2 * q3 - q0 * q1)  ],
            [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1),     1 - 2 * (q1**2 + q2**2)  ],
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
    REF_MAX_PTS  = 50_000        # ring buffer capacity
    REF_STEP_M   = 0.08          # grow the reference buffer once moved this far...
    REF_STEP_RAD = np.deg2rad(8) # ...or rotated this much (pure rotation still needs a refresh)
    REF_VOX_SIZE = 0.05          # dedup growth batches to this voxel size — matching doesn't need mm precision

    def __init__(self) -> None:
        self._t              = np.zeros(3, dtype=np.float32)
        self._lock            = threading.Lock()
        self._ref_buffer      = np.zeros((self.REF_MAX_PTS, 3), dtype=np.float32)
        self._ref_write_index = 0
        self._ref_filled      = 0
        self._last_ref_t: np.ndarray | None = None
        self._last_ref_R: np.ndarray | None = None

    @property
    def t(self) -> np.ndarray:
        with self._lock:
            return self._t.copy()

    def update(self, xyz_cam: np.ndarray, R: np.ndarray) -> np.ndarray:
        points_gravity_aligned = (xyz_cam @ R.T).astype(np.float32)
        with self._lock:
            t_estimate = self._t.copy()
            # View, not copy: update() is only ever called serially, and the only
            # writer to _ref_buffer is this same call's growth step below, which
            # runs after this read completes.
            reference_points = self._ref_buffer[: self._ref_filled] if self._ref_filled else None

        if reference_points is None or len(points_gravity_aligned) < self.MIN_PTS:
            t_new = t_estimate
        else:
            source_count = min(self.N_SRC, len(points_gravity_aligned))
            dest_count   = min(self.N_DST, len(reference_points))
            source_points = points_gravity_aligned[
                np.random.choice(len(points_gravity_aligned), source_count, replace=False)
            ]
            dest_points = reference_points[
                np.random.choice(len(reference_points), dest_count, replace=False)
            ]
            tree = cKDTree(dest_points)
            for _ in range(self.ITERS):
                distances, nearest_index = tree.query(source_points + t_estimate, k=1, workers=1)
                inliers = distances < self.MAX_DIST
                if inliers.sum() < _MIN_ICP_INLIERS:
                    break
                centroid_offset = (
                    dest_points[nearest_index[inliers]].mean(axis=0)
                    - (source_points[inliers] + t_estimate).mean(axis=0)
                )
                t_estimate = (t_estimate + _ICP_STEP_GAIN * centroid_offset).astype(np.float32)
            t_new = t_estimate

        with self._lock:
            t_smooth = (_SMOOTH_ALPHA * t_new + (1.0 - _SMOOTH_ALPHA) * self._t).astype(np.float32)
            self._t  = t_smooth

        moved = self._last_ref_t is None or float(np.linalg.norm(t_smooth - self._last_ref_t)) > self.REF_STEP_M
        if not moved and self._last_ref_R is not None:
            cos_angle = float((np.trace(R @ self._last_ref_R.T) - 1.0) / 2.0)
            moved = float(np.arccos(np.clip(cos_angle, -1.0, 1.0))) > self.REF_STEP_RAD

        if moved:
            world_points = (points_gravity_aligned + t_smooth).astype(np.float32)
            voxel_indices = np.floor(world_points / self.REF_VOX_SIZE).astype(np.int32)
            _, first_index_per_voxel = np.unique(_pack(voxel_indices), return_index=True)
            growth_batch = world_points[first_index_per_voxel]
            with self._lock:
                self._last_ref_t = t_smooth.copy()
                self._last_ref_R = R.copy()
                self._write_ring_buffer(growth_batch)

        return t_smooth

    def _write_ring_buffer(self, batch: np.ndarray) -> None:
        batch_size = len(batch)
        if batch_size == 0:
            return
        if batch_size >= self.REF_MAX_PTS:
            self._ref_buffer[:] = batch[-self.REF_MAX_PTS:]
            self._ref_write_index = 0
            self._ref_filled = self.REF_MAX_PTS
            return
        end_index = self._ref_write_index + batch_size
        if end_index <= self.REF_MAX_PTS:
            self._ref_buffer[self._ref_write_index : end_index] = batch
        else:
            first_segment_size = self.REF_MAX_PTS - self._ref_write_index
            self._ref_buffer[self._ref_write_index :] = batch[:first_segment_size]
            self._ref_buffer[: end_index - self.REF_MAX_PTS] = batch[first_segment_size:]
        self._ref_write_index = end_index % self.REF_MAX_PTS
        self._ref_filled = min(self._ref_filled + batch_size, self.REF_MAX_PTS)

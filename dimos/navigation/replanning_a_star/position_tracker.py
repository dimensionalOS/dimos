# Copyright 2025 Dimensional Inc.
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

from threading import RLock
import time
from typing import cast

import numpy as np
from numpy.typing import NDArray

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

_max_points_per_second = 1000


class PositionTracker:
    _lock: RLock
    _time_window: float
    _max_points: int
    _threshold: float
    _timestamps: NDArray[np.float32]
    _positions: NDArray[np.float32]
    _index: int
    _size: int

    def __init__(self, time_window: float) -> None:
        self._lock = RLock()
        self._time_window = time_window
        self._threshold = 0.4
        self._max_points = int(_max_points_per_second * self._time_window)
        self.reset_data()

    def reset_data(self) -> None:
        with self._lock:
            self._timestamps = np.zeros(self._max_points, dtype=np.float32)
            self._positions = np.zeros((self._max_points, 2), dtype=np.float32)
            self._index = 0
            self._size = 0

    def add_position(self, pose: PoseStamped) -> None:
        with self._lock:
            self._timestamps[self._index] = time.time()
            self._positions[self._index] = (pose.position.x, pose.position.y)
            self._index = (self._index + 1) % self._max_points
            self._size = min(self._size + 1, self._max_points)

    def _get_recent_positions(self) -> NDArray[np.float32]:
        cutoff = time.time() - self._time_window

        if self._size == 0:
            return np.empty((0, 2), dtype=np.float32)

        if self._size < self._max_points:
            mask = self._timestamps[: self._size] >= cutoff
            return self._positions[: self._size][mask]

        ts = np.concatenate([self._timestamps[self._index :], self._timestamps[: self._index]])
        pos = np.concatenate([self._positions[self._index :], self._positions[: self._index]])
        mask = ts >= cutoff
        return cast("NDArray[np.float32]", pos[mask])

    def is_stuck(self) -> bool:
        with self._lock:
            recent = self._get_recent_positions()

        if len(recent) == 0:
            return False

        centroid = recent.mean(axis=0)
        distances = np.linalg.norm(recent - centroid, axis=1)

        return bool(np.all(distances < self._threshold))

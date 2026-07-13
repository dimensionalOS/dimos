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

"""Client-side helper to turn live lidar over LCM into a plain numpy array.

`PointCloud2.points_f32()`/`.as_numpy()` already convert a single message,
and `dimos.mapping.pointclouds.accumulators`/`occupancy` already handle
persistent map accumulation and grid rasterization inside the running module
graph (see `dimos.robot.unitree.type.map.Map`). What's missing is a way for
code running *outside* that graph (a script, or a skill) to subscribe to
`/lidar` (and optionally an already-accumulated `/global_map`) and pull a
numpy snapshot on demand, without hand-rolling the subscribe/accumulate/
vstack bookkeeping each time.
"""

from __future__ import annotations

import threading

import numpy as np

from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class LidarPointCloudClient:
    """Subscribes to live lidar over LCM and exposes it as a numpy array."""

    def __init__(
        self,
        lidar_topic: str = "/lidar",
        global_map_topic: str | None = "/global_map",
    ) -> None:
        self.lidar_topic = lidar_topic
        self.global_map_topic = global_map_topic

        self._lock = threading.Lock()
        self._chunks: list[np.ndarray] = []
        self._global_map: np.ndarray | None = None
        self._message_count = 0

        self._lidar_transport: LCMTransport[PointCloud2] | None = None
        self._global_map_transport: LCMTransport[PointCloud2] | None = None

    def start(self) -> None:
        self._lidar_transport = LCMTransport(self.lidar_topic, PointCloud2)
        self._lidar_transport.subscribe(self._on_lidar)
        if self.global_map_topic is not None:
            self._global_map_transport = LCMTransport(self.global_map_topic, PointCloud2)
            self._global_map_transport.subscribe(self._on_global_map)

    def stop(self) -> None:
        if self._lidar_transport is not None:
            self._lidar_transport.stop()
            self._lidar_transport = None
        if self._global_map_transport is not None:
            self._global_map_transport.stop()
            self._global_map_transport = None

    def _on_lidar(self, pc: PointCloud2) -> None:
        pts = pc.points_f32()
        if len(pts) == 0:
            return
        with self._lock:
            self._chunks.append(pts)
            self._message_count += 1

    def _on_global_map(self, pc: PointCloud2) -> None:
        pts = pc.points_f32()
        if len(pts) == 0:
            return
        with self._lock:
            self._global_map = pts  # cumulative — latest wins

    def snapshot(self) -> np.ndarray:
        """Thread-safe (N, 3) float32 vstack of buffered chunks + global map."""
        with self._lock:
            arrays = list(self._chunks)
            if self._global_map is not None:
                arrays.append(self._global_map)
        if not arrays:
            return np.zeros((0, 3), dtype=np.float32)
        return np.vstack(arrays)

    def clear(self) -> None:
        """Drop buffered /lidar chunks (the last global map, if any, is kept)."""
        with self._lock:
            self._chunks = []

    @property
    def message_count(self) -> int:
        return self._message_count

    def __enter__(self) -> LidarPointCloudClient:
        self.start()
        return self

    def __exit__(self, *exc: object) -> None:
        self.stop()

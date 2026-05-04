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

from __future__ import annotations

from collections import defaultdict
import threading
import time
from typing import Any

import numpy as np
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class TerrainMapExtConfig(ModuleConfig):
    world_frame: str = "map"
    voxel_size: float = 0.4  # meters per voxel (coarser than local)
    decay_time: float = 8.0  # seconds before points expire
    publish_rate: float = 2.0  # Hz
    max_range: float = 40.0  # max distance from robot to keep


class TerrainMapExt(Module):
    """Extended terrain map: accumulates local terrain into a wider, slower-decaying map."""

    config: TerrainMapExtConfig

    terrain_map: In[PointCloud2]
    odometry: In[Odometry]
    terrain_map_ext: Out[PointCloud2]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._running = False
        self._thread: threading.Thread | None = None
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._lock = threading.Lock()
        # Voxel storage: key=(ix,iy,iz) -> (x, y, z, intensity, timestamp)
        self._voxels: dict[tuple[int, int, int], tuple[float, float, float, float, float]] = {}
        # Reverse index: (ix,iy) -> set of iz values present in _voxels
        self._column_index: defaultdict[tuple[int, int], set[int]] = defaultdict(set)

    @rpc
    def start(self) -> None:
        super().start()
        # Reset state on start
        self._voxels.clear()
        self._column_index.clear()
        self.register_disposable(Disposable(self.terrain_map.subscribe(self._on_terrain)))
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odom)))
        self._running = True
        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._robot_x = msg.pose.position.x
            self._robot_y = msg.pose.position.y

    def _on_terrain(self, cloud: PointCloud2) -> None:
        points, _ = cloud.as_numpy()
        if len(points) == 0:
            return

        vs = self.config.voxel_size
        now = time.time()

        # Vectorized voxel index computation
        indices = np.floor(points[:, :3] / vs).astype(int)

        with self._lock:
            # Z-column clearing: for each observed (ix, iy) column, remove
            # all existing voxels so that disappeared obstacles don't persist
            # as phantoms until time-decay.  Uses the reverse index for O(1)
            # per-column lookup instead of scanning the full voxel dict.
            observed_columns: set[tuple[int, int]] = set()
            for i in range(len(indices)):
                observed_columns.add((int(indices[i, 0]), int(indices[i, 1])))

            for column in observed_columns:
                iz_set = self._column_index.pop(column, None)
                if iz_set:
                    for iz in iz_set:
                        self._voxels.pop((*column, iz), None)

            # Insert fresh observations and rebuild column index entries
            for i in range(len(indices)):
                ix, iy, iz = int(indices[i, 0]), int(indices[i, 1]), int(indices[i, 2])
                x, y, z = float(points[i, 0]), float(points[i, 1]), float(points[i, 2])
                self._voxels[(ix, iy, iz)] = (x, y, z, 0.0, now)
                col = (ix, iy)
                if col in self._column_index:
                    self._column_index[col].add(iz)
                else:
                    self._column_index[col] = {iz}

    def _publish_loop(self) -> None:
        dt = 1.0 / self.config.publish_rate
        while self._running:
            t0 = time.monotonic()
            now = time.time()
            decay = self.config.decay_time
            max_r2 = self.config.max_range**2

            with self._lock:
                rx, ry = self._robot_x, self._robot_y
                # Expire old voxels and range-cull
                expired = []
                pts = []
                for k, (x, y, z, _intensity, ts) in self._voxels.items():
                    if now - ts > decay:
                        expired.append(k)
                    elif (x - rx) ** 2 + (y - ry) ** 2 > max_r2:
                        expired.append(k)
                    else:
                        pts.append([x, y, z])
                for k in expired:
                    del self._voxels[k]
                    col = (k[0], k[1])
                    iz_set = self._column_index.get(col)
                    if iz_set is not None:
                        iz_set.discard(k[2])
                        if not iz_set:
                            del self._column_index[col]

            if pts:
                arr = np.array(pts, dtype=np.float32)
                self.terrain_map_ext.publish(
                    PointCloud2.from_numpy(arr, frame_id=self.config.world_frame, timestamp=now)
                )

            elapsed = time.monotonic() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)

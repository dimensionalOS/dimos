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

"""GlobalMapUpdater — accumulated voxelized point cloud from registered_scan.

Subscribes to registered_scan and odometry, accumulates points into a
voxel grid with TIME DECAY and RANGE CULLING, and publishes the bounded
accumulated cloud periodically for visualization or incremental mapping.

This is the bounded-memory alternative to PreloadedMapTracker. Points
are expired after `decay_time` seconds and culled if they're more than
`max_range` meters from the robot. A hard cap (`max_points`) prevents
runaway growth.

Suitable for long-running systems where the full explored history is
not needed. For planners that need the full persistent history (e.g.
PCT tomograms), use PreloadedMapTracker instead.
"""

from __future__ import annotations

import threading
import time
from typing import Any

import numpy as np

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class GlobalMapUpdaterConfig(ModuleConfig):
    """Config for global map updater (bounded voxel accumulator)."""

    voxel_size: float = 0.15  # meters per voxel (fine enough for map detail)
    decay_time: float = 300.0  # seconds before points expire (5 min)
    publish_rate: float = 1.0  # Hz — keep low to avoid memory explosion
    max_range: float = 80.0  # max distance from robot to keep
    max_points: int = 500_000  # hard cap on published points
    height_min: float = -2.0  # clip floor noise
    height_max: float = 4.0  # clip ceiling


class GlobalMapUpdater(Module[GlobalMapUpdaterConfig]):
    """Bounded-memory accumulated global point cloud from registered_scan.

    Voxelizes incoming scans and maintains a persistent map with
    time-based decay and range culling. Publishes the bounded accumulated
    cloud for visualization or incremental mapping.

    Ports:
        registered_scan (In[PointCloud2]): World-frame lidar scan.
        odometry (In[Odometry]): Vehicle pose for range culling.
        global_map (Out[PointCloud2]): Accumulated voxelized cloud.
    """

    default_config = GlobalMapUpdaterConfig

    registered_scan: In[PointCloud2]
    odometry: In[Odometry]
    global_map: Out[PointCloud2]

    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None
        # Voxel storage: key=(ix,iy,iz) -> (x, y, z, timestamp)
        self._voxels: dict[tuple[int, int, int], tuple[float, float, float, float]] = {}
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_z = 0.0

    def __getstate__(self) -> dict[str, Any]:
        state = super().__getstate__()
        for k in ("_lock", "_thread", "_voxels"):
            state.pop(k, None)
        return state

    def __setstate__(self, state: dict[str, Any]) -> None:
        super().__setstate__(state)
        self._lock = threading.Lock()
        self._thread = None
        self._voxels = {}

    @rpc
    def start(self) -> None:
        self.registered_scan.subscribe(self._on_scan)
        self.odometry.subscribe(self._on_odom)
        self._running = True
        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        super().stop()

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._robot_x = msg.pose.position.x
            self._robot_y = msg.pose.position.y
            self._robot_z = msg.pose.position.z

    def _on_scan(self, cloud: PointCloud2) -> None:
        points, _ = cloud.as_numpy()
        if len(points) == 0:
            return

        vs = self.config.voxel_size
        h_min = self.config.height_min
        h_max = self.config.height_max
        now = time.time()

        with self._lock:
            for i in range(len(points)):
                x, y, z = float(points[i, 0]), float(points[i, 1]), float(points[i, 2])
                # Height filter
                if z < h_min or z > h_max:
                    continue
                ix = int(np.floor(x / vs))
                iy = int(np.floor(y / vs))
                iz = int(np.floor(z / vs))
                self._voxels[(ix, iy, iz)] = (x, y, z, now)

    def _publish_loop(self) -> None:
        dt = 1.0 / self.config.publish_rate
        while self._running:
            t0 = time.monotonic()
            now = time.time()
            decay = self.config.decay_time
            max_r2 = self.config.max_range**2
            max_pts = self.config.max_points

            with self._lock:
                rx, ry = self._robot_x, self._robot_y
                # Expire old voxels and range-cull
                expired = []
                pts = []
                for k, (x, y, z, ts) in self._voxels.items():
                    if now - ts > decay:
                        expired.append(k)
                    elif (x - rx) ** 2 + (y - ry) ** 2 > max_r2:
                        expired.append(k)
                    else:
                        pts.append([x, y, z])
                for k in expired:
                    del self._voxels[k]

            if pts:
                # Cap total points to prevent memory explosion
                if len(pts) > max_pts:
                    pts = pts[:max_pts]
                arr = np.array(pts, dtype=np.float32)
                self.global_map.publish(
                    PointCloud2.from_numpy(arr, frame_id="map", timestamp=now)
                )

            elapsed = time.monotonic() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)

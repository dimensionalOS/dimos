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

"""TerrainMapExt: extended persistent terrain map with time decay.

Accumulates terrain_map messages from TerrainAnalysis into a larger
rolling voxel grid (~40m radius, 2m voxels, 4s decay). Publishes
the accumulated map as terrain_map_ext for visualization and planning.

Port of terrain_analysis_ext from the original ROS2 codebase, simplified
to Python using numpy voxel hashing.
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


class TerrainMapExtConfig(ModuleConfig):
    """Config for extended terrain map."""

    voxel_size: float = 0.4  # meters per voxel (coarser than local)
    obstacle_height_threshold: float = 0.2  # terrain intensity at/above this is blocked
    decay_time: float = 8.0  # seconds before points expire
    publish_rate: float = 2.0  # Hz
    max_range: float = 40.0  # max distance from robot to keep
    robot_exclusion_radius: float = 0.0  # ignore self/near-footprint points in the global map


class TerrainMapExt(Module[TerrainMapExtConfig]):
    """Extended terrain map with time-decayed voxel accumulation.

    Subscribes to terrain_map (local) and accumulates into a persistent
    map that covers a larger area with slower decay.

    Ports:
        terrain_map (In[PointCloud2]): Local terrain from TerrainAnalysis.
        odometry (In[Odometry]): Vehicle pose for range culling.
        terrain_map_ext (Out[PointCloud2]): Extended accumulated terrain.
    """

    default_config = TerrainMapExtConfig

    terrain_map: In[PointCloud2]
    odometry: In[Odometry]
    terrain_map_ext: Out[PointCloud2]

    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None
        # Voxel storage: key=(ix,iy,iz) -> (x, y, z, intensity, timestamp)
        self._voxels: dict[tuple[int, int, int], tuple[float, float, float, float, float]] = {}
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._has_odom = False

    def __getstate__(self) -> dict[str, Any]:
        s = super().__getstate__()
        for k in ("_lock", "_thread", "_voxels"):
            s.pop(k, None)
        return s

    def __setstate__(self, s: dict) -> None:
        super().__setstate__(s)
        self._lock = threading.Lock()
        self._thread = None
        self._voxels = {}
        if not hasattr(self, "_has_odom"):
            self._has_odom = False

    @rpc
    def start(self) -> None:
        self.terrain_map.subscribe(self._on_terrain)
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
            self._has_odom = True

    def _on_terrain(self, cloud: PointCloud2) -> None:
        points = cloud.points_f32()
        if len(points) == 0:
            return
        intensities = cloud.intensity_f32()

        vs = self.config.voxel_size
        now = time.time()
        threshold = self.config.obstacle_height_threshold
        updates: list[tuple[tuple[int, int, int], tuple[float, float, float, float, float]]] = []
        clear_columns: set[tuple[int, int]] = set()
        obstacle_columns: set[tuple[int, int]] = set()

        with self._lock:
            rx = self._robot_x
            ry = self._robot_y
            has_odom = getattr(self, "_has_odom", False)
            exclusion_radius = self.config.robot_exclusion_radius
            exclusion_radius_sq = exclusion_radius * exclusion_radius
            for i in range(len(points)):
                x, y, z = float(points[i, 0]), float(points[i, 1]), float(points[i, 2])
                if (
                    has_odom
                    and exclusion_radius > 0.0
                    and (x - rx) ** 2 + (y - ry) ** 2 <= exclusion_radius_sq
                ):
                    continue
                intensity = (
                    float(intensities[i])
                    if intensities is not None and len(intensities) == len(points)
                    else 0.0
                )
                ix = int(np.floor(x / vs))
                iy = int(np.floor(y / vs))
                iz = int(np.floor(z / vs))
                column = (ix, iy)
                if intensity >= threshold:
                    obstacle_columns.add(column)
                else:
                    clear_columns.add(column)
                updates.append(((ix, iy, iz), (x, y, z, intensity, now)))

            for column in clear_columns - obstacle_columns:
                stale = [
                    key
                    for key, value in self._voxels.items()
                    if key[:2] == column and value[3] >= threshold
                ]
                for key in stale:
                    del self._voxels[key]

            for key, value in updates:
                self._voxels[key] = value

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
                intensities = []
                for k, (x, y, z, intensity, ts) in self._voxels.items():
                    if now - ts > decay:
                        expired.append(k)
                    elif (x - rx) ** 2 + (y - ry) ** 2 > max_r2:
                        expired.append(k)
                    else:
                        pts.append([x, y, z])
                        intensities.append(intensity)
                for k in expired:
                    del self._voxels[k]

            if pts:
                arr = np.array(pts, dtype=np.float32)
                intensity_arr = np.array(intensities, dtype=np.float32)
                self.terrain_map_ext.publish(
                    PointCloud2.from_numpy(
                        arr, frame_id="map", timestamp=now, intensity=intensity_arr
                    )
                )

            elapsed = time.monotonic() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)

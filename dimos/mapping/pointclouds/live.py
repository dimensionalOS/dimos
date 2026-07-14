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
from dimos.mapping.pointclouds import signals
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

    def add_lidar(self, pc: PointCloud2) -> None:
        """Ingest one lidar message.

        Public entry point so code that already has the message in hand — e.g.
        a Module feeding its in-graph ``In[PointCloud2]`` stream — can reuse
        this accumulator without opening a second LCM subscription via
        ``start()``.
        """
        self._on_lidar(pc)

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

    # ---- derived signals over the current snapshot (see signals.py) -------
    # Thin wrappers so an outside-graph caller can ask the live sensor a
    # question directly, without threading snapshot() through every call.

    def extents(self, percentiles: tuple[float, float] | None = None) -> signals.Extents:
        """Bounding box + room dimensions of everything scanned so far (objective 1).

        Pass ``percentiles`` (e.g. ``(1.0, 99.0)``) for outlier-robust bounds;
        see :func:`signals.extents`.
        """
        return signals.extents(self.snapshot(), percentiles=percentiles)

    def nearest_obstacle(
        self,
        origin: tuple[float, float, float] = (0.0, 0.0, 0.0),
        height_band: tuple[float, float] | None = None,
    ) -> float:
        """Horizontal distance to the closest point; inf if none (objective 2)."""
        return signals.nearest_obstacle(self.snapshot(), origin=origin, height_band=height_band)

    def clearance(
        self,
        heading: float,
        fov_deg: float = 60.0,
        origin: tuple[float, float, float] = (0.0, 0.0, 0.0),
        height_band: tuple[float, float] | None = None,
    ) -> float:
        """Distance to the closest point in a cone around ``heading`` (objective 2)."""
        return signals.clearance(
            self.snapshot(), heading, fov_deg=fov_deg, origin=origin, height_band=height_band
        )

    def occupancy_grid(
        self,
        resolution: float = 0.1,
        height_band: tuple[float, float] | None = None,
        bounds: tuple[float, float, float, float] | None = None,
    ) -> signals.OccupancyGrid2D:
        """Top-down occupancy raster of the scan as a portable artifact (objective 3)."""
        return signals.occupancy_grid(
            self.snapshot(), resolution=resolution, height_band=height_band, bounds=bounds
        )

    def coverage_summary(self, resolution: float = 0.1) -> dict:  # type: ignore[type-arg]
        """Coverage feedback (objective 6): points, messages, and scanned area.

        ``area_m2`` is None until anything has been scanned so callers can tell
        "nothing yet" from "0 area".
        """
        snap = self.snapshot()
        return {
            "points": len(snap),
            "messages": self.message_count,
            "area_m2": (signals.coverage_area(snap, resolution=resolution) if len(snap) else None),
        }

    def __enter__(self) -> LidarPointCloudClient:
        self.start()
        return self

    def __exit__(self, *exc: object) -> None:
        self.stop()

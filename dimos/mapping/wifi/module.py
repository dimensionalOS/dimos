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

from dataclasses import asdict
import threading
import time
from typing import Any

import numpy as np
import open3d as o3d
from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.wifi.coverage import CoverageMap
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.sensor_msgs.WifiScan import WifiScan


class WifiCoverageConfig(ModuleConfig):
    cell_size: float = Field(default=0.5, gt=0)
    max_cells: int = Field(default=10000, ge=1)
    max_pose_age_s: float = Field(default=0.5, gt=0)
    max_sample_age_s: float = Field(default=5.0, gt=0)
    # Empty selects the strongest observed AP in each cell.
    bssid: str = ""


class WifiCoverage(Module):
    """Colour receiver locations by measured RSSI; never infer AP locations or latency."""

    config: WifiCoverageConfig
    odometry: In[Odometry]
    wifi_scan: In[WifiScan]
    wifi_coverage: Out[PointCloud2]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.RLock()
        self._map = CoverageMap(
            self.config.cell_size,
            self.config.max_cells,
            self.config.max_pose_age_s,
            self.config.max_sample_age_s,
        )
        self._bssid = self.config.bssid.lower()

    async def handle_odometry(self, msg: Odometry) -> None:
        p = msg.pose.position
        with self._lock:
            previous_frame = self._map.frame_id
            self._map.add_pose(msg.ts, msg.frame_id, (p.x, p.y, p.z))
            if previous_frame and previous_frame != self._map.frame_id:
                self._publish(msg.ts)

    async def handle_wifi_scan(self, msg: WifiScan) -> None:
        with self._lock:
            if not self._map.add_scan(msg):
                return
            self._publish(msg.ts)

    def _publish(self, ts: float) -> None:
        cells = self._map.selected_cells(self._bssid)
        points = np.asarray([cell.position for cell in cells], dtype=np.float64).reshape(-1, 3)
        signal = np.asarray([cell.mean_dbm for cell in cells])
        strength = np.clip((signal + 90) / 50, 0, 1)
        colors = np.column_stack(
            (np.minimum(2 * (1 - strength), 1), np.minimum(2 * strength, 1), np.zeros(len(cells)))
        )
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        cloud.colors = o3d.utility.Vector3dVector(colors)
        self.wifi_coverage.publish(PointCloud2(cloud, frame_id=self._map.frame_id, ts=ts))

    @rpc
    def select_access_point(self, bssid: str = "") -> None:
        """Show one BSSID, or the strongest observed AP per cell when empty."""
        with self._lock:
            self._bssid = bssid.lower()
            self._publish(time.time())

    @rpc
    def get_cells(self) -> list[dict[str, Any]]:
        """Return selected cells, including counts, RSSI range and observation timestamps."""
        with self._lock:
            return [asdict(cell) for cell in self._map.selected_cells(self._bssid)]

    @rpc
    def get_access_points(self) -> list[dict[str, Any]]:
        """List observed BSSIDs without merging radios that share an SSID."""
        with self._lock:
            latest = {cell.bssid: cell for cell in self._map.cells.values()}
            return [
                dict(
                    bssid=cell.bssid,
                    ssid=cell.ssid,
                    frequency_mhz=cell.frequency_mhz,
                    last_seen=cell.last_seen,
                )
                for cell in latest.values()
            ]

    @rpc
    def reset(self) -> None:
        """Clear coverage and pose history before a new localization/map epoch."""
        with self._lock:
            frame_id = self._map.frame_id
            self._map.clear()
            self.wifi_coverage.publish(PointCloud2(frame_id=frame_id, ts=time.time()))

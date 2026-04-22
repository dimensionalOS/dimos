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

"""Corrects raw lidar scans using PGO-corrected odom and overlays onto PGO map.

Takes the robot's raw world-frame lidar, un-registers it using the raw odom,
re-registers it with PGO's corrected odom, then combines it with PGO's global
map using z-column clearing so that stale obstacles are removed.

Rate-limited to PGO corrected odom — only produces output when a new corrected
pose arrives, regardless of how fast the lidar publishes.
"""

import threading
import time

import numpy as np
from reactivex.disposable import Disposable
from scipy.spatial.transform import Rotation

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class ScanCorrectorConfig(ModuleConfig):
    column_resolution: float = 0.05
    """XY grid resolution (metres) for z-column clearing."""


class ScanCorrector(Module):
    """Overlay PGO-corrected lidar scans onto PGO's global map.

    Inputs:
        registered_scan: World-frame lidar from the robot (registered with raw odom).
        raw_odom: The robot's raw pose used to register the scan.
        corrected_odometry: PGO's loop-closure-corrected odometry (rate-limiter).
        global_static_map: PGO's accumulated global static map (slow-updating).

    Output:
        corrected_map: Combined pointcloud — PGO map with z-columns cleared and
                       replaced by the latest corrected lidar frame.
    """

    config: ScanCorrectorConfig
    default_config = ScanCorrectorConfig

    registered_scan: In[PointCloud2]
    raw_odom: In[PoseStamped]
    corrected_odometry: In[Odometry]
    global_static_map: In[PointCloud2]

    corrected_map: Out[PointCloud2]
    corrected_lidar: Out[PointCloud2]

    def __init__(self, **kwargs):  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        # Scan paired with the raw odom at the time it arrived
        self._latest_scan: PointCloud2 | None = None
        self._latest_scan_raw_r = np.eye(3)
        self._latest_scan_raw_t = np.zeros(3)
        # Most recent raw odom (updated continuously)
        self._latest_raw_r = np.eye(3)
        self._latest_raw_t = np.zeros(3)
        self._latest_pgo_map_pts: np.ndarray | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._disposables.add(Disposable(self.registered_scan.subscribe(self._on_scan)))
        self._disposables.add(Disposable(self.raw_odom.subscribe(self._on_raw_odom)))
        self._disposables.add(
            Disposable(self.corrected_odometry.subscribe(self._on_corrected_odom))
        )
        self._disposables.add(Disposable(self.global_static_map.subscribe(self._on_pgo_map)))
        logger.info("ScanCorrector started")

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_scan(self, cloud: PointCloud2) -> None:
        with self._lock:
            self._latest_scan = cloud
            self._latest_scan_raw_r = self._latest_raw_r.copy()
            self._latest_scan_raw_t = self._latest_raw_t.copy()

    def _on_raw_odom(self, msg: PoseStamped) -> None:
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        r = Rotation.from_quat(q).as_matrix()
        t = np.array([msg.x, msg.y, msg.z])
        with self._lock:
            self._latest_raw_r = r
            self._latest_raw_t = t

    def _on_pgo_map(self, cloud: PointCloud2) -> None:
        pts, _ = cloud.as_numpy()
        if len(pts) == 0:
            return
        with self._lock:
            self._latest_pgo_map_pts = pts[:, :3].copy()

    def _on_corrected_odom(self, msg: Odometry) -> None:
        """Rate-limiter: triggered by PGO corrected odom."""
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        corrected_r = Rotation.from_quat(q).as_matrix()
        corrected_t = np.array([msg.x, msg.y, msg.z])

        with self._lock:
            scan = self._latest_scan
            raw_r = self._latest_scan_raw_r.copy()
            raw_t = self._latest_scan_raw_t.copy()
            pgo_map_pts = self._latest_pgo_map_pts

        if scan is None:
            return

        scan_pts, _ = scan.as_numpy()
        if len(scan_pts) == 0:
            return
        world_pts = scan_pts[:, :3]

        # Un-register: world-frame (raw odom) -> body-frame
        body_pts = (raw_r.T @ (world_pts.T - raw_t[:, None])).T

        # Re-register: body-frame -> world-frame (corrected odom)
        corrected_pts = (corrected_r @ body_pts.T).T + corrected_t

        now = time.time()

        # Publish just the re-registered lidar for debugging
        self.corrected_lidar.publish(
            PointCloud2.from_numpy(corrected_pts.astype(np.float32), frame_id="map", timestamp=now)
        )

        if pgo_map_pts is None or len(pgo_map_pts) == 0:
            combined = corrected_pts
        else:
            combined = self._column_carve_and_overlay(pgo_map_pts, corrected_pts)

        self.corrected_map.publish(
            PointCloud2.from_numpy(combined.astype(np.float32), frame_id="map", timestamp=now)
        )

    def _column_carve_and_overlay(
        self, base_pts: np.ndarray, overlay_pts: np.ndarray
    ) -> np.ndarray:
        """Clear z-columns in base_pts that are occupied by overlay_pts, then combine."""
        res = self.config.column_resolution

        # Discretize overlay columns
        overlay_cols = np.floor(overlay_pts[:, :2] / res).astype(np.int64)
        overlay_keys = overlay_cols[:, 0] + overlay_cols[:, 1] * 1_000_000

        # Discretize base columns
        base_cols = np.floor(base_pts[:, :2] / res).astype(np.int64)
        base_keys = base_cols[:, 0] + base_cols[:, 1] * 1_000_000

        # Keep base points whose column is NOT in overlay (vectorized)
        mask = ~np.isin(base_keys, overlay_keys)

        surviving = base_pts[mask]
        return np.vstack([surviving, overlay_pts]) if len(surviving) > 0 else overlay_pts

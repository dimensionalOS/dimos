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

"""PreloadedMapTracker — adaptation of ROS visualizationTools.

Original: ros-navigation-autonomy-stack/src/base_autonomy/visualization_tools/src/visualizationTools.cpp

Maintains two point clouds:
  1. preloaded_map: a static reference map loaded from disk (or empty). Never
     grows at runtime. Equivalent to /overall_map in ROS.
  2. explored_areas: accumulated registered scans at 0.3m voxel resolution.

WARNING: This module accumulates explored_areas without decay or range culling.
Growth is bounded only by voxel resolution (~111K voxels per 100m² floor at 0.3m).
Memory usage grows with explored area size. For long runs or large environments,
consider GlobalMapUpdater instead.

Also tracks trajectory with cumulative traveling distance as per-point intensity.
"""

from __future__ import annotations

import math
import os
import threading
import time
from typing import Any

import numpy as np
import open3d as o3d

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class PreloadedMapTrackerConfig(ModuleConfig):
    """Config for PreloadedMapTracker."""

    # Pre-loaded reference map
    preloaded_map_path: str = ""  # path to .pcd/.ply file (empty = no map)
    preloaded_map_voxel_size: float = 0.5  # downsample resolution for reference map
    preloaded_map_publish_rate: float = 1.0  # Hz (ROS default: ~1Hz)

    # Explored areas accumulator
    explored_areas_voxel_size: float = 0.3  # matches ROS exploredAreaVoxelSize
    explored_areas_publish_rate: float = 10.0  # Hz (ROS publishes every 10 scan updates)

    # Trajectory tracking (thresholds before appending a new trajectory point)
    trajectory_trans_interval: float = 0.2  # meters
    trajectory_yaw_interval_deg: float = 10.0  # degrees

    # Height clipping (to match terrain-band filtering)
    height_min: float = -2.0
    height_max: float = 4.0


class PreloadedMapTracker(Module[PreloadedMapTrackerConfig]):
    """Static pre-loaded reference map + unbounded explored_areas accumulator.

    Ports:
        registered_scan (In[PointCloud2]): World-frame lidar scan.
        odometry (In[Odometry]): Vehicle pose for trajectory tracking.
        preloaded_map (Out[PointCloud2]): Static pre-loaded reference (empty if no file).
        explored_areas (Out[PointCloud2]): Accumulated scans at 0.3m voxel (PCT input).
        trajectory (Out[PointCloud2]): Robot path breadcrumb with cumulative distance.
    """

    default_config = PreloadedMapTrackerConfig

    registered_scan: In[PointCloud2]
    odometry: In[Odometry]
    preloaded_map: Out[PointCloud2]
    explored_areas: Out[PointCloud2]
    trajectory: Out[PointCloud2]

    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._running = False
        self._scan_thread: threading.Thread | None = None
        self._preloaded_thread: threading.Thread | None = None

        # Explored areas: voxel key -> (x, y, z)
        self._explored_voxels: dict[tuple[int, int, int], tuple[float, float, float]] = {}

        # Trajectory
        self._trajectory_points: list[tuple[float, float, float, float]] = []  # x,y,z,distance
        self._traveling_distance = 0.0
        self._last_traj_x = 0.0
        self._last_traj_y = 0.0
        self._last_traj_z = 0.0
        self._last_traj_yaw = 0.0
        self._trajectory_initialized = False

        # Pre-loaded reference map (loaded lazily in start())
        self._preloaded_points: np.ndarray | None = None

    def __getstate__(self) -> dict[str, Any]:
        state = super().__getstate__()
        for k in (
            "_lock",
            "_scan_thread",
            "_preloaded_thread",
            "_explored_voxels",
            "_trajectory_points",
            "_preloaded_points",
        ):
            state.pop(k, None)
        return state

    def __setstate__(self, state: dict[str, Any]) -> None:
        super().__setstate__(state)
        self._lock = threading.Lock()
        self._scan_thread = None
        self._preloaded_thread = None
        self._explored_voxels = {}
        self._trajectory_points = []
        self._preloaded_points = None

    @rpc
    def start(self) -> None:
        self._load_preloaded_map()
        self.registered_scan.subscribe(self._on_scan)
        self.odometry.subscribe(self._on_odom)
        self._running = True
        self._scan_thread = threading.Thread(target=self._explored_publish_loop, daemon=True)
        self._scan_thread.start()
        self._preloaded_thread = threading.Thread(target=self._preloaded_publish_loop, daemon=True)
        self._preloaded_thread.start()

    @rpc
    def stop(self) -> None:
        self._running = False
        if self._scan_thread:
            self._scan_thread.join(timeout=3.0)
        if self._preloaded_thread:
            self._preloaded_thread.join(timeout=3.0)
        super().stop()

    # ─── Pre-loaded map ──────────────────────────────────────────────────────

    def _load_preloaded_map(self) -> None:
        path = self.config.preloaded_map_path
        if not path or not os.path.exists(path):
            return
        try:
            pcd = o3d.io.read_point_cloud(path)
            if len(pcd.points) == 0:
                return
            vs = self.config.preloaded_map_voxel_size
            pcd_ds = pcd.voxel_down_sample(voxel_size=vs)
            self._preloaded_points = np.asarray(pcd_ds.points, dtype=np.float32)
        except Exception:
            self._preloaded_points = None

    def _preloaded_publish_loop(self) -> None:
        dt = 1.0 / self.config.preloaded_map_publish_rate
        while self._running:
            t0 = time.monotonic()
            now = time.time()

            # Publish preloaded_map (cached, never grows)
            if self._preloaded_points is not None and len(self._preloaded_points) > 0:
                self.preloaded_map.publish(
                    PointCloud2.from_numpy(self._preloaded_points, frame_id="map", timestamp=now)
                )

            # Publish trajectory snapshot
            with self._lock:
                traj = list(self._trajectory_points)
            if traj:
                arr = np.array([[p[0], p[1], p[2]] for p in traj], dtype=np.float32)
                self.trajectory.publish(
                    PointCloud2.from_numpy(arr, frame_id="map", timestamp=now)
                )

            elapsed = time.monotonic() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)

    # ─── Odometry / trajectory ───────────────────────────────────────────────

    def _on_odom(self, msg: Odometry) -> None:
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        z = float(msg.pose.position.z)
        # Yaw from quaternion: yaw = atan2(2(wz+xy), 1-2(y²+z²))
        qx = float(msg.pose.orientation.x)
        qy = float(msg.pose.orientation.y)
        qz = float(msg.pose.orientation.z)
        qw = float(msg.pose.orientation.w)
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

        with self._lock:
            if not self._trajectory_initialized:
                self._last_traj_x = x
                self._last_traj_y = y
                self._last_traj_z = z
                self._last_traj_yaw = yaw
                self._trajectory_initialized = True
                # Seed with first point
                self._trajectory_points.append((x, y, z, 0.0))
                return

            dx = x - self._last_traj_x
            dy = y - self._last_traj_y
            dz = z - self._last_traj_z
            dis = math.sqrt(dx * dx + dy * dy + dz * dz)
            d_yaw = abs(yaw - self._last_traj_yaw)
            if d_yaw > math.pi:
                d_yaw = 2.0 * math.pi - d_yaw
            d_yaw_deg = math.degrees(d_yaw)

            if (
                dis < self.config.trajectory_trans_interval
                and d_yaw_deg < self.config.trajectory_yaw_interval_deg
            ):
                return

            self._traveling_distance += dis
            self._last_traj_x = x
            self._last_traj_y = y
            self._last_traj_z = z
            self._last_traj_yaw = yaw
            self._trajectory_points.append((x, y, z, self._traveling_distance))

    # ─── Registered scan / explored_areas ────────────────────────────────────

    def _on_scan(self, cloud: PointCloud2) -> None:
        points, _ = cloud.as_numpy()
        if len(points) == 0:
            return

        vs = self.config.explored_areas_voxel_size
        h_min = self.config.height_min
        h_max = self.config.height_max

        # Vectorized height filter + voxelization
        pts = np.asarray(points, dtype=np.float32)
        z = pts[:, 2]
        mask = (z >= h_min) & (z <= h_max)
        if not np.any(mask):
            return
        pts = pts[mask]
        keys = np.floor(pts / vs).astype(np.int64)

        with self._lock:
            for i in range(pts.shape[0]):
                k = (int(keys[i, 0]), int(keys[i, 1]), int(keys[i, 2]))
                if k not in self._explored_voxels:
                    self._explored_voxels[k] = (
                        float(pts[i, 0]),
                        float(pts[i, 1]),
                        float(pts[i, 2]),
                    )

    def _explored_publish_loop(self) -> None:
        dt = 1.0 / self.config.explored_areas_publish_rate
        while self._running:
            t0 = time.monotonic()
            now = time.time()

            with self._lock:
                pts = list(self._explored_voxels.values())

            if pts:
                arr = np.array(pts, dtype=np.float32)
                self.explored_areas.publish(
                    PointCloud2.from_numpy(arr, frame_id="map", timestamp=now)
                )

            elapsed = time.monotonic() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)

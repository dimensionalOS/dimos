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

"""Tests for PreloadedMapTracker module."""

from __future__ import annotations

import math
import time

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.smart_nav.modules.preloaded_map_tracker.preloaded_map_tracker import (
    PreloadedMapTracker,
    PreloadedMapTrackerConfig,
)


class _MockTransport:
    def __init__(self):
        self._messages = []
        self._subscribers = []

    def publish(self, msg):
        self._messages.append(msg)
        for cb in self._subscribers:
            cb(msg)

    def broadcast(self, _stream, msg):
        self.publish(msg)

    def subscribe(self, cb):
        self._subscribers.append(cb)
        return lambda: self._subscribers.remove(cb)


def _make_cloud(points: np.ndarray) -> PointCloud2:
    return PointCloud2.from_numpy(points.astype(np.float32), frame_id="map", timestamp=time.time())


def _make_odom(x: float, y: float, z: float, yaw: float = 0.0) -> Odometry:
    q = Quaternion.from_euler(Vector3(0.0, 0.0, yaw))
    return Odometry(
        ts=time.time(),
        frame_id="map",
        child_frame_id="base",
        pose=Pose(position=[x, y, z], orientation=[q.x, q.y, q.z, q.w]),
    )


def _wire(module: PreloadedMapTracker):
    module.preloaded_map._transport = _MockTransport()
    module.explored_areas._transport = _MockTransport()
    module.trajectory._transport = _MockTransport()
    return module


class TestPreloadedMapTrackerConfig:
    def test_default_config(self):
        cfg = PreloadedMapTrackerConfig()
        assert cfg.preloaded_map_path == ""
        assert cfg.explored_areas_voxel_size == 0.3
        assert cfg.preloaded_map_voxel_size == 0.5
        assert cfg.trajectory_trans_interval == 0.2
        assert cfg.trajectory_yaw_interval_deg == 10.0

    def test_voxel_size_matches_ros(self):
        # ROS visualizationTools.cpp defaults
        cfg = PreloadedMapTrackerConfig()
        assert cfg.explored_areas_voxel_size == 0.3
        assert cfg.preloaded_map_voxel_size == 0.5


class TestPreloadedMapTrackerModule:
    @pytest.fixture(autouse=True)
    def _mod(self):
        self.m = _wire(PreloadedMapTracker())
        yield
        self.m._running = False
        self.m.stop()

    def test_ports_declared(self):
        assert hasattr(self.m, "registered_scan")
        assert hasattr(self.m, "odometry")
        assert hasattr(self.m, "preloaded_map")
        assert hasattr(self.m, "explored_areas")
        assert hasattr(self.m, "trajectory")

    def test_empty_preloaded_map(self):
        # No path given → no preloaded points loaded
        self.m._load_preloaded_map()
        assert self.m._preloaded_points is None

    def test_explored_areas_grows(self):
        # Feed two non-overlapping scans and verify voxel count grows
        pts1 = np.array([[0.0, 0.0, 0.5], [1.0, 0.0, 0.5]])
        pts2 = np.array([[5.0, 5.0, 0.5], [6.0, 5.0, 0.5]])
        self.m._on_scan(_make_cloud(pts1))
        assert len(self.m._explored_voxels) == 2
        self.m._on_scan(_make_cloud(pts2))
        assert len(self.m._explored_voxels) == 4

    def test_explored_areas_voxel_dedup(self):
        # Nearby points in same voxel collapse
        pts = np.array([[0.01, 0.01, 0.5], [0.05, 0.05, 0.5], [0.1, 0.1, 0.5]])
        self.m._on_scan(_make_cloud(pts))
        assert len(self.m._explored_voxels) == 1

    def test_height_filter(self):
        # Below height_min and above height_max get filtered
        pts = np.array([[0.0, 0.0, -5.0], [0.0, 0.0, 0.5], [0.0, 0.0, 10.0]])
        self.m._on_scan(_make_cloud(pts))
        assert len(self.m._explored_voxels) == 1

    def test_no_decay(self):
        # Old voxels stay forever — feed once, wait, feed again elsewhere
        pts1 = np.array([[0.0, 0.0, 0.5]])
        self.m._on_scan(_make_cloud(pts1))
        initial_count = len(self.m._explored_voxels)
        time.sleep(0.05)
        pts2 = np.array([[10.0, 10.0, 0.5]])
        self.m._on_scan(_make_cloud(pts2))
        # Original voxel still there (no decay, no culling)
        assert len(self.m._explored_voxels) == initial_count + 1

    def test_trajectory_initial_seed(self):
        self.m._on_odom(_make_odom(1.0, 2.0, 0.0))
        assert len(self.m._trajectory_points) == 1
        assert self.m._trajectory_points[0][:3] == (1.0, 2.0, 0.0)
        assert self.m._traveling_distance == 0.0

    def test_trajectory_threshold_translation(self):
        # Moving < trans_interval produces no new point
        self.m._on_odom(_make_odom(0.0, 0.0, 0.0))
        self.m._on_odom(_make_odom(0.05, 0.0, 0.0))  # below 0.2m threshold
        assert len(self.m._trajectory_points) == 1
        # Moving > trans_interval appends
        self.m._on_odom(_make_odom(0.5, 0.0, 0.0))
        assert len(self.m._trajectory_points) == 2

    def test_trajectory_threshold_yaw(self):
        self.m._on_odom(_make_odom(0.0, 0.0, 0.0, yaw=0.0))
        # Yaw change > 10 deg (≈ 0.175 rad) triggers append
        self.m._on_odom(_make_odom(0.0, 0.0, 0.0, yaw=math.radians(15.0)))
        assert len(self.m._trajectory_points) == 2

    def test_trajectory_cumulative_distance(self):
        self.m._on_odom(_make_odom(0.0, 0.0, 0.0))
        self.m._on_odom(_make_odom(1.0, 0.0, 0.0))  # +1m
        self.m._on_odom(_make_odom(1.0, 1.0, 0.0))  # +1m
        assert self.m._trajectory_points[-1][3] == pytest.approx(2.0, abs=1e-4)

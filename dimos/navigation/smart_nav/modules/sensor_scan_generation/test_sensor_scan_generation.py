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

"""Tests for SensorScanGeneration module."""

import math
import time

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.smart_nav.modules.sensor_scan_generation.sensor_scan_generation import (
    SensorScanGeneration,
)


class _MockTransport:
    """Lightweight mock transport that captures published messages."""

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

        def unsub():
            self._subscribers.remove(cb)

        return unsub


def make_pointcloud(points: np.ndarray, frame_id: str = "map") -> PointCloud2:
    """Create a PointCloud2 from an Nx3 numpy array."""
    return PointCloud2.from_numpy(
        points.astype(np.float32), frame_id=frame_id, timestamp=time.time()
    )


def make_odometry(x: float, y: float, z: float, yaw: float = 0.0) -> Odometry:
    """Create an Odometry message at the given position and yaw."""
    quat = Quaternion.from_euler(Vector3(0.0, 0.0, yaw))
    return Odometry(
        ts=time.time(),
        frame_id="map",
        child_frame_id="sensor",
        pose=Pose(
            position=[x, y, z],
            orientation=[quat.x, quat.y, quat.z, quat.w],
        ),
    )


def _wire_transports(module):
    """Wire mock transports onto all ports of a SensorScanGeneration module."""
    scan_out_transport = _MockTransport()
    odom_out_transport = _MockTransport()
    module.sensor_scan._transport = scan_out_transport
    module.odometry_at_scan._transport = odom_out_transport
    return scan_out_transport, odom_out_transport


class TestSensorScanGeneration:
    """Test SensorScanGeneration module transforms."""

    @pytest.fixture(autouse=True)
    def _create_module(self):
        self.module = SensorScanGeneration()
        self.scan_t, self.odom_t = _wire_transports(self.module)
        yield
        self.module.stop()

    def test_identity_transform(self):
        """When vehicle is at origin with zero rotation, sensor frame = world frame."""
        odom = make_odometry(0.0, 0.0, 0.0, 0.0)
        self.module._on_odometry(odom)

        world_points = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        cloud = make_pointcloud(world_points)

        results = []
        self.scan_t.subscribe(lambda msg: results.append(msg))
        self.module._on_scan(cloud)

        assert len(results) == 1
        sensor_points, _ = results[0].as_numpy()
        np.testing.assert_allclose(sensor_points, world_points, atol=1e-4)

    def test_translation_transform(self):
        """Points should be shifted by the inverse of the vehicle translation."""
        odom = make_odometry(2.0, 3.0, 0.0, 0.0)
        self.module._on_odometry(odom)

        world_points = np.array([[5.0, 3.0, 0.0]])
        cloud = make_pointcloud(world_points)

        results = []
        self.scan_t.subscribe(lambda msg: results.append(msg))
        self.module._on_scan(cloud)

        assert len(results) == 1
        sensor_points, _ = results[0].as_numpy()
        np.testing.assert_allclose(sensor_points[0], [3.0, 0.0, 0.0], atol=1e-4)

    def test_rotation_transform(self):
        """Points should be rotated by the inverse of the vehicle rotation."""
        odom = make_odometry(0.0, 0.0, 0.0, math.pi / 2)
        self.module._on_odometry(odom)

        world_points = np.array([[1.0, 0.0, 0.0]])
        cloud = make_pointcloud(world_points)

        results = []
        self.scan_t.subscribe(lambda msg: results.append(msg))
        self.module._on_scan(cloud)

        assert len(results) == 1
        sensor_points, _ = results[0].as_numpy()
        np.testing.assert_allclose(sensor_points[0], [0.0, -1.0, 0.0], atol=1e-4)

    def test_no_odometry_no_output(self):
        """If no odometry has been received, no scan should be published."""
        world_points = np.array([[1.0, 0.0, 0.0]])
        cloud = make_pointcloud(world_points)

        results = []
        self.scan_t.subscribe(lambda msg: results.append(msg))
        self.module._on_scan(cloud)

        assert len(results) == 0

    def test_empty_cloud(self):
        """Empty point cloud should produce empty output."""
        odom = make_odometry(0.0, 0.0, 0.0)
        self.module._on_odometry(odom)

        cloud = make_pointcloud(np.zeros((0, 3)))

        results = []
        self.scan_t.subscribe(lambda msg: results.append(msg))
        self.module._on_scan(cloud)

        assert len(results) == 1
        assert len(results[0]) == 0

    def test_odometry_at_scan_published(self):
        """Odometry at scan time should be published."""
        odom = make_odometry(1.0, 2.0, 3.0)
        self.module._on_odometry(odom)

        cloud = make_pointcloud(np.array([[0.0, 0.0, 0.0]]))

        odom_results = []
        self.odom_t.subscribe(lambda msg: odom_results.append(msg))
        self.module._on_scan(cloud)

        assert len(odom_results) == 1
        assert odom_results[0].frame_id == "map"
        assert odom_results[0].child_frame_id == "sensor_at_scan"

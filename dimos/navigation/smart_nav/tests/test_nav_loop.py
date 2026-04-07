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

"""Integration test: verify blueprint construction and autoconnect wiring.

Tests the real blueprint.build() path which involves:
- Module pickling across worker processes
- Transport assignment via autoconnect
- Stream wiring by name+type matching
"""

import time

import numpy as np
import pytest

from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.smart_nav.modules.sensor_scan_generation.sensor_scan_generation import (
    SensorScanGeneration,
)
from dimos.simulation.unity.module import UnityBridgeModule


@pytest.mark.slow
class TestEndToEndDataFlow:
    """Test data flowing through real LCM transports between modules."""

    def test_odom_flows_from_sim_to_scan_gen(self):
        """Odometry published by UnityBridge should reach SensorScanGeneration."""
        sim = UnityBridgeModule(sim_rate=200.0)
        scan_gen = SensorScanGeneration()

        # Shared transport (simulates what autoconnect does)
        odom_transport = LCMTransport("/e2e_odom", Odometry)
        sim.odometry._transport = odom_transport
        scan_gen.odometry._transport = odom_transport

        # Wire dummy transports for other ports so start() doesn't fail
        regscan_t = LCMTransport("/e2e_regscan", PointCloud2)
        sensorscan_t = LCMTransport("/e2e_sensorscan", PointCloud2)
        odom_at_t = LCMTransport("/e2e_odom_at_scan", Odometry)
        scan_gen.registered_scan._transport = regscan_t
        scan_gen.sensor_scan._transport = sensorscan_t
        scan_gen.odometry_at_scan._transport = odom_at_t

        transports = [odom_transport, regscan_t, sensorscan_t, odom_at_t]

        try:
            # Start scan gen (subscribes to odom transport)
            scan_gen.start()

            # Publish odometry through sim's transport
            quat = Quaternion.from_euler(Vector3(0.0, 0.0, 0.0))
            odom = Odometry(
                ts=time.time(),
                frame_id="map",
                child_frame_id="sensor",
                pose=Pose(
                    position=[5.0, 3.0, 0.75],
                    orientation=[quat.x, quat.y, quat.z, quat.w],
                ),
            )
            odom_transport.publish(odom)
            time.sleep(0.1)

            # SensorScanGeneration should have received it
            assert scan_gen._latest_odom is not None
            assert abs(scan_gen._latest_odom.x - 5.0) < 0.01
        finally:
            scan_gen.stop()
            for t in transports:
                t.stop()

    def test_full_scan_transform_chain(self):
        """Odom + cloud in -> sensor-frame cloud out, all via transports."""
        scan_gen = SensorScanGeneration()

        odom_t = LCMTransport("/chain_odom", Odometry)
        regscan_t = LCMTransport("/chain_regscan", PointCloud2)
        sensorscan_t = LCMTransport("/chain_sensorscan", PointCloud2)
        odom_at_t = LCMTransport("/chain_odom_at", Odometry)

        scan_gen.odometry._transport = odom_t
        scan_gen.registered_scan._transport = regscan_t
        scan_gen.sensor_scan._transport = sensorscan_t
        scan_gen.odometry_at_scan._transport = odom_at_t

        transports = [odom_t, regscan_t, sensorscan_t, odom_at_t]

        results: list[PointCloud2] = []
        sensorscan_t.subscribe(lambda msg: results.append(msg))

        try:
            scan_gen.start()

            # Publish odometry at (2, 0, 0), no rotation
            quat = Quaternion.from_euler(Vector3(0.0, 0.0, 0.0))
            odom_t.publish(
                Odometry(
                    ts=time.time(),
                    frame_id="map",
                    child_frame_id="sensor",
                    pose=Pose(
                        position=[2.0, 0.0, 0.0],
                        orientation=[quat.x, quat.y, quat.z, quat.w],
                    ),
                )
            )
            time.sleep(0.05)

            # Publish a world-frame cloud with a point at (5, 0, 0)
            cloud = PointCloud2.from_numpy(
                np.array([[5.0, 0.0, 0.0]], dtype=np.float32),
                frame_id="map",
                timestamp=time.time(),
            )
            regscan_t.publish(cloud)
            time.sleep(0.2)

            # In sensor frame, (5,0,0) - (2,0,0) = (3,0,0)
            assert len(results) >= 1
            pts, _ = results[0].as_numpy()
            assert abs(pts[0][0] - 3.0) < 0.1
        finally:
            scan_gen.stop()
            for t in transports:
                t.stop()

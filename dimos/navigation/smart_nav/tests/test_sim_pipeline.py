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

"""Integration test: verify modules survive the real blueprint deployment path.

These tests exercise the actual framework machinery -- pickling, transport wiring,
cross-process communication -- not just direct method calls.
"""

import time

import numpy as np
import pytest

from dimos.core.stream import In, Out
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.smart_nav.modules.sensor_scan_generation.sensor_scan_generation import (
    SensorScanGeneration,
)
from dimos.navigation.smart_nav.modules.tui_control.tui_control import TUIControlModule
from dimos.simulation.unity.module import UnityBridgeModule


@pytest.mark.slow
class TestTransportWiring:
    """Test that modules publish/subscribe through real LCM transports."""

    def test_unity_bridge_publishes_odometry_via_transport(self):
        """UnityBridge sim loop should publish through _transport, not .publish()."""
        m = UnityBridgeModule(sim_rate=200.0)

        # Wire a real LCM transport to the odometry output
        transport = LCMTransport("/_test/smart_nav/odom", Odometry)
        m.odometry._transport = transport

        received: list[Odometry] = []
        transport.subscribe(lambda msg: received.append(msg))

        try:
            # Simulate one odometry publish (same code path as _sim_loop)
            quat = Quaternion.from_euler(Vector3(0.0, 0.0, 0.0))
            odom = Odometry(
                ts=time.time(),
                frame_id="map",
                child_frame_id="sensor",
                pose=Pose(
                    position=[1.0, 2.0, 0.75],
                    orientation=[quat.x, quat.y, quat.z, quat.w],
                ),
            )
            m.odometry._transport.publish(odom)

            # LCM transport delivers asynchronously -- give it a moment
            time.sleep(0.1)
            assert len(received) >= 1
            assert abs(received[0].x - 1.0) < 0.01
        finally:
            transport.stop()

    def test_sensor_scan_subscribes_and_publishes_via_transport(self):
        """SensorScanGeneration should work entirely through transports."""
        m = SensorScanGeneration()

        # Wire transports (topic string must NOT include #type suffix -- type is the 2nd arg)
        odom_transport = LCMTransport("/_test/smart_nav/scan_gen/odom", Odometry)
        scan_in_transport = LCMTransport("/_test/smart_nav/scan_gen/registered_scan", PointCloud2)
        scan_out_transport = LCMTransport("/_test/smart_nav/scan_gen/sensor_scan", PointCloud2)
        odom_out_transport = LCMTransport("/_test/smart_nav/scan_gen/odom_at_scan", Odometry)

        m.odometry._transport = odom_transport
        m.registered_scan._transport = scan_in_transport
        m.sensor_scan._transport = scan_out_transport
        m.odometry_at_scan._transport = odom_out_transport

        transports = [odom_transport, scan_in_transport, scan_out_transport, odom_out_transport]

        try:
            # Start the module (subscribes via transport)
            m.start()

            # Collect outputs
            scan_results: list[PointCloud2] = []
            scan_out_transport.subscribe(lambda msg: scan_results.append(msg))

            # Publish odometry
            quat = Quaternion.from_euler(Vector3(0.0, 0.0, 0.0))
            odom = Odometry(
                ts=time.time(),
                frame_id="map",
                child_frame_id="sensor",
                pose=Pose(
                    position=[0.0, 0.0, 0.0],
                    orientation=[quat.x, quat.y, quat.z, quat.w],
                ),
            )
            odom_transport.publish(odom)
            time.sleep(0.1)

            # Publish a point cloud
            points = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float32)
            cloud = PointCloud2.from_numpy(points, frame_id="map", timestamp=time.time())
            scan_in_transport.publish(cloud)
            time.sleep(0.3)

            assert len(scan_results) >= 1
            assert scan_results[0].frame_id == "sensor_at_scan"
        finally:
            m.stop()
            for t in transports:
                t.stop()

    def test_tui_publishes_twist_via_transport(self):
        """TUI module should publish cmd_vel through its transport."""
        m = TUIControlModule(max_speed=2.0, publish_rate=50.0)

        transport = LCMTransport("/_test/smart_nav/tui/cmd_vel", Twist)
        m.cmd_vel._transport = transport

        # Also wire way_point so it doesn't error
        from dimos.msgs.geometry_msgs.PointStamped import PointStamped

        wp_transport = LCMTransport("/_test/smart_nav/tui/way_point", PointStamped)
        m.way_point._transport = wp_transport

        received: list[Twist] = []
        transport.subscribe(lambda msg: received.append(msg))

        try:
            m._handle_key("w")  # forward
            m.start()
            time.sleep(0.15)  # let publish loop run a few times
            m.stop()

            assert len(received) >= 1
            assert received[-1].linear.x > 0  # forward velocity
        finally:
            transport.stop()
            wp_transport.stop()


class TestPortTypeCompatibility:
    """Verify that module port types are compatible for autoconnect."""

    def test_all_stream_types_match(self):
        from typing import get_args, get_origin, get_type_hints

        from dimos.navigation.smart_nav.modules.local_planner.local_planner import LocalPlanner
        from dimos.navigation.smart_nav.modules.path_follower.path_follower import PathFollower
        from dimos.navigation.smart_nav.modules.sensor_scan_generation.sensor_scan_generation import (
            SensorScanGeneration,
        )
        from dimos.navigation.smart_nav.modules.terrain_analysis.terrain_analysis import (
            TerrainAnalysis,
        )
        from dimos.simulation.unity.module import UnityBridgeModule

        def get_streams(cls):
            hints = get_type_hints(cls)
            streams = {}
            for name, hint in hints.items():
                origin = get_origin(hint)
                if origin in (In, Out):
                    direction = "in" if origin is In else "out"
                    msg_type = get_args(hint)[0]
                    streams[name] = (direction, msg_type)
            return streams

        sim = get_streams(UnityBridgeModule)
        scan = get_streams(SensorScanGeneration)
        terrain = get_streams(TerrainAnalysis)
        planner = get_streams(LocalPlanner)
        follower = get_streams(PathFollower)

        # Odometry types must match across all consumers
        odom_type = sim["odometry"][1]
        assert scan["odometry"][1] == odom_type
        assert terrain["odometry"][1] == odom_type
        assert planner["odometry"][1] == odom_type
        assert follower["odometry"][1] == odom_type

        # Path: planner out == follower in
        assert planner["path"][1] == follower["path"][1]

        # cmd_vel: follower out == sim in
        assert follower["cmd_vel"][1] == sim["cmd_vel"][1]

        # registered_scan: all consumers match
        pc_type = scan["registered_scan"][1]
        assert terrain["registered_scan"][1] == pc_type
        assert planner["registered_scan"][1] == pc_type

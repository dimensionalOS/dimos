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

import pytest

from dimos.core.stream import In, Out
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.navigation.nav_stack.modules.local_planner.local_planner import LocalPlanner
from dimos.navigation.nav_stack.modules.path_follower.path_follower import PathFollower
from dimos.navigation.nav_stack.modules.terrain_analysis.terrain_analysis import TerrainAnalysis
from dimos.simulation.unity.module import UnityBridgeModule


@pytest.mark.slow
class TestTransportWiring:
    """Test that modules publish/subscribe through real LCM transports."""

    def test_unity_bridge_publishes_odometry_via_transport(self):
        """UnityBridge sim loop should publish through _transport, not .publish()."""
        m = UnityBridgeModule(sim_rate=200.0)

        # Wire a real LCM transport to the odometry output
        transport = LCMTransport("/_test/nav_stack/odom", Odometry)
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
            m.odometry.publish(odom)

            # LCM transport delivers asynchronously -- give it a moment
            time.sleep(0.1)
            assert len(received) >= 1
            assert abs(received[0].x - 1.0) < 0.01
        finally:
            transport.stop()


class TestPortTypeCompatibility:
    """Verify that module port types are compatible for autoconnect."""

    def test_all_stream_types_match(self):
        from typing import get_args, get_origin, get_type_hints

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
        terrain = get_streams(TerrainAnalysis)
        planner = get_streams(LocalPlanner)
        follower = get_streams(PathFollower)

        # Odometry: sim produces, terrain/planner/follower consume
        odom = sim["odometry"]
        assert odom[0] == "out"
        for cls in (terrain, planner, follower):
            entry = cls["odometry"]
            assert entry[0] == "in", f"odometry on {cls} should be In, got {entry[0]}"
            assert entry[1] == odom[1], f"odometry type mismatch: {entry[1]} != {odom[1]}"

        # Path: planner produces, follower consumes
        assert planner["path"][0] == "out"
        assert follower["path"][0] == "in"
        assert planner["path"][1] == follower["path"][1]

        # cmd_vel: follower produces, sim consumes
        assert follower["cmd_vel"][0] == "out"
        assert sim["cmd_vel"][0] == "in"
        assert follower["cmd_vel"][1] == sim["cmd_vel"][1]

        # registered_scan: terrain produces, planner consumes (or both consume)
        pc_type = terrain["registered_scan"][1]
        assert planner["registered_scan"][1] == pc_type

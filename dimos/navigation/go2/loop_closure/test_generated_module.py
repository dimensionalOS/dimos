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

from contextlib import ExitStack
from threading import Event

from dimos_cdrgen.registry import encode
import numpy as np

from dimos.core.cdr_transport import CdrTransport
from dimos.messages.dimos_msgs.msg import LineSegments3D
from dimos.messages.geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, Quaternion
from dimos.messages.sensor_msgs.msg import PointCloud2
from dimos.messages.std_msgs.msg import Header
from dimos.msgs.pointcloud import pointcloud_from_xyz, pointcloud_xyz
from dimos.msgs.time import time_from_nanoseconds
from dimos.navigation.go2.loop_closure.module import PGOVoxelMapper
from dimos.protocol.rpc.pubsubrpc import LCMRPC


class MemoryRPC(LCMRPC):
    def __init__(self, **kwargs):
        super().__init__(url="memq://", **kwargs)


def test_actual_pgo_module_ports_publish_generated_map_graph_and_telemetry():
    with ExitStack() as cleanup:
        module = PGOVoxelMapper(rpc_transport=MemoryRPC, telemetry=True)
        cleanup.callback(module.stop)
        received = {}
        events = {}
        transports = {}

        def collector(name):
            def receive(value):
                received[name] = value
                events[name].set()

            return receive

        for name in (
            "lidar",
            "odom",
            "global_map",
            "pgo_keyframes",
            "pgo_loops",
            "mapper_frame_ms",
            "pgo_loop_ms",
            "pgo_rebuild_ms",
        ):
            port = getattr(module, name)
            transport = CdrTransport(
                "generated_pgo_" + name, port.type, backend="lcm", url="memq://"
            )
            transports[name] = transport
            cleanup.callback(transport.stop)
            port.transport = transport
            if name not in ("lidar", "odom"):
                events[name] = Event()
                cleanup.callback(transport.subscribe(collector(name)))
        assert module.lidar.type is PointCloud2 and module.odom.type is PoseStamped
        assert module.pgo_keyframes.type is PoseArray and module.pgo_loops.type is LineSegments3D
        module.start()
        pose_ready = Event()
        cleanup.callback(module.odom.subscribe(lambda _: pose_ready.set()))
        stamp = time_from_nanoseconds(1700000000123456789)
        header = Header(frame_id="world", stamp=stamp)
        pose = PoseStamped(
            header=header, pose=Pose(position=Point(1, 0, 0), orientation=Quaternion(0, 0, 0, 1))
        )
        transports["odom"].broadcast(None, pose)
        assert pose_ready.wait(5)
        cloud = pointcloud_from_xyz(
            np.array([[1.025, 0.025, 0.025]], dtype=np.float32), header=header
        )
        before = encode(cloud)
        transports["lidar"].broadcast(None, cloud)
        for name in ("global_map", "pgo_keyframes", "pgo_loops", "mapper_frame_ms"):
            assert events[name].wait(5), name
        output = received["global_map"]
        assert output.header == header
        np.testing.assert_allclose(pointcloud_xyz(output), [[1.025, 0.025, 0.025]], atol=1e-6)
        graph = received["pgo_keyframes"]
        assert graph.header == header and len(graph.poses) == 1
        assert graph.poses[0].position.x == 1 and graph.poses[0].orientation.w == 1
        assert received["pgo_loops"].header == header and received["pgo_loops"].segments == []
        assert received["mapper_frame_ms"].data >= 0
        assert encode(cloud) == before

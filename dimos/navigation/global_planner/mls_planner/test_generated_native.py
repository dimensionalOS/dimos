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
from copy import deepcopy
import json
import os
from pathlib import Path as FilePath
import socket
from threading import Event
import time

import numpy as np
import pytest
import zenoh

from dimos.core.cdr_transport import CdrTransport
from dimos.core.global_config import global_config
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.messages.dimos_msgs.msg import LineSegments3DRegion, PointCloudRegion, RegionBounds
from dimos.messages.geometry_msgs.msg import (
    Point,
    PointStamped,
    Quaternion,
    Transform,
    TransformStamped,
    Vector3,
)
from dimos.messages.nav_msgs.msg import Path
from dimos.messages.std_msgs.msg import Header
from dimos.messages.tf2_msgs.msg import TFMessage
from dimos.msgs.pointcloud import pointcloud_from_xyz
from dimos.msgs.time import time_from_nanoseconds
from dimos.navigation.global_planner.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.protocol.rpc.pubsubrpc import LCMRPC
from dimos.protocol.service.zenohservice import ZenohConfig


class MemoryRPC(LCMRPC):
    def __init__(self, **kwargs):
        super().__init__(url="memq://", **kwargs)


def test_mls_native_ports_reuse_canonical_region_and_geometry_types():
    module = MLSPlannerNative(rpc_transport=MemoryRPC)
    try:
        assert module.region_bounds.type is RegionBounds and module.seed_bounds.type is RegionBounds
        assert (
            module.seed_map.type is PointCloudRegion and module.surface_map.type is PointCloudRegion
        )
        assert module.node_edges.type is LineSegments3DRegion
        assert module.goal.type is PointStamped and module.path.type is Path
    finally:
        module.stop()


def test_actual_native_mapper_to_mls_cdr_route_and_weighted_viewer_regions(monkeypatch):
    paths = [
        os.environ.get("DIMOS_NATIVE_RAY_TRACING_BINARY"),
        os.environ.get("DIMOS_NATIVE_MLS_BINARY"),
    ]
    if not all(paths):
        pytest.skip("Set both DIMOS_NATIVE_RAY_TRACING_BINARY and DIMOS_NATIVE_MLS_BINARY")
    mapper_binary, planner_binary = [str(FilePath(path).resolve()) for path in paths]
    assert all(FilePath(path).is_file() for path in [mapper_binary, planner_binary])
    monkeypatch.setattr(global_config, "transport", "zenoh")
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        endpoint = f"tcp/127.0.0.1:{reservation.getsockname()[1]}"
    config = zenoh.Config()
    for key, value in {
        "mode": "router",
        "listen/endpoints": [endpoint],
        "scouting/multicast/enabled": False,
        "scouting/gossip/enabled": False,
    }.items():
        config.insert_json5(key, json.dumps(value))
    with ExitStack() as cleanup:
        cleanup.enter_context(zenoh.open(config))
        session = ZenohConfig(
            mode="client",
            connect=[endpoint],
            listen=[],
            scouting=False,
            multicast=False,
            gossip=False,
            connect_timeout=5,
        )
        mapper = RayTracingVoxelMap(
            rpc_transport=MemoryRPC,
            executable=mapper_binary,
            source_dir=None,
            build_command=None,
            session=session,
            voxel_size=0.2,
            support_min=0,
            min_health=0,
            max_health=1,
            emit_every=1,
            global_emit_every=0,
            viz_emit_every=0,
            world_frame="odom",
        )
        planner = MLSPlannerNative(
            rpc_transport=MemoryRPC,
            executable=planner_binary,
            source_dir=None,
            build_command=None,
            session=session,
            voxel_size=0.2,
            robot_height=1.0,
            start_z_offset_m=1.0,
            world_frame="odom",
            base_frame="base_link",
            viz_publish_hz=10.0,
        )
        wires = {}
        wire_types = {}
        shared = {"local_map", "region_bounds", "seed_map", "seed_bounds", "tf"}
        for owner, name in [
            (mapper, name)
            for name in [
                "lidar",
                "loaded_map",
                "tf",
                "global_map",
                "local_map",
                "local_map_fine",
                "region_bounds",
                "map_regions",
                "seed_map",
                "seed_bounds",
            ]
        ] + [
            (planner, name)
            for name in [
                "global_map",
                "local_map",
                "region_bounds",
                "seed_map",
                "seed_bounds",
                "goal",
                "tf",
                "path",
                "surface_map",
                "nodes",
                "node_edges",
            ]
        ]:
            key = name if name in shared else ("map/" if owner is mapper else "plan/") + name
            port = getattr(owner, name)
            if key not in wires:
                wire_types[key] = port.type
                wires[key] = CdrTransport(
                    "native/mls-chain/" + key, port.type, backend="zenoh", **session.model_dump()
                )
                cleanup.callback(wires[key].stop)
            assert wire_types[key] is port.type
            port.transport = wires[key]
        cleanup.callback(mapper.stop)
        cleanup.callback(planner.stop)
        ready, surface_ready, edges_ready = Event(), Event(), Event()
        all_ready = Event()
        routes, surfaces, edges = [], [], []

        def mark_ready():
            if ready.is_set() and surface_ready.is_set() and edges_ready.is_set():
                all_ready.set()

        def route(value):
            if len(value.poses) >= 2:
                routes.append(value)
                ready.set()
                mark_ready()

        def surface(value):
            if value.cloud.width:
                surfaces.append(value)
                surface_ready.set()
                mark_ready()

        def edge(value):
            if value.lines.segments:
                edges.append(value)
                edges_ready.set()
                mark_ready()

        cleanup.callback(wires["plan/path"].subscribe(route))
        cleanup.callback(wires["plan/surface_map"].subscribe(surface))
        cleanup.callback(wires["plan/node_edges"].subscribe(edge))
        planner.start()
        mapper.start()
        coords = np.arange(-3.0, 3.0, 0.1, dtype=np.float32)
        xx, yy = np.meshgrid(coords, coords)
        # Sensor pose is (-2,-2,1.1); register these points onto the world floor.
        points = np.stack(
            [xx.ravel() + 2, yy.ravel() + 2, np.full(xx.size, -1.1, dtype=np.float32)], axis=-1
        )
        deadline = time.monotonic() + 20
        sequence = 0
        while not all_ready.is_set() and time.monotonic() < deadline:
            header = Header(
                frame_id="odom", stamp=time_from_nanoseconds(1700000000123456789 + sequence)
            )
            sequence += 1
            wires["tf"].publish(
                TFMessage(
                    transforms=[
                        TransformStamped(
                            header=deepcopy(header),
                            child_frame_id=child,
                            transform=Transform(
                                translation=Vector3(x=-2, y=-2, z=1.1),
                                rotation=Quaternion(x=0, y=0, z=0, w=1),
                            ),
                        )
                        for child in ["lidar", "base_link"]
                    ]
                )
            )
            sensor_header = deepcopy(header)
            sensor_header.frame_id = "lidar"
            wires["map/lidar"].publish(pointcloud_from_xyz(points, header=sensor_header))
            wires["plan/goal"].publish(PointStamped(header=header, point=Point(x=2, y=2, z=0)))
            all_ready.wait(0.15)
            for module in [mapper, planner]:
                assert module._process is not None and module._process.poll() is None
        assert ready.is_set() and surface_ready.is_set() and edges_ready.is_set(), (
            "No native CDR route and weighted regions before deadline"
        )
        path = routes[0]
        assert type(path) is Path and path.header.frame_id == "odom"
        assert all(p.header == path.header and type(p.pose.position) is Point for p in path.poses)
        assert (
            np.linalg.norm([path.poses[-1].pose.position.x - 2, path.poses[-1].pose.position.y - 2])
            < 0.4
        )
        assert (
            type(surfaces[0]) is PointCloudRegion and surfaces[0].header == surfaces[0].cloud.header
        )
        assert type(edges[0]) is LineSegments3DRegion and edges[0].header == edges[0].lines.header
        assert all(
            np.isfinite(segment.weight) and segment.weight >= 0
            for segment in edges[0].lines.segments
        )

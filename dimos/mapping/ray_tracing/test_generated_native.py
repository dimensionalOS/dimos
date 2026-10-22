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
from pathlib import Path
import socket
from threading import Event, Lock
import time

import numpy as np
import pytest
import zenoh

from dimos.core.global_config import global_config
from dimos.core.transport import ZenohTransport
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.message_utils.pointcloud import pointcloud_from_xyz, pointcloud_xyz
from dimos.message_utils.time import time_from_nanoseconds, to_nanoseconds
from dimos.messages.dimos_msgs.msg import PointCloudRegion, RegionBounds
from dimos.messages.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos.messages.sensor_msgs.msg import PointCloud2
from dimos.messages.std_msgs.msg import Header
from dimos.messages.tf2_msgs.msg import TFMessage
from dimos.protocol.pubsub.impl.zenohpubsub import Topic as ZenohTopic
from dimos.protocol.rpc.pubsubrpc import LCMRPC
from dimos.protocol.service.zenohservice import ZenohConfig


class MemoryRPC(LCMRPC):
    def __init__(self, **kwargs):
        super().__init__(url="memq://", **kwargs)


def test_native_mapper_declares_canonical_cloud_and_region_ports():
    module = RayTracingVoxelMap(rpc_transport=MemoryRPC)
    try:
        assert module.lidar.type is PointCloud2 and module.global_map.type is PointCloud2
        assert (
            module.map_regions.type is PointCloudRegion and module.seed_map.type is PointCloudRegion
        )
        assert module.region_bounds.type is RegionBounds and module.seed_bounds.type is RegionBounds
        assert module.tf.type is TFMessage
    finally:
        module.stop()


def test_actual_native_mapper_cdr_registered_clouds_keyed_regions_and_seed_pair(monkeypatch):
    executable = os.environ.get("DIMOS_NATIVE_RAY_TRACING_BINARY")
    if not executable:
        pytest.skip("Set DIMOS_NATIVE_RAY_TRACING_BINARY to the compiled voxel_ray_tracing")
    binary = Path(executable).resolve()
    assert binary.is_file()
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
        module = RayTracingVoxelMap(
            rpc_transport=MemoryRPC,
            executable=str(binary),
            source_dir=None,
            build_command=None,
            session=session,
            voxel_size=0.1,
            support_min=0,
            min_health=0,
            max_health=1,
            emit_every=1,
            global_emit_every=1,
            viz_emit_every=1,
            viz_cell_m=4,
            world_frame="odom",
        )
        names = [
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
        wires = {}
        for name in names:
            port = getattr(module, name)
            wire = ZenohTransport(
                ZenohTopic("native/ray/" + name, port.type), **session.model_dump()
            )
            port.transport = wire
            wires[name] = wire
            cleanup.callback(wire.stop)
        cleanup.callback(module.stop)
        live_names = ["global_map", "region_bounds", "map_regions"]
        seed_names = ["seed_map", "seed_bounds"]
        received = {name: {} for name in live_names + seed_names}
        live_ready, seed_ready = Event(), Event()
        lock = Lock()

        def receive(name, value):
            stamp = to_nanoseconds(value.header.stamp)
            group = seed_names if name in seed_names else live_names
            key = (stamp, value.region_id) if name in seed_names else stamp
            with lock:
                received[name].setdefault(key, value)
                if set.intersection(*(set(received[n]) for n in group)):
                    (seed_ready if name in seed_names else live_ready).set()

        for name in received:
            cleanup.callback(wires[name].subscribe(lambda value, name=name: receive(name, value)))
        module.start()
        epoch = 1700000000123456789
        sequence = 0
        source_stamps = set()

        def tf_and_header():
            nonlocal sequence
            header = Header(frame_id="odom", stamp=time_from_nanoseconds(epoch + sequence))
            source_stamps.add(epoch + sequence)
            sequence += 1
            wires["tf"].publish(
                TFMessage(
                    transforms=[
                        TransformStamped(
                            header=deepcopy(header),
                            child_frame_id=child,
                            transform=Transform(
                                translation=Vector3(x=x, y=0, z=0),
                                rotation=Quaternion(x=0, y=0, z=0, w=1),
                            ),
                        )
                        for child, x in [("lidar", 0), ("map", 1)]
                    ]
                )
            )
            return header

        deadline = time.monotonic() + 15
        while not live_ready.is_set() and time.monotonic() < deadline:
            header = tf_and_header()
            header.frame_id = "lidar"
            wires["lidar"].publish(
                pointcloud_from_xyz(
                    np.array([[4.15, -0.85, 0.25], [4.15, -0.85, 0.35]], dtype=np.float32),
                    header=header,
                )
            )
            live_ready.wait(0.15)
        assert live_ready.is_set(), "No matching generated cloud/region/bounds batch"
        with lock:
            stamp = max(set.intersection(*(set(received[n]) for n in live_names)))
            cloud, bounds, region = [received[n][stamp] for n in live_names]
        assert stamp in source_stamps
        assert type(cloud) is PointCloud2 and cloud.header.frame_id == "odom"
        assert bounds.center.x == bounds.center.y == 0 and bounds.radius > 4
        assert region.region_id == (1 << 16) | 0xFFFF
        assert region.header == region.cloud.header
        assert len(pointcloud_xyz(region.cloud)) == 2
        np.testing.assert_allclose(pointcloud_xyz(cloud)[:, 0], 4.15, atol=0.05)

        deadline = time.monotonic() + 15
        while not seed_ready.is_set() and time.monotonic() < deadline:
            header = tf_and_header()
            header.frame_id = "map"
            wires["loaded_map"].publish(
                pointcloud_from_xyz(np.array([[7.15, 2.15, 0.25]], dtype=np.float32), header=header)
            )
            seed_ready.wait(0.15)
        assert seed_ready.is_set(), "No paired canonical seed region and bounds"
        with lock:
            key = max(set.intersection(*(set(received[n]) for n in seed_names)))
            seeded, seed_bounds = [received[n][key] for n in seed_names]
        assert key[0] in source_stamps and seeded.region_id == seed_bounds.region_id
        assert seeded.header == seed_bounds.header == seeded.cloud.header
        assert seeded.cloud.header.frame_id == "odom"
        np.testing.assert_allclose(pointcloud_xyz(seeded.cloud)[:, 0], 8.15, atol=0.05)
        assert module._process is not None and module._process.poll() is None

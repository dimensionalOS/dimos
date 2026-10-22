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
from itertools import pairwise
import json
import os
from pathlib import Path as FilePath
import socket
from threading import Event
import time

import numpy as np
import pytest
import zenoh

from dimos.core.global_config import global_config
from dimos.core.transport import ZenohTransport
from dimos.message_utils.pointcloud import pointcloud_from_xyz
from dimos.message_utils.time import time_from_nanoseconds, to_nanoseconds
from dimos.messages.geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
    Transform,
    TransformStamped,
    Vector3,
)
from dimos.messages.nav_msgs.msg import Path
from dimos.messages.sensor_msgs.msg import PointCloud2
from dimos.messages.std_msgs.msg import Header
from dimos.messages.tf2_msgs.msg import TFMessage
from dimos.navigation.embodiment.go2 import GO2
from dimos.navigation.local_planner.native import LocalPlannerNative
from dimos.protocol.pubsub.impl.zenohpubsub import Topic as ZenohTopic
from dimos.protocol.rpc.pubsubrpc import LCMRPC
from dimos.protocol.service.zenohservice import ZenohConfig


class MemoryRPC(LCMRPC):
    def __init__(self, **kwargs):
        super().__init__(url="memq://", **kwargs)


def test_native_planner_declares_canonical_generated_port_identities():
    module = LocalPlannerNative(rpc_transport=MemoryRPC)
    try:
        assert module.local_map.type is PointCloud2
        assert module.planner_path.type is Path and module.path.type is Path
        assert module.tf.type is TFMessage
    finally:
        module.stop()


def test_actual_native_planner_consumes_python_cdr_and_returns_canonical_path(monkeypatch):
    executable = os.environ.get("DIMOS_NATIVE_LOCAL_PLANNER_BINARY")
    if not executable:
        pytest.skip("Set DIMOS_NATIVE_LOCAL_PLANNER_BINARY to the compiled local_planner")
    binary = FilePath(executable).resolve()
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
        module = LocalPlannerNative(
            rpc_transport=MemoryRPC,
            executable=str(binary),
            source_dir=None,
            build_command=None,
            session=session,
        )
        wires = {}
        for name in ["local_map", "planner_path", "tf", "path"]:
            port = getattr(module, name)
            wire = ZenohTransport(
                ZenohTopic("native/planner/" + name, port.type), **session.model_dump()
            )
            port.transport = wire
            wires[name] = wire
            cleanup.callback(wire.stop)
        cleanup.callback(module.stop)
        received = []
        ready = Event()

        def receive(value):
            if len(value.poses) >= 2:
                received.append(value)
                ready.set()

        cleanup.callback(wires["path"].subscribe(receive))
        module.start()
        points = np.array(
            [[x, y, 0.0] for x in np.arange(-1.0, 4.1, 0.25) for y in np.arange(-2.0, 2.1, 0.25)],
            dtype=np.float32,
        )
        deadline = time.monotonic() + 15
        index = 0
        while not ready.is_set() and time.monotonic() < deadline:
            header = Header(
                frame_id="odom", stamp=time_from_nanoseconds(1700000000123456789 + index)
            )
            wires["tf"].publish(
                TFMessage(
                    transforms=[
                        TransformStamped(
                            header=deepcopy(header),
                            child_frame_id="base_link",
                            transform=Transform(
                                translation=Vector3(x=0, y=0, z=GO2.base_height),
                                rotation=Quaternion(x=0, y=0, z=0, w=1),
                            ),
                        )
                    ]
                )
            )
            wires["local_map"].publish(pointcloud_from_xyz(points, header=deepcopy(header)))
            route = Path(
                header=deepcopy(header),
                poses=[
                    PoseStamped(
                        header=deepcopy(header),
                        pose=Pose(
                            position=Point(x=x, y=0, z=0),
                            orientation=Quaternion(x=0, y=0, z=0, w=1),
                        ),
                    )
                    for x in [0.0, 1.0, 2.0]
                ],
            )
            wires["planner_path"].publish(route)
            index += 1
            ready.wait(0.2)
        assert ready.is_set(), "No native CDR route before the bounded deadline"
        result = received[0]
        assert type(result) is Path and type(result.header) is Header
        assert result.header.frame_id == "odom"
        assert result.poses[-1].pose.position.x > 1.5
        assert abs(result.poses[-1].pose.position.y) < 0.2
        stamps = [to_nanoseconds(p.header.stamp) for p in result.poses]
        assert all(b >= a for a, b in pairwise(stamps))
        assert module._process is not None and module._process.poll() is None

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

import pytest
import zenoh

from dimos.core.cdr_transport import CdrTransport
from dimos.core.global_config import global_config
from dimos.messages.geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
    Transform,
    TransformStamped,
    Twist,
    Vector3,
)
from dimos.messages.nav_msgs.msg import Path
from dimos.messages.std_msgs.msg import Bool, Header
from dimos.messages.tf2_msgs.msg import TFMessage
from dimos.msgs.time import time_from_nanoseconds
from dimos.navigation.trajectory_follower.fancy.native import TrajectoryFollowerNative
from dimos.protocol.rpc.pubsubrpc import LCMRPC
from dimos.protocol.service.zenohservice import ZenohConfig


class MemoryRPC(LCMRPC):
    def __init__(self, **kwargs):
        super().__init__(url="memq://", **kwargs)


def test_native_follower_declares_canonical_generated_port_identities():
    module = TrajectoryFollowerNative(rpc_transport=MemoryRPC)
    try:
        assert module.path.type is Path and module.tf.type is TFMessage
        assert module.nav_cmd_vel.type is Twist and module.goal_reached.type is Bool
    finally:
        module.stop()


def test_actual_native_follower_cdr_drive_arrival_new_goal_and_empty_stop(monkeypatch):
    executable = os.environ.get("DIMOS_NATIVE_FOLLOWER_BINARY")
    if not executable:
        pytest.skip("Set DIMOS_NATIVE_FOLLOWER_BINARY to the compiled trajectory_follower")
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
        module = TrajectoryFollowerNative(
            rpc_transport=MemoryRPC,
            executable=str(binary),
            source_dir=None,
            build_command=None,
            session=session,
        )
        wires = {}
        for name in ["path", "tf", "nav_cmd_vel", "goal_reached"]:
            port = getattr(module, name)
            wire = CdrTransport(
                "native/follower/" + name, port.type, backend="zenoh", **session.model_dump()
            )
            port.transport = wire
            wires[name] = wire
            cleanup.callback(wire.stop)
        cleanup.callback(module.stop)
        moving, arrived, stopped = Event(), Event(), Event()
        commands, arrivals = [], []

        def command(value):
            commands.append(value)
            if value.linear.x > 0:
                moving.set()
            if value.linear.x == value.linear.y == value.angular.z == 0:
                stopped.set()

        def arrival(value):
            arrivals.append(value)
            if value.data:
                arrived.set()

        cleanup.callback(wires["nav_cmd_vel"].subscribe(command))
        cleanup.callback(wires["goal_reached"].subscribe(arrival))
        module.start()
        epoch = 1700000000123456789
        sequence = 0

        def feed(x, route):
            nonlocal sequence
            header = Header(frame_id="odom", stamp=time_from_nanoseconds(epoch + sequence))
            sequence += 1
            wires["tf"].publish(
                TFMessage(
                    transforms=[
                        TransformStamped(
                            header=header,
                            child_frame_id="base_link",
                            transform=Transform(
                                translation=Vector3(x=x, y=0, z=0),
                                rotation=Quaternion(x=0, y=0, z=0, w=1),
                            ),
                        )
                    ]
                )
            )
            wires["path"].publish(route)

        def route(start):
            return Path(
                header=Header(frame_id="odom", stamp=time_from_nanoseconds(epoch)),
                poses=[
                    PoseStamped(
                        header=Header(
                            frame_id="odom", stamp=time_from_nanoseconds(epoch + i * 125000000)
                        ),
                        pose=Pose(
                            position=Point(x=start + i * 0.125, y=0, z=0),
                            orientation=Quaternion(x=0, y=0, z=0, w=1),
                        ),
                    )
                    for i in range(17)
                ],
            )

        def until(event, x, path):
            deadline = time.monotonic() + 10
            while not event.is_set() and time.monotonic() < deadline:
                feed(x, path)
                event.wait(0.15)
            assert event.is_set(), "Native follower did not reach the expected state"

        first = route(0)
        until(moving, 0, first)
        assert type(commands[-1]) is Twist
        until(arrived, 2, first)
        assert type(arrivals[-1]) is Bool and arrivals[-1].data
        moving.clear()
        second = route(2)
        until(moving, 2, second)
        stopped.clear()
        until(stopped, 2, Path(header=deepcopy(second.header), poses=[]))
        assert commands[-1].linear.x == commands[-1].linear.y == commands[-1].angular.z == 0
        assert module._process is not None and module._process.poll() is None

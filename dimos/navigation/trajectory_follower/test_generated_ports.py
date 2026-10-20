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
from threading import Event

import pytest

from dimos.core.cdr_transport import CdrTransport
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
from dimos.messages.std_msgs.msg import Bool, Header
from dimos.messages.tf2_msgs.msg import TFMessage
from dimos.msgs.time import time_from_nanoseconds
from dimos.navigation.trajectory_follower.basic.module import BasicPathFollower
from dimos.navigation.trajectory_follower.fancy.module import TrajectoryFollower
from dimos.protocol.rpc.pubsubrpc import LCMRPC


class MemoryRPC(LCMRPC):
    def __init__(self, **kwargs):
        super().__init__(url="memq://", **kwargs)


def route(header, end):
    return Path(
        header=deepcopy(header),
        poses=[
            PoseStamped(
                header=deepcopy(header),
                pose=Pose(
                    position=Point(x=x, y=0, z=0), orientation=Quaternion(x=0, y=0, z=0, w=1)
                ),
            )
            for x in [end * index / 16 for index in range(17)]
        ],
    )


def base_transform(header, x):
    return TFMessage(
        transforms=[
            TransformStamped(
                header=deepcopy(header),
                child_frame_id="base_link",
                transform=Transform(
                    translation=Vector3(x=x, y=0, z=0.01), rotation=Quaternion(x=0, y=0, z=0, w=1)
                ),
            )
        ]
    )


@pytest.mark.parametrize("follower_type", [BasicPathFollower, TrajectoryFollower])
def test_actual_generated_follower_ports_drive_arrive_and_preempt_over_cdr(follower_type):
    with ExitStack() as cleanup:
        follower = follower_type(rpc_transport=MemoryRPC)
        wires = {}
        for name in ["tf", "path", "nav_cmd_vel", "goal_reached"]:
            port = getattr(follower, name)
            wire = CdrTransport("follower_" + name, port.type, backend="lcm", url="memq://")
            port.transport = wire
            wires[name] = wire
            cleanup.callback(wire.stop)
        cleanup.callback(follower.stop)
        commands = []
        goals = []
        moving = Event()
        arrived = Event()
        zeroed = Event()

        def receive_command(value):
            commands.append(value)
            if value.linear.x > 0:
                moving.set()
            elif value.linear.x == 0 and value.linear.y == 0 and value.angular.z == 0:
                zeroed.set()

        def receive_goal(value):
            goals.append(value)
            if value.data:
                arrived.set()

        cleanup.callback(wires["nav_cmd_vel"].subscribe(receive_command))
        cleanup.callback(wires["goal_reached"].subscribe(receive_goal))
        follower.start()
        header = Header(frame_id="odom", stamp=time_from_nanoseconds(1700000000123456789))
        wires["tf"].publish(base_transform(header, 0.0))
        wires["path"].publish(route(header, 2.0))
        assert moving.wait(5)
        header.stamp = time_from_nanoseconds(1700000000123456790)
        wires["tf"].publish(base_transform(header, 2.0))
        assert arrived.wait(5)
        assert goals == [Bool(data=True)]
        moving.clear()
        wires["path"].publish(route(header, 4.0))
        assert moving.wait(5)
        zeroed.clear()
        wires["path"].publish(Path(header=deepcopy(header), poses=[]))
        assert zeroed.wait(5)
        assert any(c.linear.x > 0 for c in commands)

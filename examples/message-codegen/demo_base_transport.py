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

"""Exchange generated base commands and odometry without robot hardware."""

from contextlib import ExitStack
import math
import socket
import threading
from typing import Any
import uuid

from demo_pubsub import free_port
from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist, Vector3

from dimos.core.transport import LCMTransport, PubSubTransport, ZenohTransport
from dimos.hardware.drive_trains.transport.adapter import TransportTwistAdapter
from dimos.protocol.service.zenohservice import ZenohSessionPool
from dimos.utils.testing.waiting import retry_until, wait_until


def demonstrate(backend: str) -> None:
    with ExitStack() as stack:
        pools = [ZenohSessionPool(), ZenohSessionPool()]
        for pool in pools:
            stack.callback(pool.close_all)
        endpoint = f"tcp/127.0.0.1:{free_port(socket.SOCK_STREAM)}"
        url = f"udpm://239.255.76.67:{free_port(socket.SOCK_DGRAM)}?ttl=0"
        prefix = f"base_{uuid.uuid4().hex[:8]}"

        def transport(topic: str, message_type: type, peer: int = 0) -> Any:
            result: PubSubTransport[Any]
            if backend == "lcm":
                result = LCMTransport(topic, message_type, url=url)
            else:
                result = ZenohTransport(
                    "dimos/" + topic.lstrip("/"),
                    message_type,
                    session_pool=pools[peer],
                    scouting=False,
                    multicast=False,
                    gossip=False,
                    listen=[endpoint] if peer == 0 else [],
                    connect=[] if peer == 0 else [endpoint],
                )
            stack.callback(result.stop)
            result.start()
            return result

        adapter = TransportTwistAdapter(hardware_id=prefix, transport_cls=transport)
        stack.callback(adapter.disconnect)
        assert adapter.connect()
        odom = transport(f"/{prefix}/odom", PoseStamped, 1)
        commands = transport(f"/{prefix}/cmd_vel", Twist, 1)
        moving, stopped = threading.Event(), threading.Event()
        received = []

        def collect(message: Twist) -> None:
            received.append(message)
            (stopped if message == Twist() else moving).set()

        stack.callback(commands.subscribe(collect))
        source = PoseStamped(
            pose=Pose(
                position=Point(x=2.0, y=-1.0),
                orientation=Quaternion(z=math.sqrt(0.5), w=math.sqrt(0.5)),
            )
        )

        def feedback_received() -> bool:
            odom.publish(source)
            return adapter.read_odometry() is not None

        wait_until(feedback_received, timeout=5)
        feedback = adapter.read_odometry()
        assert feedback is not None and feedback[:2] == [2.0, -1.0]
        assert math.isclose(feedback[2], math.pi / 2)
        assert not adapter.write_velocities([0.5, -0.2, 0.3])
        adapter.write_enable(True)

        def send_command() -> None:
            assert adapter.write_velocities([0.5, -0.2, 0.3])

        retry_until(moving, send_command, timeout=5)
        assert received[0] == Twist(linear=Vector3(x=0.5, y=-0.2), angular=Vector3(z=0.3))
        adapter.write_enable(False)
        assert stopped.wait(5), "Disabling the adapter did not deliver a generated stop command"
        assert adapter.read_velocities() == [0.0, 0.0, 0.0]
        print(f"{backend}: odometry={feedback}, command=(0.5,-0.2,0.3), disable=(0,0,0)")
    print(f"PASS: {backend} generated base commands, odometry, and stop")


if __name__ == "__main__":
    for backend in ("lcm", "zenoh"):
        demonstrate(backend)

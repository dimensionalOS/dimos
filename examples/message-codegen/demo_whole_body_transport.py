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

"""Exchange generated motor commands and feedback through the whole-body adapter."""

from contextlib import ExitStack
import socket
import threading
from typing import Any
import uuid

from demo_pubsub import free_port
from dimos_generated.dimos_msgs.msg import MotorCommandArray
from dimos_generated.geometry_msgs.msg import Quaternion, Vector3
from dimos_generated.sensor_msgs.msg import Imu, JointState

from dimos.core.transport import LCMTransport, PubSubTransport, ZenohTransport
from dimos.hardware.whole_body.spec import MotorCommand
from dimos.hardware.whole_body.transport.adapter import TransportWholeBodyAdapter
from dimos.protocol.service.zenohservice import ZenohSessionPool
from dimos.utils.testing.waiting import retry_until, wait_until


def demonstrate(backend: str) -> None:
    with ExitStack() as stack:
        pools = [ZenohSessionPool(), ZenohSessionPool()]
        for pool in pools:
            stack.callback(pool.close_all)
        endpoint = f"tcp/127.0.0.1:{free_port(socket.SOCK_STREAM)}"
        url = f"udpm://239.255.76.67:{free_port(socket.SOCK_DGRAM)}?ttl=0"
        prefix = f"wb_{uuid.uuid4().hex[:8]}"

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

        adapter = TransportWholeBodyAdapter(dof=2, hardware_id=prefix, transport_cls=transport)
        stack.callback(adapter.disconnect)
        assert adapter.connect()
        states = transport(f"/{prefix}/motor_states", JointState, 1)
        imu = transport(f"/{prefix}/imu", Imu, 1)
        commands = transport(f"/{prefix}/motor_command", MotorCommandArray, 1)
        received = []
        ready = threading.Event()

        def collect(message: MotorCommandArray) -> None:
            received.append(message)
            ready.set()

        stack.callback(commands.subscribe(collect))

        def feedback_received() -> bool:
            states.publish(
                JointState(position=[0.25, -0.5], velocity=[0.1, 0.2], effort=[1.0, 2.0])
            )
            imu.publish(Imu(orientation=Quaternion(w=0.8, z=0.6), angular_velocity=Vector3(z=0.2)))
            return adapter.has_motor_states() and adapter.read_imu().gyroscope[2] == 0.2

        wait_until(feedback_received, timeout=5)
        feedback = adapter.read_motor_states()
        assert [value.q for value in feedback] == [0.25, -0.5]
        assert adapter.read_imu().quaternion == (0.8, 0.0, 0.0, 0.6)

        def send_command() -> None:
            assert adapter.write_motor_commands(
                [
                    MotorCommand(q=0.75, dq=0.1, kp=5.0, kd=0.5, tau=1.0),
                    MotorCommand(q=-0.25, dq=0.2, kp=6.0, kd=0.6, tau=2.0),
                ]
            )

        retry_until(ready, send_command, timeout=5)
        message = received[0]
        assert list(message.q) == [0.75, -0.25]
        assert list(message.kp) == [5.0, 6.0]
        assert list(message.tau) == [1.0, 2.0]
        assert message.header.stamp.sec > 0
        print(
            f"{backend}: feedback q={[value.q for value in feedback]}, command q={list(message.q)}, IMU wxyz={adapter.read_imu().quaternion}"
        )
    print(f"PASS: {backend} whole-body generated CDR commands and feedback")


if __name__ == "__main__":
    for backend in ("lcm", "zenoh"):
        demonstrate(backend)

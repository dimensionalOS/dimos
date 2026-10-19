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

"""Generate, transport, execute, and inspect a CDR joint trajectory without hardware."""

from contextlib import ExitStack
import math
import socket
import threading
from typing import Any
import uuid

from demo_pubsub import free_port
from dimos_generated.dimos_msgs.msg import TrajectoryStatus
from dimos_generated.trajectory_msgs.msg import JointTrajectory

from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.trajectory_task.trajectory_task import (
    JointTrajectoryTask,
    JointTrajectoryTaskConfig,
    TrajectoryExecutionStatus,
)
from dimos.core.transport import LCMTransport, PubSubTransport, ZenohTransport
from dimos.manipulation.planning.trajectory_generator.joint_trajectory_generator import (
    JointTrajectoryGenerator,
)
from dimos.msgs.time import time_from_nanoseconds, to_nanoseconds, to_seconds
from dimos.msgs.trajectory import TrajectoryState, trajectory_duration
from dimos.protocol.service.zenohservice import ZenohSessionPool
from dimos.utils.testing.waiting import retry_until


def demonstrate(backend: str) -> None:
    with ExitStack() as stack:
        pools = [ZenohSessionPool(), ZenohSessionPool()]
        for pool in pools:
            stack.callback(pool.close_all)
        endpoint = f"tcp/127.0.0.1:{free_port(socket.SOCK_STREAM)}"
        url = f"udpm://239.255.76.67:{free_port(socket.SOCK_DGRAM)}?ttl=0"
        prefix = f"trajectory_{uuid.uuid4().hex[:8]}"

        def transport(channel: str, message_type: type, peer: int) -> PubSubTransport[Any]:
            result: PubSubTransport[Any]
            if backend == "lcm":
                result = LCMTransport(channel, message_type, url=url)
            else:
                result = ZenohTransport(
                    channel,
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

        sender = transport(prefix + "/plan", JointTrajectory, 0)
        receiver = transport(prefix + "/plan", JointTrajectory, 1)
        feedback_sender = transport(prefix + "/status", TrajectoryStatus, 1)
        feedback_receiver = transport(prefix + "/status", TrajectoryStatus, 0)
        received: list[JointTrajectory] = []
        feedback: list[TrajectoryStatus] = []
        ready, completed = threading.Event(), threading.Event()

        def receive(plan: JointTrajectory) -> None:
            received.append(plan)
            ready.set()

        def receive_status(status: TrajectoryStatus) -> None:
            feedback.append(status)
            if status.state == TrajectoryStatus.COMPLETED:
                completed.set()

        stack.callback(receiver.subscribe(receive))
        stack.callback(feedback_receiver.subscribe(receive_status))
        source = JointTrajectoryGenerator(
            num_joints=2, max_velocity=1.0, max_acceleration=2.0
        ).generate([[0.0, 0.0], [0.3, -0.2]])
        source.joint_names = ["shoulder", "wrist"]
        source.header.frame_id = "robot"
        source.header.stamp = time_from_nanoseconds(1700000000123456789)
        retry_until(ready, lambda: sender.publish(source), timeout=5)
        plan = received[0]
        assert plan == source
        task = JointTrajectoryTask(JointTrajectoryTaskConfig(joint_names=tuple(plan.joint_names)))
        positions = dict.fromkeys(plan.joint_names, 0.0)
        assert task.execute(plan, positions).status == TrajectoryExecutionStatus.ACCEPTED
        duration = trajectory_duration(plan)
        print(
            f"{backend}: received {len(plan.points)} generated points, duration={duration:.9f}s, source ns={to_nanoseconds(plan.header.stamp)}"
        )
        steps = math.ceil(duration / 0.05) + 3
        for tick in range(steps):
            now = tick * 0.05
            output = task.compute(
                CoordinatorState(
                    joints=JointStateSnapshot(joint_positions=positions), t_now=now, dt=0.05
                )
            )
            if output is not None:
                assert output.positions is not None
                positions = dict(zip(output.joint_names, output.positions, strict=True))
            status = task.get_status(now)
            feedback_sender.publish(status)
            if tick % max(1, steps // 4) == 0:
                print(
                    f"  t={now:.2f}s q={list(positions.values())}, status={TrajectoryState(status.state).name}"
                )
        terminal = task.get_status(steps * 0.05)
        assert terminal.state == TrajectoryStatus.COMPLETED
        retry_until(completed, lambda: feedback_sender.publish(terminal), timeout=5)
        final = next(value for value in feedback if value.state == TrajectoryStatus.COMPLETED)
        assert final.progress == 1.0 and to_seconds(final.time_remaining) == 0.0
        assert math.isclose(to_seconds(final.time_elapsed), duration, abs_tol=1e-9)
        assert math.isclose(positions["shoulder"], 0.3, abs_tol=1e-9)
        assert math.isclose(positions["wrist"], -0.2, abs_tol=1e-9)
    print(f"PASS: {backend} generated trajectory and status reach the declared goal")


if __name__ == "__main__":
    for backend in ("lcm", "zenoh"):
        demonstrate(backend)

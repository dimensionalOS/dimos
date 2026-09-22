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

"""Inspect generated joint-state streams from a running two-arm mock coordinator."""

from contextlib import ExitStack
import threading

from dimos_generated.sensor_msgs.msg import JointState

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import ControlCoordinator
from dimos.core.stream import Out
from dimos.msgs.time import to_nanoseconds


class DemoCoordinator(ControlCoordinator):
    left_arm_joints: Out[JointState]
    right_arm_joints: Out[JointState]


def main() -> None:
    coordinator = DemoCoordinator(
        instance_name="ControlCoordinator",
        publish_robot_joint_states=True,
        hardware=[
            HardwareComponent(
                hardware_id=name,
                hardware_type=HardwareType.MANIPULATOR,
                joints=[f"{name}/joint1", f"{name}/joint2"],
                adapter_type="mock",
                adapter_kwargs={"initial_positions": positions},
            )
            for name, positions in [("left_arm", [0.1, 0.2]), ("right_arm", [0.3, 0.4])]
        ],
    )
    ready = threading.Event()
    samples: dict[int, dict[str, JointState]] = {}
    with ExitStack() as stack:
        stack.callback(coordinator.stop)

        def receive(port: str, message: JointState) -> None:
            tick = samples.setdefault(to_nanoseconds(message.header.stamp), {})
            tick[port] = message
            if len(tick) == 3:
                ready.set()

        for name in ["coordinator_joint_state", "left_arm_joints", "right_arm_joints"]:
            stack.callback(
                getattr(coordinator, name).subscribe(
                    lambda message, name=name: receive(name, message)
                )
            )
        coordinator.start()
        assert ready.wait(5), "Coordinator did not publish a full generated feedback tick"
        # Stop the tick loop before reading the accumulated snapshot.
        coordinator.stop()
        complete = next(tick for tick in samples.values() if len(tick) == 3)
        assert list(complete["left_arm_joints"].position) == [0.1, 0.2]
        assert list(complete["right_arm_joints"].position) == [0.3, 0.4]
        assert len(complete["coordinator_joint_state"].position) == 4
        for port, message in complete.items():
            assert JointState.decode(message.encode()) == message
            print(
                f"{port}: frame={message.header.frame_id}, names={list(message.name)}, q={list(message.position)}, stamp={to_nanoseconds(message.header.stamp)}"
            )
    print("PASS: aggregate and per-robot streams use generated JointState and share one tick stamp")


if __name__ == "__main__":
    main()

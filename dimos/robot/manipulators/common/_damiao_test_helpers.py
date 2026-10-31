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

"""Real mock connections and coordinators joined by in-memory test transports."""

from collections.abc import Callable, Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
import threading
from typing import Any, Generic, TypeVar
from unittest.mock import PropertyMock
import xml.etree.ElementTree as ET

from pytest_mock import MockerFixture

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tick_loop import TickLoop
from dimos.core.stream import Stream, Transport
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.common.damiao_connection import DamiaoConnection
from dimos.utils.testing.waiting import wait_until

T = TypeVar("T")


class LocalTransport(Transport[T], Generic[T]):
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._messages: list[T] = []
        self._subscribers: list[Callable[[T], Any]] = []

    @property
    def latest(self) -> T:
        with self._lock:
            assert self._messages, "No message arrived on this transport"
            return self._messages[-1]

    def broadcast(self, selfstream: Stream[T] | None, value: T) -> None:
        with self._lock:
            self._messages.append(value)
            callbacks = tuple(self._subscribers)
        for callback in callbacks:
            callback(value)

    def subscribe(
        self, callback: Callable[[T], Any], selfstream: Stream[T] | None = None
    ) -> Callable[[], None]:
        with self._lock:
            self._subscribers.append(callback)

        def dispose() -> None:
            with self._lock:
                if callback in self._subscribers:
                    self._subscribers.remove(callback)

        return dispose

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass


@dataclass
class MockConnectionRig:
    coordinator: ControlCoordinator
    connection: DamiaoConnection
    inputs: dict[str, LocalTransport[Any]]
    readings: LocalTransport[JointState]
    sent_commands: LocalTransport[JointState]

    @property
    def positions(self) -> dict[str, float]:
        message = self.readings.latest
        return dict(zip(message.name, message.position, strict=True))

    @property
    def commands(self) -> JointState:
        return self.sent_commands.latest

    def tick(self) -> None:
        assert self.coordinator._tick_loop is not None
        self.coordinator._tick_loop._tick()
        command = self.commands
        expected = dict(zip(command.name, command.position, strict=True))
        wait_until(
            lambda: all(
                abs(self.positions[name] - value) < 1e-7 for name, value in expected.items()
            ),
            timeout=2,
        )


def _local_test_model(connection: type[DamiaoConnection], path: Path) -> RobotModel:
    root = ET.Element("robot", name="test")
    ET.SubElement(root, "link", name="base")
    grippers = {gripper.name for gripper in connection.robot.grippers}
    for index, name in enumerate(connection.robot.joint_names):
        child = f"link{index}"
        ET.SubElement(root, "link", name=child)
        joint = ET.SubElement(root, "joint", name=name, type="revolute")
        ET.SubElement(joint, "parent", link="base")
        ET.SubElement(joint, "child", link=child)
        ET.SubElement(
            joint,
            "limit",
            lower="0" if name in grippers else "-3.2",
            upper="1" if name in grippers else "3.2",
            velocity="2",
            effort="100",
        )
    path.write_text(ET.tostring(root, encoding="unicode"))
    return RobotModel.from_file(path)


@contextmanager
def mock_connection_rig(
    connection_type: type[DamiaoConnection],
    coordinator: ControlCoordinator,
    *,
    tmp_path: Path,
    mocker: MockerFixture,
) -> Iterator[MockConnectionRig]:
    """Exercise the real connection path; only geometry and native boundaries are fixtures."""
    connection = None
    try:
        mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.start")
        mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.serve_module_rpc")
        model = _local_test_model(connection_type, tmp_path / "control.urdf")
        connection = connection_type(model=model, source="arm", deadman_timeout_s=10)
        coordinator.config.model = model.with_renamed_joints(
            {name: f"arm/{name}" for name in connection_type.robot.joint_names}
        )
        inputs = {name: LocalTransport[Any]() for name in coordinator.inputs}
        for name, transport in inputs.items():
            coordinator.set_transport(name, transport)
        for name in coordinator.outputs:
            coordinator.set_transport(name, LocalTransport[Any]())
        readings = LocalTransport[JointState]()
        commands = LocalTransport[JointState]()
        coordinator.set_transport("arm_joint_state", readings)
        coordinator.set_transport("arm_position_command", commands)
        connection.set_transport("joint_state", readings)
        connection.set_transport("position_command", commands)
        mocker.patch.object(TickLoop, "start")
        mocker.patch.object(TickLoop, "is_running", new_callable=PropertyMock, return_value=True)
        coordinator.start()
        connection.start()
        wait_until(lambda: coordinator._connections["arm"].ready_for_control(), timeout=2)
        rig = MockConnectionRig(coordinator, connection, inputs, readings, commands)
        rig.tick()
        yield rig
    finally:
        try:
            coordinator.stop()
        finally:
            if connection is not None:
                connection.stop()

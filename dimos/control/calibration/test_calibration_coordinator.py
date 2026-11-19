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

"""The calibration coordinator: targets reach the connection on the next tick, unchanged.

Once they stop, nothing reaches it, so its deadman halts the arm.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
import math
import threading
import time
from typing import Any

import pytest

from dimos.control._control_test_helpers import FAKE_ARM_JOINTS, FakeArm, fake_arm_model
from dimos.control.calibration.coordinator import (
    CALIBRATION_COMMAND_PORT,
    CALIBRATION_PASSTHROUGH_TASK,
    CalibrationCoordinator,
    calibration_passthrough_task,
)
from dimos.control.connection import connection_module
from dimos.control.connection.connection_module import ConnectionConfig
from dimos.control.connection.module_config import ConfigPatch
from dimos.control.contract.keys import Interface
from dimos.control.tick_loop import TickLoop
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.testing.waiting import wait_until

ARM_JOINTS = [f"arm/{joint}" for joint in FAKE_ARM_JOINTS]


class LocalTransport:
    """An in-process transport that keeps what was sent."""

    def __init__(self) -> None:
        self.sent: list[Any] = []
        self.subscribers: list[Callable[[Any], Any]] = []

    def broadcast(self, selfstream: Any, value: Any) -> None:
        self.sent.append(value)
        for callback in list(self.subscribers):
            callback(value)

    def publish(self, value: Any) -> None:
        self.broadcast(None, value)

    def subscribe(
        self, callback: Callable[[Any], Any], selfstream: Any = None
    ) -> Callable[[], None]:
        self.subscribers.append(callback)
        return lambda: self.subscribers.remove(callback)

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass


def _declaration() -> ConnectionConfig:
    return ConnectionConfig(
        source="arm",
        model=fake_arm_model(),
        joint_commands={joint: (Interface.POSITION,) for joint in FAKE_ARM_JOINTS},
    )


class Rig:
    """A calibration coordinator with an in-process transport on every port."""

    def __init__(self, **config: Any) -> None:
        self.coordinator = CalibrationCoordinator(
            connections=[_declaration()],
            tasks=[calibration_passthrough_task(ARM_JOINTS)],
            zero_stiffness_joints=[],
            **config,
        )
        self.ports = {
            name: LocalTransport()
            for name in {**self.coordinator.inputs, **self.coordinator.outputs}
        }
        for name, transport in self.ports.items():
            self.coordinator.set_transport(name, transport)  # type: ignore[arg-type]

    def start(self) -> None:
        self.coordinator.start()

    def tick(self) -> None:
        loop = self.coordinator._tick_loop
        assert loop is not None
        loop._tick()

    def report(self, *positions: float) -> None:
        self.ports["arm_joint_state"].publish(
            JointState(ts=time.time(), name=ARM_JOINTS, position=list(positions))
        )

    def target(self, **positions: float) -> None:
        self.ports[CALIBRATION_COMMAND_PORT].publish(
            JointState(
                ts=time.time(),
                name=[f"arm/{joint}" for joint in positions],
                position=list(positions.values()),
            )
        )

    def commands(self) -> list[JointState]:
        return list(self.ports["arm_position_command"].sent)


@pytest.fixture
def manual(mocker: Any) -> Iterator[Callable[..., Rig]]:
    """Coordinators whose tick loop runs only when ``tick`` is called."""
    mocker.patch.object(TickLoop, "start", lambda loop: loop._stop_event.clear())
    made: list[Rig] = []

    def build(**config: Any) -> Rig:
        made.append(Rig(**config))
        made[-1].start()
        return made[-1]

    yield build
    for rig in made:
        rig.coordinator._tick_loop._stop_event.set()  # type: ignore[union-attr]
        rig.coordinator.stop()


AT_REST = [0.0] * len(ARM_JOINTS)


def test_a_step_reaches_the_connection_on_the_next_tick_unchanged(
    manual: Callable[..., Rig],
) -> None:
    rig = manual()
    rig.report(*AT_REST)
    rig.target(joint1=0.1)
    rig.tick()
    assert rig.commands()[-1].position[0] == 0.1

    step = 0.1 + math.radians(30.0)
    rig.target(joint1=step)
    rig.tick()

    sent = rig.commands()[-1]
    assert sent.name == ARM_JOINTS
    assert sent.position[0] == step
    # The other joints are held where they were read.
    assert sent.position[1:] == AT_REST[1:]


def test_nothing_is_sent_and_the_holds_drop_while_the_targets_are_stale(
    manual: Callable[..., Rig],
) -> None:
    rig = manual()
    rig.report(*AT_REST)
    rig.tick()
    assert rig.commands() == []

    rig.target(joint1=0.2)
    rig.report(*AT_REST)
    rig.tick()
    assert len(rig.commands()) == 1
    time.sleep(0.15)
    rig.report(0.15, *AT_REST[1:])
    rig.tick()
    rig.report(0.15, *AT_REST[1:])
    rig.tick()
    assert len(rig.commands()) == 1
    task = rig.coordinator.get_task(CALIBRATION_PASSTHROUGH_TASK)
    assert task is not None and task.get_status().stale  # type: ignore[attr-defined]

    # The next target goes out once a reading after the drop came in.
    rig.target(joint1=0.3)
    rig.tick()
    rig.report(0.15, *AT_REST[1:])
    rig.tick()
    assert rig.commands()[-1].position[:2] == [0.3, 0.0]


def test_hold_over_task_invoke_stops_the_commands_at_once(manual: Callable[..., Rig]) -> None:
    rig = manual()
    rig.report(*AT_REST)
    rig.target(joint1=0.2)
    rig.tick()
    sent = len(rig.commands())

    status = rig.coordinator.task_invoke(CALIBRATION_PASSTHROUGH_TASK, "hold")

    assert status.stale
    rig.report(*AT_REST)
    rig.tick()
    assert len(rig.commands()) == sent


@pytest.fixture
def running(mocker: Any) -> Iterator[Callable[..., tuple[Rig, FakeArm]]]:
    """A running calibration coordinator driving an in-process ``FakeArm`` named "arm"."""
    mocker.patch.object(connection_module, "CONFIG_PARK_GRACE_S", 0.1)
    made: list[tuple[Rig, FakeArm]] = []

    def build(**arm_config: Any) -> tuple[Rig, FakeArm]:
        rig = Rig()
        arm = FakeArm(source="arm", config_sessions=True, **arm_config)
        arm.set_transport("position_command", rig.ports["arm_position_command"])  # type: ignore[arg-type]
        arm.set_transport("joint_state", rig.ports["arm_joint_state"])  # type: ignore[arg-type]
        arm.start()
        rig.start()
        made.append((rig, arm))
        return rig, arm

    yield build
    for rig, arm in made:
        loop = rig.coordinator._tick_loop
        if loop is not None and loop.is_running:
            rig.coordinator.stop()
        arm.stop()


def _stream(rig: Rig, seconds: float, joint1: float = 0.2) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rig.target(joint1=joint1)
        time.sleep(0.02)


def test_a_sender_that_dies_lets_the_deadman_halt_the_arm_and_end_the_session(
    running: Callable[..., tuple[Rig, FakeArm]],
) -> None:
    rig, arm = running(deadman_timeout_s=0.5)
    session = arm.config_set(ConfigPatch(target="arm"), lease_s=5.0).session_id
    _stream(rig, 1.0)
    assert arm.status().config_session_id == session and arm.halts == 0
    assert arm.writes[-1]["arm/joint1/position"] == 0.2

    died = time.monotonic()
    wait_until(lambda: arm.status().deadman_fired, timeout=2.0, interval=0.005)

    assert time.monotonic() - died < 1.0
    assert arm.status().config_session_id is None and arm.halts == 1
    assert arm._config_ended == "the connection halted"


def test_stopping_waits_while_targets_come_then_stops(
    running: Callable[..., tuple[Rig, FakeArm]],
) -> None:
    rig, arm = running(deadman_timeout_s=10.0)
    parking = threading.Thread(target=_stream, args=(rig, 0.6))
    parking.start()
    time.sleep(0.1)
    began = time.monotonic()

    rig.coordinator.stop()

    took = time.monotonic() - began
    parking.join()
    assert 0.5 - 0.05 <= took < 0.5 + 0.1 + 0.3
    assert rig.coordinator._tick_loop is not None and not rig.coordinator._tick_loop.is_running


def test_stopping_gives_up_after_the_park_wait(
    running: Callable[..., tuple[Rig, FakeArm]], mocker: Any
) -> None:
    mocker.patch.object(connection_module, "CONFIG_PARK_WAIT_S", 0.3)
    rig, _ = running(deadman_timeout_s=10.0)
    parking = threading.Thread(target=_stream, args=(rig, 1.5))
    parking.start()
    time.sleep(0.1)
    began = time.monotonic()

    rig.coordinator.stop()

    assert time.monotonic() - began < 0.6
    parking.join()


def test_stopping_without_targets_waits_only_the_grace(
    running: Callable[..., tuple[Rig, FakeArm]],
) -> None:
    rig, _ = running()
    began = time.monotonic()
    rig.coordinator.stop()
    assert time.monotonic() - began < 0.4

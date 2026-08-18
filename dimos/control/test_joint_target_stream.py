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

"""Tests for the opt-in coordinator_joint_target stream."""

from __future__ import annotations

from collections.abc import Callable, Iterator
import threading
from typing import Any

import pytest

from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.JointState import JointState

ARM_JOINTS = make_joints("arm", 2)
MEASURED_POSITIONS = [0.11, 0.12]
COMMANDED_POSITIONS = [0.51, 0.62]


class OutTap:
    def __init__(self, port: Out) -> None:
        self._lock = threading.Lock()
        self._messages: list[JointState] = []
        port.subscribe(self._on_message)

    def _on_message(self, msg: JointState) -> None:
        with self._lock:
            self._messages.append(msg)

    @property
    def count(self) -> int:
        with self._lock:
            return len(self._messages)

    def latest(self) -> JointState:
        with self._lock:
            assert self._messages, "port never published"
            return self._messages[-1]


def _arm_component() -> HardwareComponent:
    return HardwareComponent(
        hardware_id="arm",
        hardware_type=HardwareType.MANIPULATOR,
        joints=ARM_JOINTS,
        adapter_type="mock",
        adapter_kwargs={"initial_positions": MEASURED_POSITIONS},
    )


def _servo_task() -> TaskConfig:
    return TaskConfig(name="servo_arm", type="servo", joint_names=ARM_JOINTS)


@pytest.fixture
def make_coordinator() -> Iterator[Callable[..., ControlCoordinator]]:
    built: list[ControlCoordinator] = []

    def make(**kwargs: Any) -> ControlCoordinator:
        coordinator = ControlCoordinator(**kwargs)
        built.append(coordinator)
        return coordinator

    try:
        yield make
    finally:
        for coordinator in built:
            coordinator.stop()


def _command() -> JointState:
    return JointState({"name": list(ARM_JOINTS), "position": list(COMMANDED_POSITIONS)})


class TestJointTargetStream:
    def test_default_is_off_and_port_stays_silent(self, make_coordinator, wait_until):
        coordinator = make_coordinator(hardware=[_arm_component()], tasks=[_servo_task()])
        state = OutTap(coordinator.coordinator_joint_state)
        target = OutTap(coordinator.coordinator_joint_target)
        coordinator.start()
        coordinator._dispatch("joint_command", _command())

        wait_until(lambda: state.count > 2, timeout=5.0)
        assert coordinator.config.publish_joint_targets is False
        assert target.count == 0

    def test_publishes_commanded_targets_not_measured_state(self, make_coordinator, wait_until):
        coordinator = make_coordinator(
            hardware=[_arm_component()],
            tasks=[_servo_task()],
            publish_joint_targets=True,
        )
        target = OutTap(coordinator.coordinator_joint_target)
        coordinator.start()
        coordinator._dispatch("joint_command", _command())

        wait_until(lambda: target.count > 0, timeout=5.0)
        msg = target.latest()
        assert sorted(msg.name) == sorted(ARM_JOINTS)
        by_name = dict(zip(msg.name, msg.position, strict=True))
        assert [by_name[name] for name in ARM_JOINTS] == pytest.approx(COMMANDED_POSITIONS)

    def test_idle_tasks_publish_no_targets(self, make_coordinator, wait_until):
        coordinator = make_coordinator(
            hardware=[_arm_component()],
            tasks=[_servo_task()],
            publish_joint_targets=True,
        )
        state = OutTap(coordinator.coordinator_joint_state)
        target = OutTap(coordinator.coordinator_joint_target)
        coordinator.start()

        wait_until(lambda: state.count > 2, timeout=5.0)
        assert target.count == 0

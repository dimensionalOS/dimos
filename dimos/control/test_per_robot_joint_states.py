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

"""Tests for opt-in per-robot JointState output streams."""

from __future__ import annotations

from collections.abc import Callable, Iterator
import threading
from typing import Any
from unittest.mock import MagicMock

import pytest

from dimos.control.components import HardwareComponent, HardwareType, make_joints
import dimos.control.coordinator as coord_mod
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.hardware_interface import ConnectedHardware
from dimos.control.tick_loop import TickLoop
from dimos.core.stream import In, Out
from dimos.hardware.manipulators.registry import adapter_registry
from dimos.hardware.manipulators.spec import ManipulatorAdapter
from dimos.msgs.sensor_msgs.JointState import JointState

LEFT_JOINTS = make_joints("left_arm", 2)
RIGHT_JOINTS = make_joints("right_arm", 3)
LEFT_POSITIONS = [0.11, 0.12]
RIGHT_POSITIONS = [0.21, 0.22, 0.23]


class DualArmCoordinator(ControlCoordinator):
    left_arm_joints: Out[JointState]
    right_arm_joints: Out[JointState]


class LeftOnlyCoordinator(ControlCoordinator):
    left_arm_joints: Out[JointState]


class TeleopStyleCoordinator(DualArmCoordinator):
    left_arm_command: In[JointState]


def _component(hardware_id: str, joints: list[str], positions: list[float]) -> HardwareComponent:
    return HardwareComponent(
        hardware_id=hardware_id,
        hardware_type=HardwareType.MANIPULATOR,
        joints=joints,
        adapter_type="mock",
        adapter_kwargs={"initial_positions": positions},
    )


def _left() -> HardwareComponent:
    return _component("left_arm", LEFT_JOINTS, LEFT_POSITIONS)


def _right() -> HardwareComponent:
    return _component("right_arm", RIGHT_JOINTS, RIGHT_POSITIONS)


class OutTap:
    def __init__(self, port: Out) -> None:
        self._messages: list[JointState] = []
        self._lock = threading.Lock()
        port.subscribe(self._record)

    def _record(self, msg: JointState) -> None:
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


class InTap:
    def __init__(self, mocker: Any, port: In) -> None:
        self._callbacks: list[Callable[[Any], None]] = []
        mocker.patch.object(port, "subscribe", side_effect=self._subscribe)

    def _subscribe(self, cb: Callable[[Any], None]) -> Callable[[], None]:
        self._callbacks.append(cb)
        return lambda: self._callbacks.remove(cb)

    def emit(self, msg: Any) -> None:
        assert self._callbacks, "port was never subscribed"
        for cb in list(self._callbacks):
            cb(msg)


@pytest.fixture
def make_coordinator() -> Iterator[Callable[..., ControlCoordinator]]:
    built: list[ControlCoordinator] = []

    def make(
        coordinator_cls: type[ControlCoordinator] = ControlCoordinator, **kwargs: Any
    ) -> ControlCoordinator:
        coordinator = coordinator_cls(**kwargs)
        built.append(coordinator)
        return coordinator

    try:
        yield make
    finally:
        for coordinator in built:
            coordinator.stop()


def _mock_hardware(
    hardware_id: str,
    joints: list[str],
    positions: list[float],
    velocities: list[float],
    efforts: list[float],
) -> ConnectedHardware:
    adapter = MagicMock(spec=ManipulatorAdapter)
    adapter.read_joint_positions.return_value = positions
    adapter.read_joint_velocities.return_value = velocities
    adapter.read_joint_efforts.return_value = efforts
    component = HardwareComponent(
        hardware_id=hardware_id,
        hardware_type=HardwareType.MANIPULATOR,
        joints=joints,
    )
    return ConnectedHardware(adapter, component)


class TestFlagOff:
    def test_default_is_off(self, make_coordinator):
        coordinator = make_coordinator(publish_joint_state=False)

        assert coordinator.config.publish_robot_joint_states is False

    def test_declared_ports_stay_silent_while_the_blob_publishes(
        self, make_coordinator, wait_until
    ):
        coordinator = make_coordinator(
            coordinator_cls=DualArmCoordinator, hardware=[_left(), _right()]
        )
        merged = OutTap(coordinator.coordinator_joint_state)
        left = OutTap(coordinator.left_arm_joints)
        right = OutTap(coordinator.right_arm_joints)
        coordinator.start()

        wait_until(lambda: merged.count > 2, timeout=5.0)
        assert left.count == 0
        assert right.count == 0

    def test_plain_coordinator_needs_no_per_robot_ports(self, make_coordinator):
        coordinator = make_coordinator(hardware=[_left(), _right()])
        coordinator.start()

        assert sorted(coordinator.list_hardware()) == ["left_arm", "right_arm"]
        assert sorted(coordinator.outputs) == ["coordinator_joint_state"]


class TestPerRobotPublishing:
    def test_each_port_gets_only_its_own_robot(self, make_coordinator, wait_until):
        coordinator = make_coordinator(
            coordinator_cls=DualArmCoordinator,
            publish_robot_joint_states=True,
            hardware=[_left(), _right()],
        )
        merged = OutTap(coordinator.coordinator_joint_state)
        left = OutTap(coordinator.left_arm_joints)
        right = OutTap(coordinator.right_arm_joints)
        coordinator.start()

        wait_until(lambda: bool(left.count and right.count and merged.count), timeout=5.0)

        left_msg = left.latest()
        assert list(left_msg.name) == LEFT_JOINTS
        assert left_msg.frame_id == "left_arm"
        assert list(left_msg.position) == LEFT_POSITIONS

        right_msg = right.latest()
        assert list(right_msg.name) == RIGHT_JOINTS
        assert right_msg.frame_id == "right_arm"
        assert list(right_msg.position) == RIGHT_POSITIONS

        assert set(merged.latest().name) == {*LEFT_JOINTS, *RIGHT_JOINTS}

    def test_runtime_added_hardware_publishes_on_its_port(self, make_coordinator, wait_until):
        coordinator = make_coordinator(
            coordinator_cls=DualArmCoordinator,
            publish_robot_joint_states=True,
            hardware=[_left()],
        )
        right = OutTap(coordinator.right_arm_joints)
        coordinator.start()
        assert right.count == 0

        component = _right()
        adapter = adapter_registry.create(
            "mock",
            dof=len(RIGHT_JOINTS),
            hardware_id="right_arm",
            initial_positions=RIGHT_POSITIONS,
        )
        assert adapter.connect()
        assert coordinator.add_hardware(adapter, component)

        wait_until(lambda: right.count > 0, timeout=5.0)
        assert list(right.latest().position) == RIGHT_POSITIONS


class TestTickLoopMessages:
    @staticmethod
    def _tick_once(publish_robot_callback):
        merged: list[JointState] = []
        left = _mock_hardware("left_arm", LEFT_JOINTS, [0.1, 0.2], [1.1, 1.2], [2.1, 2.2])
        right = _mock_hardware(
            "right_arm", RIGHT_JOINTS, [0.3, 0.4, 0.5], [1.3, 1.4, 1.5], [2.3, 2.4, 2.5]
        )
        tick_loop = TickLoop(
            tick_rate=100.0,
            hardware={"left_arm": left, "right_arm": right},
            hardware_lock=threading.Lock(),
            tasks={},
            task_lock=threading.Lock(),
            joint_to_hardware={},
            publish_callback=merged.append,
            publish_robot_callback=publish_robot_callback,
        )
        tick_loop._tick()
        return merged, left, right

    def test_message_carries_only_that_robots_values(self):
        published: list[tuple[str, JointState]] = []
        merged, _, _ = self._tick_once(lambda hw_id, msg: published.append((hw_id, msg)))

        by_id = dict(published)
        assert sorted(by_id) == ["left_arm", "right_arm"]

        left_msg = by_id["left_arm"]
        assert list(left_msg.name) == LEFT_JOINTS
        assert list(left_msg.position) == [0.1, 0.2]
        assert list(left_msg.velocity) == [1.1, 1.2]
        assert list(left_msg.effort) == [2.1, 2.2]
        assert left_msg.frame_id == "left_arm"

        right_msg = by_id["right_arm"]
        assert list(right_msg.position) == [0.3, 0.4, 0.5]
        assert list(right_msg.velocity) == [1.3, 1.4, 1.5]
        assert list(right_msg.effort) == [2.3, 2.4, 2.5]
        assert right_msg.frame_id == "right_arm"

        assert left_msg.ts == merged[0].ts
        assert right_msg.ts == merged[0].ts

    def test_hardware_is_read_once_per_tick(self):
        published: list[tuple[str, JointState]] = []
        _, left, _ = self._tick_once(lambda hw_id, msg: published.append((hw_id, msg)))

        assert left.adapter.read_joint_positions.call_count == 1
        assert left.adapter.read_joint_velocities.call_count == 1
        assert left.adapter.read_joint_efforts.call_count == 1

    def test_one_failing_port_does_not_starve_the_others(self):
        published: list[str] = []

        def publish(hw_id: str, msg: JointState) -> None:
            if hw_id == "left_arm":
                raise RuntimeError("transport down")
            published.append(hw_id)

        self._tick_once(publish)

        assert published == ["right_arm"]


class TestMissingPortIsLoud:
    def test_startup_names_the_hardware_and_the_line_to_add(self, make_coordinator, mocker):
        coordinator = make_coordinator(
            coordinator_cls=LeftOnlyCoordinator,
            publish_robot_joint_states=True,
            hardware=[_left(), _right()],
        )
        create = mocker.patch.object(adapter_registry, "create", wraps=adapter_registry.create)

        with pytest.raises(ValueError) as excinfo:
            coordinator.start()

        message = str(excinfo.value)
        assert "right_arm" in message
        assert "right_arm_joints: Out[JointState]" in message
        assert coordinator.list_hardware() == []
        assert not create.called

    def test_hardware_id_that_cannot_name_a_port_says_so(self, make_coordinator):
        coordinator = make_coordinator(
            coordinator_cls=LeftOnlyCoordinator,
            publish_robot_joint_states=True,
            hardware=[_component("left-arm", ["left-arm/joint1"], [0.0])],
        )

        with pytest.raises(ValueError) as excinfo:
            coordinator.start()

        message = str(excinfo.value)
        assert "left-arm" in message
        assert "rename" in message

    def test_runtime_add_hardware_names_the_line_to_add(self, make_coordinator):
        coordinator = make_coordinator(
            coordinator_cls=LeftOnlyCoordinator,
            publish_robot_joint_states=True,
            hardware=[_left()],
        )
        coordinator.start()

        adapter = adapter_registry.create("mock", dof=len(RIGHT_JOINTS), hardware_id="right_arm")
        assert adapter.connect()

        with pytest.raises(ValueError) as excinfo:
            coordinator.add_hardware(adapter, _right())

        message = str(excinfo.value)
        assert "right_arm" in message
        assert "right_arm_joints: Out[JointState]" in message
        assert coordinator.list_hardware() == ["left_arm"]


class TestPostPivotPattern:
    def test_subclass_serves_its_own_input_and_outputs_together(
        self, make_coordinator, mocker, wait_until
    ):
        coordinator = make_coordinator(
            coordinator_cls=TeleopStyleCoordinator,
            instance_name="ControlCoordinator",
            publish_robot_joint_states=True,
            hardware=[_left(), _right()],
            tasks=[
                TaskConfig(
                    name="servo_left",
                    type="servo",
                    joint_names=LEFT_JOINTS,
                    stream_bind={"joint_command": "left_arm_command"},
                )
            ],
        )
        commands = InTap(mocker, coordinator.left_arm_command)
        left = OutTap(coordinator.left_arm_joints)
        right = OutTap(coordinator.right_arm_joints)
        coordinator.start()

        commands.emit(JointState(name=LEFT_JOINTS, position=[0.4, 0.5]))

        assert coordinator.get_task("servo_left")._target == [0.4, 0.5]
        wait_until(lambda: bool(left.count and right.count), timeout=5.0)
        assert list(left.latest().name) == LEFT_JOINTS
        assert list(right.latest().name) == RIGHT_JOINTS


class TestInstanceNameGuard:
    def test_subclass_without_instance_name_warns(self, make_coordinator, mocker):
        warn = mocker.patch.object(coord_mod.logger, "warning")
        make_coordinator(coordinator_cls=DualArmCoordinator).start()

        logged = " ".join(str(call) for call in warn.call_args_list)
        assert "instance_name" in logged
        assert "DualArmCoordinator" in logged

    def test_subclass_with_instance_name_does_not_warn(self, make_coordinator, mocker):
        warn = mocker.patch.object(coord_mod.logger, "warning")
        make_coordinator(
            coordinator_cls=DualArmCoordinator, instance_name="ControlCoordinator"
        ).start()

        assert not any("instance_name" in str(call) for call in warn.call_args_list)

    def test_plain_coordinator_does_not_warn(self, make_coordinator, mocker):
        warn = mocker.patch.object(coord_mod.logger, "warning")
        make_coordinator().start()

        assert not any("instance_name" in str(call) for call in warn.call_args_list)

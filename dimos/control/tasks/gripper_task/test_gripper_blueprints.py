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

"""Gripper task wiring tests: blueprints and the keyboard-to-adapter path."""

from __future__ import annotations

from typing import Any, cast

import pytest

from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.hardware_interface import ConnectedHardware
from dimos.control.tasks.gripper_task.gripper_task import create_task
from dimos.core.coordination.blueprints import Blueprint
from dimos.hardware.manipulators.mock.adapter import MockAdapter
from dimos.hardware.manipulators.spec import JointLimits
from dimos.msgs.std_msgs.Bool import Bool

_KEYBOARD_BLUEPRINTS = ["a1z", "piper"]


def _coordinator_kwargs(blueprint: Blueprint) -> dict[str, Any]:
    return next(atom.kwargs for atom in blueprint.blueprints if atom.module is ControlCoordinator)


def _load(name: str) -> Blueprint:
    if name == "a1z":
        from dimos.robot.manipulators.a1z.blueprints.teleop import keyboard_teleop_a1z

        return keyboard_teleop_a1z
    from dimos.robot.manipulators.piper.blueprints.teleop import keyboard_teleop_piper

    return keyboard_teleop_piper


class TestBothKeyboardBlueprintsMigrated:
    @pytest.mark.parametrize("name", _KEYBOARD_BLUEPRINTS)
    def test_carries_a_gripper_task_claiming_the_gripper_joints(self, name: str) -> None:
        kwargs = _coordinator_kwargs(_load(name))
        hardware = kwargs["hardware"][0]
        tasks = cast("list[TaskConfig]", kwargs["tasks"])

        gripper = [t for t in tasks if t.type == "gripper"]
        assert len(gripper) == 1, f"{name}: expected exactly one gripper task"
        assert gripper[0].name == "arm_gripper"
        assert gripper[0].joint_names == ["arm/gripper"]
        assert set(gripper[0].joint_names) <= set(hardware.joints)

    @pytest.mark.parametrize("name", _KEYBOARD_BLUEPRINTS)
    def test_no_servo_task_claims_a_gripper_joint(self, name: str) -> None:
        kwargs = _coordinator_kwargs(_load(name))
        gripper_joints = {"arm/gripper"}

        for task in cast("list[TaskConfig]", kwargs["tasks"]):
            if task.type == "gripper":
                continue
            assert not (set(task.joint_names) & gripper_joints), (
                f"{name}: task {task.name!r} ({task.type}) also claims "
                f"{gripper_joints & set(task.joint_names)} — R17 says one owner"
            )

    @pytest.mark.parametrize("name", _KEYBOARD_BLUEPRINTS)
    def test_gripper_joint_has_exactly_one_claimant(self, name: str) -> None:
        kwargs = _coordinator_kwargs(_load(name))
        for joint in ["arm/gripper"]:
            claimants = [t.name for t in kwargs["tasks"] if joint in t.joint_names]
            assert claimants == ["arm_gripper"], f"{name}: {joint} claimed by {claimants}"


class TestKeyboardReachesTheAdapter:
    """Keyboard open/close drives a real adapter through the real wiring."""

    @pytest.fixture
    def wired(self) -> Any:
        component = HardwareComponent(
            hardware_id="arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=[*make_joints("arm", 6), "arm/gripper"],
            adapter_type="mock",
        )
        adapter = MockAdapter(
            dof=7,
            limits=JointLimits(
                position_lower=[-3.14] * 6 + [0.0],
                position_upper=[3.14] * 6 + [0.08],
                velocity_max=[1.0] * 7,
            ),
        )
        adapter.connect()
        hardware = ConnectedHardware(adapter, component)

        task = create_task(
            TaskConfig(
                name="arm_gripper",
                type="gripper",
                joint_names=["arm/gripper"],
                priority=20,
            ),
            {"arm": hardware},
        )
        return task, hardware, adapter

    def test_open_and_close_reach_the_adapter_in_native_units(self, wired: Any) -> None:
        task, hardware, _ = wired

        # `]` — closed. The keyboard sends a wish; the task converts once.
        task.on_gripper_command(Bool(data=True), 0.0)
        assert task.compute(_snapshot(hardware)).positions == [0.0]

        # `[` — open. 0.08 m is the declared stroke, not a 0-1 fraction.
        task.on_gripper_command(Bool(data=False), 0.0)
        assert task.compute(_snapshot(hardware)).positions == [pytest.approx(0.08)]

    def test_command_travels_the_one_array_to_the_adapter(self, wired: Any) -> None:
        task, hardware, adapter = wired

        task.on_gripper_command(Bool(data=False), 0.0)
        output = task.compute(_snapshot(hardware))
        hardware.write_command(
            dict(zip(output.joint_names, output.positions, strict=True)), output.mode
        )

        positions = adapter.read_joint_positions()
        assert len(positions) == 7, "one array covering arm and gripper"
        assert positions[-1] == pytest.approx(0.08)

    def test_get_position_agrees_with_the_published_joint_state(self, wired: Any) -> None:
        task, hardware, adapter = wired
        adapter.write_joint_positions([0.0] * 6 + [0.042])

        state = _snapshot(hardware)
        task.compute(state)

        assert task.get_position() == [pytest.approx(0.042)]
        assert state.joints.get_position("arm/gripper") == pytest.approx(0.042)


def _snapshot(hardware: ConnectedHardware) -> Any:
    """One tick's worth of measured state, as the tick loop builds it."""
    from dimos.control.task import CoordinatorState, JointStateSnapshot

    positions = {n: js.position for n, js in hardware.read_state().items()}
    return CoordinatorState(joints=JointStateSnapshot(joint_positions=positions), t_now=0.0)

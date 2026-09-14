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

from __future__ import annotations

from collections.abc import Iterator
import importlib
import math
import sys
from types import ModuleType
from typing import Any, ClassVar

import pytest

from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.control.hardware_interface import ConnectedHardware
from dimos.hardware.manipulators.spec import ControlMode


class _FakeXArmSdk:
    instances: ClassVar[list[_FakeXArmSdk]] = []

    def __init__(self, ip: str) -> None:
        self.instances.append(self)
        self.ip = ip
        self.connected = False
        self.warn_code = 1
        self.error_code = 2
        self.state = 4
        self.mode = 0
        self.actions: list[Any] = []
        self.servo_joint_commands: list[list[float]] = []
        self.gripper_commands: list[float] = []

    def connect(self) -> None:
        self.connected = True
        self.actions.append(("connect", self.ip))

    def set_mode(self, mode: int) -> int:
        self.mode = mode
        self.actions.append(("set_mode", mode))
        return 0

    def set_state(self, state: int) -> int:
        self.state = state
        self.actions.append(("set_state", state))
        return 0

    def clean_warn(self) -> int:
        self.warn_code = 0
        self.actions.append(("clean_warn", None))
        return 0

    def clean_error(self) -> int:
        self.error_code = 0
        self.actions.append(("clean_error", None))
        return 0

    def motion_enable(self, *, enable: bool) -> int:
        self.actions.append(("motion_enable", enable))
        return 0

    def set_servo_angle(
        self,
        *,
        angle: list[float],
        speed: float,
        mvacc: float,
        wait: bool,
    ) -> int:
        self.actions.append(("set_servo_angle", list(angle), speed, mvacc, wait))
        return 0

    def set_servo_angle_j(self, angles: list[float], *, speed: float, mvacc: float) -> int:
        self.servo_joint_commands.append(list(angles))
        self.actions.append(("set_servo_angle_j", list(angles), speed, mvacc))
        return 0

    def get_servo_angle(self) -> tuple[int, list[float]]:
        return 0, [0.0] * 7

    def get_gripper_position(self) -> tuple[int, float]:
        return 0, 0.0

    def set_gripper_enable(self, enabled: bool) -> int:
        self.actions.append(("set_gripper_enable", enabled))
        return 0

    def set_gripper_position(self, position: float, *, wait: bool) -> int:
        self.gripper_commands.append(position)
        self.actions.append(("set_gripper_position", position, wait))
        return 0


@pytest.fixture
def xarm_adapter_module(monkeypatch: pytest.MonkeyPatch) -> Iterator[ModuleType]:
    _FakeXArmSdk.instances.clear()
    xarm_pkg = ModuleType("xarm")
    wrapper = ModuleType("xarm.wrapper")
    wrapper.XArmAPI = _FakeXArmSdk  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "xarm", xarm_pkg)
    monkeypatch.setitem(sys.modules, "xarm.wrapper", wrapper)
    sys.modules.pop("dimos.hardware.manipulators.xarm.adapter", None)
    yield importlib.import_module("dimos.hardware.manipulators.xarm.adapter")
    sys.modules.pop("dimos.hardware.manipulators.xarm.adapter", None)


def test_activate_prepares_xarm_for_safe_commanded_motion(
    xarm_adapter_module: ModuleType,
) -> None:
    adapter = xarm_adapter_module.XArmAdapter(address="192.0.2.10", dof=6)
    assert adapter.connect()

    assert adapter.activate()

    arm = _FakeXArmSdk.instances[-1]
    assert arm.warn_code == 0
    assert arm.error_code == 0
    # Bringing a blueprint up must not command the arm anywhere.
    assert arm.actions[-7:] == [
        ("clean_warn", None),
        ("clean_error", None),
        ("motion_enable", True),
        ("set_mode", 0),
        ("set_state", 0),
        ("set_mode", 1),
        ("set_state", 0),
    ]
    assert not any(action[0] == "set_servo_angle" for action in arm.actions)
    assert adapter.get_control_mode() == xarm_adapter_module.ControlMode.SERVO_POSITION


def test_activate_moves_only_to_an_explicitly_configured_initial_pose(
    xarm_adapter_module: ModuleType,
) -> None:
    initial = [0.0, -math.pi / 4, 0.0, 0.0, math.pi / 2, 0.0]
    adapter = xarm_adapter_module.XArmAdapter(
        address="192.0.2.10", dof=6, initial_positions=initial
    )
    assert adapter.connect()

    assert adapter.activate()

    arm = _FakeXArmSdk.instances[-1]
    assert ("set_servo_angle", [0.0, -45.0, 0.0, 0.0, 90.0, 0.0], 20.0, 500.0, True) in arm.actions


def test_initial_positions_must_match_the_arm_axis_count(
    xarm_adapter_module: ModuleType,
) -> None:
    with pytest.raises(ValueError):
        xarm_adapter_module.XArmAdapter(address="192.0.2.10", dof=6, initial_positions=[0.0])


def test_joint_position_commands_use_degrees_for_xarm_sdk(
    xarm_adapter_module: ModuleType,
) -> None:
    adapter = xarm_adapter_module.XArmAdapter(address="192.0.2.10", dof=6)
    assert adapter.connect()

    assert adapter.write_joint_positions([math.pi / 2, -math.pi / 4, math.pi, 0.0, 0.0, 0.0])

    arm = _FakeXArmSdk.instances[-1]
    assert arm.servo_joint_commands[-1] == pytest.approx([90.0, -45.0, 180.0, 0.0, 0.0, 0.0])


def test_gripper_command_reaches_sdk_once_in_native_units(
    xarm_adapter_module: ModuleType,
) -> None:
    adapter = xarm_adapter_module.XArmAdapter(
        address="192.0.2.10",
        dof=8,
        arm_dof=7,
    )
    assert adapter.connect()
    hardware = ConnectedHardware(
        adapter,
        HardwareComponent(
            hardware_id="arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=[*make_joints("arm", 7), "arm/gripper"],
        ),
    )

    assert hardware.write_command({"arm/gripper": 850.0}, ControlMode.SERVO_POSITION)

    sdk = _FakeXArmSdk.instances[-1]
    assert sdk.servo_joint_commands == [[0.0] * 7]
    assert sdk.gripper_commands == [850.0]

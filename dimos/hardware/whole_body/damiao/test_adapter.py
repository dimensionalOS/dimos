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

from typing import cast

import can_motor_control
import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.hardware.whole_body.damiao.adapter import DamiaoWholeBodyAdapter
from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.hardware.whole_body.spec import MotorCommand, MotorState


class FakeArm:
    def __init__(self, positions: list[float]) -> None:
        self._positions = np.asarray(positions, dtype=np.float64)
        self.modes: list[str] = []
        self.commands: list[np.ndarray] = []

    def positions(self) -> np.ndarray:
        return self._positions

    def velocities(self) -> np.ndarray:
        return np.zeros_like(self._positions)

    def torques(self) -> np.ndarray:
        return np.zeros_like(self._positions)

    def set_mode(self, mode: str) -> None:
        self.modes.append(mode)

    def mit_control(self, commands: np.ndarray) -> None:
        self.commands.append(commands)


class FakeGripper:
    def __init__(self, opening: float) -> None:
        self.opening = opening
        self.commands: list[float] = []

    def set_opening(self, opening: float) -> None:
        self.commands.append(opening)


class FakeRobot:
    def __init__(self, groups: dict[str, FakeArm | FakeGripper]) -> None:
        self.groups = groups
        self.connected = False
        self.enable_error: Exception | None = None
        self.enable_count = 0
        self.disable_count = 0
        self.refresh_count = 0
        self.tick_count = 0

    def __getitem__(self, name: str) -> FakeArm | FakeGripper:
        return self.groups[name]

    def connect(self) -> None:
        self.connected = True

    def enable(self) -> None:
        self.enable_count += 1
        if self.enable_error is not None:
            raise self.enable_error

    def disable(self) -> None:
        self.disable_count += 1

    def refresh(self) -> None:
        self.refresh_count += 1

    def tick(self, _deadline: int) -> None:
        self.tick_count += 1

    def is_connected(self) -> bool:
        return self.connected


class DualAdapter(DamiaoWholeBodyAdapter):
    arm_joints = {
        "left_arm": ("left_arm/joint1", "left_arm/joint2"),
        "right_arm": ("right_arm/joint1", "right_arm/joint2"),
    }
    gripper_joints = {
        "left_gripper": "left_arm/gripper",
        "right_gripper": "right_arm/gripper",
    }
    bus_defaults = {"left": "can0", "right": "can1"}

    def __init__(self, robot: FakeRobot, **kwargs: object) -> None:
        self.fake_robot = robot
        super().__init__(**kwargs)

    def _build_robot(self) -> can_motor_control.Robot:
        return cast("can_motor_control.Robot", self.fake_robot)


@pytest.fixture
def dual_robot() -> FakeRobot:
    return FakeRobot(
        {
            "left_arm": FakeArm([0.1, 0.2]),
            "right_arm": FakeArm([0.3, 0.4]),
            "left_gripper": FakeGripper(0.5),
            "right_gripper": FakeGripper(0.6),
        }
    )


@pytest.fixture
def dual_adapter(dual_robot: FakeRobot, mocker: MockerFixture) -> DualAdapter:
    mocker.patch.object(
        DualAdapter,
        "_require_arm",
        side_effect=lambda robot, name: robot[name],
    )
    mocker.patch.object(
        DualAdapter,
        "_require_gripper",
        side_effect=lambda robot, name: robot[name],
    )
    adapter = DualAdapter(
        dual_robot,
        runtime_config=DamiaoRuntimeConfig(gravity_comp=False),
        dof=6,
    )
    assert adapter.connect()
    assert adapter.activate()
    return adapter


def test_dual_arm_state_includes_both_normalized_grippers(
    dual_adapter: DualAdapter,
    dual_robot: FakeRobot,
) -> None:
    ticks_before = dual_robot.tick_count

    assert dual_adapter.joint_names == (
        "left_arm/joint1",
        "left_arm/joint2",
        "right_arm/joint1",
        "right_arm/joint2",
        "left_arm/gripper",
        "right_arm/gripper",
    )
    assert dual_adapter.read_motor_states() == [
        MotorState(q=0.1),
        MotorState(q=0.2),
        MotorState(q=0.3),
        MotorState(q=0.4),
        MotorState(q=0.5),
        MotorState(q=0.6),
    ]
    assert dual_robot.tick_count == ticks_before


def test_combined_command_ticks_once_and_splits_groups(
    dual_adapter: DualAdapter,
    dual_robot: FakeRobot,
) -> None:
    ticks_before = dual_robot.tick_count

    assert dual_adapter.write_motor_commands(
        [
            MotorCommand(q=1.0, kp=10.0),
            MotorCommand(q=1.1, kp=11.0),
            MotorCommand(q=2.0, kp=20.0),
            MotorCommand(q=2.1, kp=21.0),
            MotorCommand(q=0.25),
            MotorCommand(q=0.75),
        ]
    )

    assert dual_robot.tick_count == ticks_before + 1
    left_arm = cast("FakeArm", dual_robot["left_arm"])
    right_arm = cast("FakeArm", dual_robot["right_arm"])
    assert left_arm.commands[-1].tolist() == [
        [10.0, 0.0, 1.0, 16000.0, 0.0],
        [11.0, 0.0, 1.1, 16000.0, 0.0],
    ]
    assert right_arm.commands[-1].tolist() == [
        [20.0, 0.0, 2.0, 16000.0, 0.0],
        [21.0, 0.0, 2.1, 16000.0, 0.0],
    ]
    assert cast("FakeGripper", dual_robot["left_gripper"]).commands == [0.25]
    assert cast("FakeGripper", dual_robot["right_gripper"]).commands == [0.75]


def test_gripper_command_rejects_out_of_range(
    dual_adapter: DualAdapter,
    dual_robot: FakeRobot,
) -> None:
    commands = [MotorCommand(q=0.0)] * 4 + [MotorCommand(q=-0.1), MotorCommand(q=0.5)]

    assert not dual_adapter.write_motor_commands(commands)
    assert cast("FakeGripper", dual_robot["left_gripper"]).commands == []


def test_gravity_is_added_to_commanded_residual_torque(
    dual_adapter: DualAdapter,
    dual_robot: FakeRobot,
    mocker: MockerFixture,
) -> None:
    mocker.patch.object(
        dual_adapter,
        "_gravity_torques",
        return_value=np.asarray([1.0, 2.0, 3.0, 4.0]),
    )

    commands = [MotorCommand(q=0.0, tau=0.5)] * 4 + [MotorCommand(q=0.5)] * 2
    assert dual_adapter.write_motor_commands(commands)

    assert cast("FakeArm", dual_robot["left_arm"]).commands[-1][:, 4].tolist() == [1.5, 2.5]
    assert cast("FakeArm", dual_robot["right_arm"]).commands[-1][:, 4].tolist() == [3.5, 4.5]


def test_activation_failure_disables_robot(
    dual_robot: FakeRobot,
    mocker: MockerFixture,
) -> None:
    mocker.patch.object(DualAdapter, "_require_arm", side_effect=lambda robot, name: robot[name])
    mocker.patch.object(
        DualAdapter,
        "_require_gripper",
        side_effect=lambda robot, name: robot[name],
    )
    adapter = DualAdapter(
        dual_robot,
        runtime_config=DamiaoRuntimeConfig(gravity_comp=False),
    )
    assert adapter.connect()
    dual_robot.enable_error = RuntimeError("calibration failed")

    assert not adapter.activate()
    assert dual_robot.disable_count == 1


def test_gripper_state_becomes_available_only_after_calibration(
    dual_robot: FakeRobot,
    mocker: MockerFixture,
) -> None:
    mocker.patch.object(DualAdapter, "_require_arm", side_effect=lambda robot, name: robot[name])
    mocker.patch.object(
        DualAdapter,
        "_require_gripper",
        side_effect=lambda robot, name: robot[name],
    )
    adapter = DualAdapter(
        dual_robot,
        runtime_config=DamiaoRuntimeConfig(gravity_comp=False),
    )

    assert not adapter.has_motor_states()
    assert adapter.connect()
    assert not adapter.has_motor_states()
    assert adapter.activate()
    assert adapter.has_motor_states()


def test_runtime_config_rejects_unknown_bus_override(dual_robot: FakeRobot) -> None:
    with pytest.raises(ValueError, match="unknown CAN bus"):
        DualAdapter(
            dual_robot,
            runtime_config=DamiaoRuntimeConfig(bus_addresses={"missing": "can9"}),
        )

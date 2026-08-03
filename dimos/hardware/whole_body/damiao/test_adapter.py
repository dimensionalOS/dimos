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

from collections.abc import Callable, Iterator
from pathlib import Path
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
        self.position_values = np.asarray(positions, dtype=np.float64)
        self.velocity_values = np.zeros_like(self.position_values)
        self.torque_values = np.zeros_like(self.position_values)
        self.mode_error: Exception | None = None
        self.command_error: Exception | None = None
        self.modes: list[str] = []
        self.commands: list[np.ndarray] = []

    def positions(self) -> np.ndarray:
        return self.position_values

    def velocities(self) -> np.ndarray:
        return self.velocity_values

    def torques(self) -> np.ndarray:
        return self.torque_values

    def set_mode(self, mode: str) -> None:
        if self.mode_error is not None:
            raise self.mode_error
        self.modes.append(mode)

    def mit_control(self, commands: np.ndarray) -> None:
        if self.command_error is not None:
            raise self.command_error
        self.commands.append(commands)


class FakeGripper:
    def __init__(self, opening: float) -> None:
        self.opening = opening
        self.command_error: Exception | None = None
        self.commands: list[float] = []

    def set_opening(self, opening: float) -> None:
        if self.command_error is not None:
            raise self.command_error
        self.commands.append(opening)


class FakeRobot:
    def __init__(self, groups: dict[str, FakeArm | FakeGripper]) -> None:
        self.groups = groups
        self.connected = False
        self.connect_error: Exception | None = None
        self.enable_error: Exception | None = None
        self.disable_error: Exception | None = None
        self.refresh_error: Exception | None = None
        self.tick_error: Exception | None = None
        self.enable_count = 0
        self.disable_count = 0
        self.refresh_count = 0
        self.tick_count = 0

    def __getitem__(self, name: str) -> FakeArm | FakeGripper:
        return self.groups[name]

    def connect(self) -> None:
        if self.connect_error is not None:
            raise self.connect_error
        self.connected = True

    def enable(self) -> None:
        self.enable_count += 1
        if self.enable_error is not None:
            raise self.enable_error

    def disable(self) -> None:
        self.disable_count += 1
        if self.disable_error is not None:
            raise self.disable_error

    def refresh(self) -> None:
        self.refresh_count += 1
        if self.refresh_error is not None:
            raise self.refresh_error

    def tick(self, _deadline: int) -> None:
        self.tick_count += 1
        if self.tick_error is not None:
            raise self.tick_error

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


class GravityDualAdapter(DualAdapter):
    gravity_joint_names = ("left1", "left2", "right1", "right2")

    def __init__(self, robot: FakeRobot, model_path: Path, **kwargs: object) -> None:
        self.model_path = model_path
        super().__init__(robot, **kwargs)

    @property
    def gravity_model_path(self) -> Path:
        return self.model_path


class FakePinModel:
    def __init__(
        self,
        *,
        nq: int = 4,
        nv: int = 4,
        names: tuple[str, ...] = ("universe", "left1", "left2", "right1", "right2"),
    ) -> None:
        self.nq = nq
        self.nv = nv
        self.names = names
        self.data = object()

    def createData(self) -> object:
        return self.data


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
def adapter_factory(mocker: MockerFixture) -> Callable[..., DualAdapter]:
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

    def create(robot: FakeRobot, **kwargs: object) -> DualAdapter:
        kwargs.setdefault("runtime_config", DamiaoRuntimeConfig(gravity_comp=False))
        return DualAdapter(robot, **kwargs)

    return create


@pytest.fixture
def active_dual_adapter(
    dual_robot: FakeRobot,
    adapter_factory: Callable[..., DualAdapter],
) -> Iterator[DualAdapter]:
    adapter = adapter_factory(dual_robot, dof=6)
    assert adapter.connect()
    assert adapter.activate()
    yield adapter
    adapter.disconnect()


def test_init_scalar_address_raises_named_bus_configuration_error(
    dual_robot: FakeRobot,
) -> None:
    with pytest.raises(ValueError, match="runtime_config.bus_addresses"):
        DualAdapter(dual_robot, address="can0")


def test_init_unknown_bus_override_raises_value_error(dual_robot: FakeRobot) -> None:
    with pytest.raises(ValueError, match="unknown CAN bus"):
        DualAdapter(
            dual_robot,
            runtime_config=DamiaoRuntimeConfig(bus_addresses={"missing": "can9"}),
        )


def test_init_mismatched_dof_raises_value_error(dual_robot: FakeRobot) -> None:
    with pytest.raises(ValueError, match="expected 6 joints, got 5"):
        DualAdapter(dual_robot, dof=5)


def test_init_duplicate_joint_mapping_raises_value_error(dual_robot: FakeRobot) -> None:
    class DuplicateJointAdapter(DualAdapter):
        arm_joints = {"left_arm": ("shared",), "right_arm": ("shared",)}
        gripper_joints = {}

    with pytest.raises(ValueError, match="duplicate names"):
        DuplicateJointAdapter(dual_robot)


def test_init_incomplete_gravity_mapping_raises_value_error(dual_robot: FakeRobot) -> None:
    class IncompleteGravityAdapter(DualAdapter):
        gravity_joint_names = ("left1",)

    with pytest.raises(ValueError, match="every angular arm joint"):
        IncompleteGravityAdapter(dual_robot)


def test_bus_address_runtime_override_returns_configured_interface(
    dual_robot: FakeRobot,
) -> None:
    adapter = DualAdapter(
        dual_robot,
        runtime_config=DamiaoRuntimeConfig(
            bus_addresses={"left": "can8"},
            gravity_comp=False,
        ),
    )

    assert adapter.bus_address("left") == "can8"


def test_bus_address_without_override_returns_declared_default(
    dual_robot: FakeRobot,
) -> None:
    adapter = DualAdapter(
        dual_robot,
        runtime_config=DamiaoRuntimeConfig(gravity_comp=False),
    )

    assert adapter.bus_address("right") == "can1"


def test_bus_address_undeclared_bus_raises_value_error(dual_robot: FakeRobot) -> None:
    adapter = DualAdapter(dual_robot)

    with pytest.raises(ValueError, match="did not declare CAN bus 'missing'"):
        adapter.bus_address("missing")


def test_connect_robot_build_failure_returns_false(
    dual_robot: FakeRobot,
    adapter_factory: Callable[..., DualAdapter],
    mocker: MockerFixture,
) -> None:
    adapter = adapter_factory(dual_robot)
    mocker.patch.object(adapter, "_build_robot", side_effect=RuntimeError("build failed"))

    assert not adapter.connect()
    assert not adapter.is_connected()


def test_connect_invalid_upstream_group_rolls_back_robot(
    dual_robot: FakeRobot,
    adapter_factory: Callable[..., DualAdapter],
    mocker: MockerFixture,
) -> None:
    adapter = adapter_factory(dual_robot)
    mocker.patch.object(adapter, "_require_arm", side_effect=TypeError("wrong group"))

    assert not adapter.connect()
    assert dual_robot.disable_count == 1
    assert not adapter.is_connected()


def test_disconnect_connected_robot_disables_and_clears_state(
    dual_robot: FakeRobot,
    adapter_factory: Callable[..., DualAdapter],
) -> None:
    adapter = adapter_factory(dual_robot)
    assert adapter.connect()
    assert adapter.activate()

    adapter.disconnect()

    assert dual_robot.disable_count == 1
    assert not adapter.is_connected()
    assert not adapter.has_motor_states()


def test_disconnect_disable_failure_still_clears_local_state(
    dual_robot: FakeRobot,
    adapter_factory: Callable[..., DualAdapter],
) -> None:
    adapter = adapter_factory(dual_robot)
    assert adapter.connect()
    assert adapter.activate()
    dual_robot.disable_error = RuntimeError("disable failed")

    adapter.disconnect()

    assert not adapter.is_connected()
    assert not adapter.has_motor_states()


def test_activate_disconnected_adapter_returns_false(
    dual_robot: FakeRobot,
    adapter_factory: Callable[..., DualAdapter],
) -> None:
    adapter = adapter_factory(dual_robot)

    assert not adapter.activate()
    assert dual_robot.enable_count == 0


def test_activate_enable_failure_disables_robot(
    dual_robot: FakeRobot,
    adapter_factory: Callable[..., DualAdapter],
) -> None:
    adapter = adapter_factory(dual_robot)
    assert adapter.connect()
    dual_robot.enable_error = RuntimeError("calibration failed")

    assert not adapter.activate()
    assert dual_robot.disable_count == 1


def test_deactivate_connected_adapter_disables_robot(
    dual_robot: FakeRobot,
    adapter_factory: Callable[..., DualAdapter],
) -> None:
    adapter = adapter_factory(dual_robot)
    assert adapter.connect()
    assert adapter.activate()

    assert adapter.deactivate()
    assert dual_robot.disable_count == 1
    assert not adapter.has_motor_states()


def test_deactivate_disconnected_adapter_returns_false(
    dual_robot: FakeRobot,
    adapter_factory: Callable[..., DualAdapter],
) -> None:
    adapter = adapter_factory(dual_robot)

    assert not adapter.deactivate()
    assert dual_robot.disable_count == 0


def test_deactivate_disable_failure_returns_false(
    dual_robot: FakeRobot,
    adapter_factory: Callable[..., DualAdapter],
) -> None:
    adapter = adapter_factory(dual_robot)
    assert adapter.connect()
    assert adapter.activate()
    dual_robot.disable_error = RuntimeError("disable failed")

    assert not adapter.deactivate()
    assert adapter.has_motor_states()


def test_has_motor_states_disconnected_adapter_returns_false(
    dual_robot: FakeRobot,
    adapter_factory: Callable[..., DualAdapter],
) -> None:
    adapter = adapter_factory(dual_robot)

    assert not adapter.has_motor_states()


def test_has_motor_states_uncalibrated_grippers_returns_false(
    dual_robot: FakeRobot,
    adapter_factory: Callable[..., DualAdapter],
) -> None:
    adapter = adapter_factory(dual_robot)
    assert adapter.connect()

    assert not adapter.has_motor_states()


def test_has_motor_states_activated_adapter_returns_true(
    active_dual_adapter: DualAdapter,
) -> None:
    assert active_dual_adapter.has_motor_states()


def test_read_motor_states_disconnected_adapter_raises_runtime_error(
    dual_robot: FakeRobot,
    adapter_factory: Callable[..., DualAdapter],
) -> None:
    adapter = adapter_factory(dual_robot)

    with pytest.raises(RuntimeError, match="not connected"):
        adapter.read_motor_states()


def test_joint_names_multiple_groups_returns_declared_order(
    active_dual_adapter: DualAdapter,
) -> None:
    assert active_dual_adapter.joint_names == (
        "left_arm/joint1",
        "left_arm/joint2",
        "right_arm/joint1",
        "right_arm/joint2",
        "left_arm/gripper",
        "right_arm/gripper",
    )


def test_read_motor_states_multiple_groups_returns_ordered_feedback(
    active_dual_adapter: DualAdapter,
) -> None:
    assert active_dual_adapter.read_motor_states() == [
        MotorState(q=0.1),
        MotorState(q=0.2),
        MotorState(q=0.3),
        MotorState(q=0.4),
        MotorState(q=0.5),
        MotorState(q=0.6),
    ]


def test_read_motor_states_valid_feedback_does_not_tick_bus(
    active_dual_adapter: DualAdapter,
    dual_robot: FakeRobot,
) -> None:
    ticks_before = dual_robot.tick_count

    active_dual_adapter.read_motor_states()

    assert dual_robot.tick_count == ticks_before


def test_read_motor_states_wrong_arm_length_raises_runtime_error(
    active_dual_adapter: DualAdapter,
    dual_robot: FakeRobot,
) -> None:
    cast("FakeArm", dual_robot["left_arm"]).velocity_values = np.asarray([0.0])

    with pytest.raises(RuntimeError, match="wrong state length"):
        active_dual_adapter.read_motor_states()


def test_read_motor_states_nonfinite_arm_feedback_raises_runtime_error(
    active_dual_adapter: DualAdapter,
    dual_robot: FakeRobot,
) -> None:
    cast("FakeArm", dual_robot["left_arm"]).torque_values[0] = np.nan

    with pytest.raises(RuntimeError, match="non-finite values"):
        active_dual_adapter.read_motor_states()


def test_read_motor_states_invalid_gripper_opening_raises_runtime_error(
    active_dual_adapter: DualAdapter,
    dual_robot: FakeRobot,
) -> None:
    cast("FakeGripper", dual_robot["left_gripper"]).opening = 1.1

    with pytest.raises(RuntimeError, match="invalid opening"):
        active_dual_adapter.read_motor_states()


def test_write_motor_commands_disconnected_adapter_rejects_command(
    dual_robot: FakeRobot,
    adapter_factory: Callable[..., DualAdapter],
) -> None:
    adapter = adapter_factory(dual_robot)

    assert not adapter.write_motor_commands([MotorCommand()] * 6)


def test_write_motor_commands_inactive_adapter_rejects_command(
    dual_robot: FakeRobot,
    adapter_factory: Callable[..., DualAdapter],
) -> None:
    adapter = adapter_factory(dual_robot)
    assert adapter.connect()

    assert not adapter.write_motor_commands([MotorCommand()] * 6)


def test_write_motor_commands_wrong_command_count_rejects_command(
    active_dual_adapter: DualAdapter,
) -> None:
    assert not active_dual_adapter.write_motor_commands([MotorCommand()] * 5)


def test_write_motor_commands_multiple_arms_routes_ordered_values(
    active_dual_adapter: DualAdapter,
    dual_robot: FakeRobot,
) -> None:
    commands = [
        MotorCommand(q=1.0, kp=10.0),
        MotorCommand(q=1.1, kp=11.0),
        MotorCommand(q=2.0, kp=20.0),
        MotorCommand(q=2.1, kp=21.0),
        MotorCommand(q=0.25),
        MotorCommand(q=0.75),
    ]

    assert active_dual_adapter.write_motor_commands(commands)

    assert cast("FakeArm", dual_robot["left_arm"]).commands[-1].tolist() == [
        [10.0, 0.0, 1.0, 16000.0, 0.0],
        [11.0, 0.0, 1.1, 16000.0, 0.0],
    ]
    assert cast("FakeArm", dual_robot["right_arm"]).commands[-1].tolist() == [
        [20.0, 0.0, 2.0, 16000.0, 0.0],
        [21.0, 0.0, 2.1, 16000.0, 0.0],
    ]


def test_write_motor_commands_grippers_routes_normalized_openings(
    active_dual_adapter: DualAdapter,
    dual_robot: FakeRobot,
) -> None:
    commands = [MotorCommand(q=0.0)] * 4 + [MotorCommand(q=0.25), MotorCommand(q=0.75)]

    assert active_dual_adapter.write_motor_commands(commands)

    assert cast("FakeGripper", dual_robot["left_gripper"]).commands == [0.25]
    assert cast("FakeGripper", dual_robot["right_gripper"]).commands == [0.75]


def test_write_motor_commands_combined_command_ticks_once(
    active_dual_adapter: DualAdapter,
    dual_robot: FakeRobot,
) -> None:
    ticks_before = dual_robot.tick_count

    assert active_dual_adapter.write_motor_commands([MotorCommand(q=0.5)] * 6)

    assert dual_robot.tick_count == ticks_before + 1


def test_write_motor_commands_out_of_range_gripper_rejects_without_writes(
    active_dual_adapter: DualAdapter,
    dual_robot: FakeRobot,
) -> None:
    commands = [MotorCommand(q=0.0)] * 4 + [MotorCommand(q=-0.1), MotorCommand(q=0.5)]

    assert not active_dual_adapter.write_motor_commands(commands)
    assert cast("FakeArm", dual_robot["left_arm"]).commands == []
    assert cast("FakeArm", dual_robot["right_arm"]).commands == []
    assert cast("FakeGripper", dual_robot["left_gripper"]).commands == []
    assert cast("FakeGripper", dual_robot["right_gripper"]).commands == []


def test_write_motor_commands_nonfinite_arm_value_rejects_without_writes(
    active_dual_adapter: DualAdapter,
    dual_robot: FakeRobot,
) -> None:
    commands = [MotorCommand(q=np.nan)] + [MotorCommand(q=0.0)] * 3 + [MotorCommand(q=0.5)] * 2

    assert not active_dual_adapter.write_motor_commands(commands)
    assert cast("FakeArm", dual_robot["left_arm"]).commands == []
    assert cast("FakeGripper", dual_robot["left_gripper"]).commands == []


def test_write_motor_commands_upstream_tick_failure_returns_false(
    active_dual_adapter: DualAdapter,
    dual_robot: FakeRobot,
) -> None:
    dual_robot.tick_error = RuntimeError("bus write failed")

    assert not active_dual_adapter.write_motor_commands([MotorCommand(q=0.5)] * 6)


def test_connect_missing_gravity_model_rolls_back_robot(
    dual_robot: FakeRobot,
    adapter_factory: Callable[..., DualAdapter],
    tmp_path: Path,
) -> None:
    adapter = GravityDualAdapter(
        dual_robot,
        tmp_path / "missing.urdf",
        runtime_config=DamiaoRuntimeConfig(gravity_comp=True),
    )

    assert not adapter.connect()
    assert dual_robot.disable_count == 1


def test_connect_existing_gravity_model_loads_model(
    dual_robot: FakeRobot,
    adapter_factory: Callable[..., DualAdapter],
    mocker: MockerFixture,
    tmp_path: Path,
) -> None:
    model_path = tmp_path / "robot.urdf"
    model_path.write_text("<robot/>")
    model = FakePinModel()
    build_model = mocker.patch(
        "dimos.hardware.whole_body.damiao.adapter.pinocchio.buildModelFromUrdf",
        return_value=model,
    )
    adapter = GravityDualAdapter(
        dual_robot,
        model_path,
        runtime_config=DamiaoRuntimeConfig(gravity_comp=True),
    )

    assert adapter.connect()
    build_model.assert_called_once_with(str(model_path))


def test_activate_gravity_model_dimension_mismatch_returns_false(
    dual_robot: FakeRobot,
    adapter_factory: Callable[..., DualAdapter],
    mocker: MockerFixture,
    tmp_path: Path,
) -> None:
    model_path = tmp_path / "robot.urdf"
    model_path.write_text("<robot/>")
    mocker.patch(
        "dimos.hardware.whole_body.damiao.adapter.pinocchio.buildModelFromUrdf",
        return_value=FakePinModel(nq=3),
    )
    adapter = GravityDualAdapter(
        dual_robot,
        model_path,
        runtime_config=DamiaoRuntimeConfig(gravity_comp=True),
    )
    assert adapter.connect()

    assert not adapter.activate()
    assert dual_robot.disable_count == 1


def test_activate_gravity_joint_order_mismatch_returns_false(
    dual_robot: FakeRobot,
    adapter_factory: Callable[..., DualAdapter],
    mocker: MockerFixture,
    tmp_path: Path,
) -> None:
    model_path = tmp_path / "robot.urdf"
    model_path.write_text("<robot/>")
    mocker.patch(
        "dimos.hardware.whole_body.damiao.adapter.pinocchio.buildModelFromUrdf",
        return_value=FakePinModel(names=("universe", "right1", "left2", "left1", "right2")),
    )
    adapter = GravityDualAdapter(
        dual_robot,
        model_path,
        runtime_config=DamiaoRuntimeConfig(gravity_comp=True),
    )
    assert adapter.connect()

    assert not adapter.activate()
    assert dual_robot.disable_count == 1


def test_activate_nonfinite_arm_positions_returns_false(
    dual_robot: FakeRobot,
    adapter_factory: Callable[..., DualAdapter],
    mocker: MockerFixture,
    tmp_path: Path,
) -> None:
    model_path = tmp_path / "robot.urdf"
    model_path.write_text("<robot/>")
    mocker.patch(
        "dimos.hardware.whole_body.damiao.adapter.pinocchio.buildModelFromUrdf",
        return_value=FakePinModel(),
    )
    adapter = GravityDualAdapter(
        dual_robot,
        model_path,
        runtime_config=DamiaoRuntimeConfig(gravity_comp=True),
    )
    assert adapter.connect()
    cast("FakeArm", dual_robot["left_arm"]).position_values[0] = np.nan

    assert not adapter.activate()
    assert dual_robot.disable_count == 1


def test_activate_nonfinite_gravity_output_returns_false(
    dual_robot: FakeRobot,
    adapter_factory: Callable[..., DualAdapter],
    mocker: MockerFixture,
    tmp_path: Path,
) -> None:
    model_path = tmp_path / "robot.urdf"
    model_path.write_text("<robot/>")
    mocker.patch(
        "dimos.hardware.whole_body.damiao.adapter.pinocchio.buildModelFromUrdf",
        return_value=FakePinModel(),
    )
    mocker.patch(
        "dimos.hardware.whole_body.damiao.adapter.pinocchio.computeGeneralizedGravity",
        return_value=np.asarray([1.0, 2.0, np.nan, 4.0]),
    )
    adapter = GravityDualAdapter(
        dual_robot,
        model_path,
        runtime_config=DamiaoRuntimeConfig(gravity_comp=True),
    )
    assert adapter.connect()

    assert not adapter.activate()
    assert dual_robot.disable_count == 1


def test_write_motor_commands_gravity_enabled_adds_computed_torque(
    dual_robot: FakeRobot,
    adapter_factory: Callable[..., DualAdapter],
    mocker: MockerFixture,
    tmp_path: Path,
) -> None:
    model_path = tmp_path / "robot.urdf"
    model_path.write_text("<robot/>")
    mocker.patch(
        "dimos.hardware.whole_body.damiao.adapter.pinocchio.buildModelFromUrdf",
        return_value=FakePinModel(),
    )
    compute_gravity = mocker.patch(
        "dimos.hardware.whole_body.damiao.adapter.pinocchio.computeGeneralizedGravity",
        return_value=np.asarray([1.0, 2.0, 3.0, 4.0]),
    )
    adapter = GravityDualAdapter(
        dual_robot,
        model_path,
        runtime_config=DamiaoRuntimeConfig(gravity_comp=True),
    )
    assert adapter.connect()
    assert adapter.activate()
    compute_gravity.reset_mock()

    commands = [MotorCommand(q=0.0, tau=0.5)] * 4 + [MotorCommand(q=0.5)] * 2
    assert adapter.write_motor_commands(commands)

    assert compute_gravity.call_count == 1
    assert cast("FakeArm", dual_robot["left_arm"]).commands[-1][:, 4].tolist() == [1.5, 2.5]
    assert cast("FakeArm", dual_robot["right_arm"]).commands[-1][:, 4].tolist() == [3.5, 4.5]

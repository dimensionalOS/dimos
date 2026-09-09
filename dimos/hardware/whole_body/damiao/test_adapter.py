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
from unittest.mock import Mock
import weakref

import can_motor_control
import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.hardware.whole_body.damiao import adapter as adapter_module
from dimos.hardware.whole_body.damiao.adapter import DamiaoWholeBodyAdapter
from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.hardware.whole_body.spec import MotorCommand, MotorState
from dimos.robot.assets.model import RobotModel


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


class FakeTransport:
    def __init__(self) -> None:
        self.closed = False


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

    def command_count(self) -> int:
        arms = sum(
            len(group.commands) for group in self.groups.values() if isinstance(group, FakeArm)
        )
        grippers = sum(
            len(group.commands) for group in self.groups.values() if isinstance(group, FakeGripper)
        )
        return arms + grippers


class DualAdapter(DamiaoWholeBodyAdapter):
    arm_joints = {
        "left_arm": ("left_arm/joint1", "left_arm/joint2"),
        "right_arm": ("right_arm/joint1", "right_arm/joint2"),
    }
    gripper_joints = {
        "left_gripper": "left_arm/gripper",
        "right_gripper": "right_arm/gripper",
    }
    bus_names = ("left", "right")

    def __init__(self, robot: FakeRobot, **kwargs: object) -> None:
        self.fake_robot = robot
        super().__init__(**kwargs)

    def _build_robot(self) -> can_motor_control.Robot:
        return cast("can_motor_control.Robot", self.fake_robot)


class RebuildingDualAdapter(DualAdapter):
    def __init__(
        self,
        robot_factory: Callable[[], FakeRobot],
        runtime_config: DamiaoRuntimeConfig,
    ) -> None:
        self.robot_factory = robot_factory
        DamiaoWholeBodyAdapter.__init__(self, runtime_config=runtime_config)

    def _build_robot(self) -> can_motor_control.Robot:
        return cast("can_motor_control.Robot", self.robot_factory())


class GravityDualAdapter(DualAdapter):
    kinematic_joint_names = ("left1", "left2", "right1", "right2")

    def __init__(self, robot: FakeRobot, model_path: Path, **kwargs: object) -> None:
        self.model_path = model_path
        super().__init__(robot, **kwargs)

    @property
    def kinematic_model(self) -> RobotModel:
        return RobotModel.from_file(self.model_path)


class FakePinModel:
    def __init__(
        self,
        *,
        nq: int = 4,
        nv: int = 4,
        names: tuple[str, ...] = ("universe", "left1", "left2", "right1", "right2"),
        lower: tuple[float, ...] = (-1.0, -2.0, -3.0, -4.0),
        upper: tuple[float, ...] = (1.0, 2.0, 3.0, 4.0),
    ) -> None:
        self.nq = nq
        self.nv = nv
        self.names = names
        self.lowerPositionLimit = np.asarray(lower, dtype=np.float64)
        self.upperPositionLimit = np.asarray(upper, dtype=np.float64)
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
def connected_dual_adapter(
    dual_robot: FakeRobot,
    adapter_factory: Callable[..., DualAdapter],
) -> Iterator[DualAdapter]:
    adapter = adapter_factory(dual_robot, dof=6)
    assert adapter.connect()
    yield adapter
    adapter.disconnect()


@pytest.fixture
def active_dual_adapter(connected_dual_adapter: DualAdapter) -> DualAdapter:
    assert connected_dual_adapter.activate()
    return connected_dual_adapter


@pytest.fixture
def pin_model_builder(mocker: MockerFixture) -> Mock:
    return mocker.patch(
        "dimos.hardware.whole_body.damiao.adapter.pinocchio.buildModelFromXML",
    )


@pytest.fixture
def gravity_adapter_factory(
    adapter_factory: Callable[..., DualAdapter],
    dual_robot: FakeRobot,
    pin_model_builder: Mock,
    tmp_path: Path,
) -> Iterator[Callable[..., GravityDualAdapter]]:
    model_path = tmp_path / "robot.urdf"
    model_path.write_text("<robot/>")
    adapters: list[GravityDualAdapter] = []

    def create(*, model: FakePinModel, gravity_comp: bool = True) -> GravityDualAdapter:
        pin_model_builder.return_value = model
        adapter = GravityDualAdapter(
            dual_robot,
            model_path,
            runtime_config=DamiaoRuntimeConfig(gravity_comp=gravity_comp),
        )
        adapters.append(adapter)
        return adapter

    yield create
    for adapter in adapters:
        adapter.disconnect()


def test_init_scalar_address_raises_named_bus_configuration_error(
    dual_robot: FakeRobot,
) -> None:
    with pytest.raises(ValueError, match="runtime_config.bus_devices"):
        DualAdapter(dual_robot, address="can0")


def test_init_unknown_bus_override_raises_value_error(dual_robot: FakeRobot) -> None:
    with pytest.raises(ValueError, match="unknown CAN bus"):
        DualAdapter(
            dual_robot,
            runtime_config=DamiaoRuntimeConfig(bus_devices={"missing": "can9"}),
        )


def test_init_duplicate_logical_bus_names_raises_value_error(dual_robot: FakeRobot) -> None:
    class DuplicateBusAdapter(DualAdapter):
        bus_names = ("left", "left")

    with pytest.raises(ValueError, match="duplicate logical bus names"):
        DuplicateBusAdapter(dual_robot)


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
        kinematic_joint_names = ("left1",)

    with pytest.raises(ValueError, match="every angular arm joint"):
        IncompleteGravityAdapter(dual_robot)


def test_make_can_bus_linux_uses_ordered_defaults(
    dual_robot: FakeRobot,
    mocker: MockerFixture,
) -> None:
    socketcan = mocker.patch.object(can_motor_control, "SocketCanBus")
    mocker.patch.object(adapter_module.sys, "platform", "linux")
    adapter = DualAdapter(dual_robot)

    assert adapter._make_can_bus("left") is socketcan.return_value
    assert adapter._make_can_bus("right") is socketcan.return_value
    assert socketcan.call_args_list == [mocker.call("can0"), mocker.call("can1")]


def test_make_can_bus_linux_uses_configured_interface(
    dual_robot: FakeRobot,
    mocker: MockerFixture,
) -> None:
    socketcan = mocker.patch.object(can_motor_control, "SocketCanBus")
    mocker.patch.object(adapter_module.sys, "platform", "linux")
    adapter = DualAdapter(
        dual_robot,
        runtime_config=DamiaoRuntimeConfig(bus_devices={"left": "can8"}),
    )

    assert adapter._make_can_bus("left") is socketcan.return_value
    socketcan.assert_called_once_with("can8")


def test_init_rehydrates_serialized_runtime_config(dual_robot: FakeRobot) -> None:
    adapter = DualAdapter(
        dual_robot,
        runtime_config={
            "bus_devices": {"left": "can8"},
            "gravity_comp": False,
            "tick_deadline_us": 2_000,
        },
    )

    assert adapter._runtime_config.bus_devices == {"left": "can8"}
    assert adapter._runtime_config.gravity_comp is False
    assert adapter._runtime_config.tick_deadline_us == 2_000


def test_make_can_bus_macos_uses_ordered_indices(
    dual_robot: FakeRobot,
    mocker: MockerFixture,
) -> None:
    gs_usb = mocker.patch.object(can_motor_control, "GsUsbBus", create=True)
    mocker.patch.object(adapter_module.sys, "platform", "darwin")
    adapter = DualAdapter(dual_robot)

    assert adapter._make_can_bus("left") is gs_usb.return_value
    assert adapter._make_can_bus("right") is gs_usb.return_value
    assert gs_usb.call_args_list == [
        mocker.call(vendor_id=0x1D50, product_id=0x606F, index=0),
        mocker.call(vendor_id=0x1D50, product_id=0x606F, index=1),
    ]


def test_make_can_bus_macos_uses_configured_serial_number_and_device_ids(
    dual_robot: FakeRobot,
    mocker: MockerFixture,
) -> None:
    class VendorAdapter(DualAdapter):
        gs_usb_vendor_id = 0x1234
        gs_usb_product_id = 0x5678

    gs_usb = mocker.patch.object(can_motor_control, "GsUsbBus", create=True)
    mocker.patch.object(adapter_module.sys, "platform", "darwin")
    adapter = VendorAdapter(
        dual_robot,
        runtime_config=DamiaoRuntimeConfig(bus_devices={"right": "serial-B"}),
    )

    assert adapter._make_can_bus("right") is gs_usb.return_value
    gs_usb.assert_called_once_with(
        vendor_id=0x1234,
        product_id=0x5678,
        serial_number="serial-B",
    )


def test_make_can_bus_undeclared_bus_raises_value_error(dual_robot: FakeRobot) -> None:
    adapter = DualAdapter(dual_robot)

    with pytest.raises(ValueError, match="did not declare CAN bus 'missing'"):
        adapter._make_can_bus("missing")


def test_make_can_bus_unsupported_platform_raises_runtime_error(
    dual_robot: FakeRobot,
    mocker: MockerFixture,
) -> None:
    mocker.patch.object(adapter_module.sys, "platform", "win32")
    adapter = DualAdapter(dual_robot)

    with pytest.raises(RuntimeError, match="unsupported on win32"):
        adapter._make_can_bus("left")


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


@pytest.mark.parametrize("failure_stage", ["group", "kinematic", "refresh"])
def test_connect_post_connect_failure_releases_transport_and_allows_reconnect(
    failure_stage: str,
    mocker: MockerFixture,
) -> None:
    transports: list[FakeTransport] = []

    def build_robot() -> FakeRobot:
        robot = FakeRobot(
            {
                "left_arm": FakeArm([0.1, 0.2]),
                "right_arm": FakeArm([0.3, 0.4]),
                "left_gripper": FakeGripper(0.5),
                "right_gripper": FakeGripper(0.6),
            }
        )
        transport = FakeTransport()
        transports.append(transport)
        weakref.finalize(robot, setattr, transport, "closed", True)
        return robot

    adapter = RebuildingDualAdapter(
        build_robot,
        runtime_config=DamiaoRuntimeConfig(gravity_comp=False),
    )
    group_failure_pending = failure_stage == "group"
    kinematic_failure_pending = failure_stage == "kinematic"
    refresh_failure_pending = failure_stage == "refresh"

    def require_arm(robot: FakeRobot, name: str) -> FakeArm:
        nonlocal group_failure_pending
        if group_failure_pending:
            group_failure_pending = False
            raise TypeError("wrong group")
        return cast("FakeArm", robot[name])

    mocker.patch.object(adapter, "_require_arm", new=require_arm)
    mocker.patch.object(
        adapter,
        "_require_gripper",
        new=lambda robot, name: robot[name],
    )

    def load_kinematic_model() -> None:
        nonlocal kinematic_failure_pending
        if kinematic_failure_pending:
            kinematic_failure_pending = False
            raise RuntimeError("kinematic load failed")

    def refresh() -> None:
        nonlocal refresh_failure_pending
        if refresh_failure_pending:
            refresh_failure_pending = False
            raise RuntimeError("refresh failed")

    mocker.patch.object(adapter, "_load_kinematic_model", new=load_kinematic_model)
    mocker.patch.object(adapter, "_refresh", new=refresh)

    assert not adapter.connect()
    assert transports[0].closed
    try:
        assert adapter.connect()
    finally:
        adapter.disconnect()
    assert transports[1].closed


def test_disconnect_connected_robot_disables_and_clears_state(
    connected_dual_adapter: DualAdapter,
    dual_robot: FakeRobot,
) -> None:
    assert not connected_dual_adapter.has_motor_states()
    assert connected_dual_adapter.activate()
    assert connected_dual_adapter.has_motor_states()

    connected_dual_adapter.disconnect()

    assert dual_robot.disable_count == 1
    assert not connected_dual_adapter.is_connected()
    assert not connected_dual_adapter.has_motor_states()


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
    active_dual_adapter: DualAdapter,
    dual_robot: FakeRobot,
) -> None:
    assert active_dual_adapter.has_motor_states()
    assert active_dual_adapter.deactivate()
    assert dual_robot.disable_count == 1
    assert not active_dual_adapter.has_motor_states()


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


def test_read_motor_states_refreshes_feedback_before_snapshot(
    active_dual_adapter: DualAdapter,
    dual_robot: FakeRobot,
) -> None:
    refreshes_before = dual_robot.refresh_count
    ticks_before = dual_robot.tick_count

    active_dual_adapter.read_motor_states()

    assert dual_robot.refresh_count == refreshes_before + 1
    assert dual_robot.tick_count == ticks_before + 1


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

    assert cast("FakeArm", dual_robot["left_arm"]).commands[-1][:, 2].tolist() == [1.0, 1.1]
    assert cast("FakeArm", dual_robot["right_arm"]).commands[-1][:, 2].tolist() == [2.0, 2.1]


def test_write_motor_commands_encodes_complete_mit_command(
    active_dual_adapter: DualAdapter,
    dual_robot: FakeRobot,
) -> None:
    command = MotorCommand(q=1.0, dq=2.0, kp=3.0, kd=4.0, tau=5.0)
    commands = [command, *[MotorCommand(q=0.0)] * 3, *[MotorCommand(q=0.5)] * 2]

    assert active_dual_adapter.write_motor_commands(commands)

    assert cast("FakeArm", dual_robot["left_arm"]).commands[-1][0].tolist() == [
        3.0,
        4.0,
        1.0,
        2.0,
        5.0,
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
    assert dual_robot.command_count() == 0


def test_write_motor_commands_nonfinite_arm_value_rejects_without_writes(
    active_dual_adapter: DualAdapter,
    dual_robot: FakeRobot,
) -> None:
    commands = [MotorCommand(q=np.nan)] + [MotorCommand(q=0.0)] * 3 + [MotorCommand(q=0.5)] * 2

    assert not active_dual_adapter.write_motor_commands(commands)
    assert dual_robot.command_count() == 0


def test_write_motor_commands_upstream_tick_failure_returns_false(
    active_dual_adapter: DualAdapter,
    dual_robot: FakeRobot,
) -> None:
    dual_robot.tick_error = RuntimeError("bus write failed")

    assert not active_dual_adapter.write_motor_commands([MotorCommand(q=0.5)] * 6)


def test_connect_missing_kinematic_model_rolls_back_robot(
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


def test_connect_existing_kinematic_model_loads_model(
    gravity_adapter_factory: Callable[..., GravityDualAdapter],
    pin_model_builder: Mock,
) -> None:
    adapter = gravity_adapter_factory(model=FakePinModel(), gravity_comp=False)

    assert adapter.connect()
    pin_model_builder.assert_called_once()


def test_connect_kinematic_model_dimension_mismatch_returns_false(
    dual_robot: FakeRobot,
    gravity_adapter_factory: Callable[..., GravityDualAdapter],
) -> None:
    adapter = gravity_adapter_factory(model=FakePinModel(nq=3))
    assert not adapter.connect()
    assert dual_robot.disable_count == 1


def test_connect_kinematic_joint_order_mismatch_returns_false(
    dual_robot: FakeRobot,
    gravity_adapter_factory: Callable[..., GravityDualAdapter],
) -> None:
    adapter = gravity_adapter_factory(
        model=FakePinModel(names=("universe", "right1", "left2", "left1", "right2")),
    )
    assert not adapter.connect()
    assert dual_robot.disable_count == 1


def test_connect_invalid_kinematic_limits_returns_false(
    dual_robot: FakeRobot,
    gravity_adapter_factory: Callable[..., GravityDualAdapter],
) -> None:
    adapter = gravity_adapter_factory(
        model=FakePinModel(lower=(-1.0, -2.0, 3.0, -4.0), upper=(1.0, 2.0, 3.0, 4.0))
    )

    assert not adapter.connect()
    assert dual_robot.disable_count == 1


def test_read_motor_states_clamps_small_angular_overshoot(
    dual_robot: FakeRobot,
    gravity_adapter_factory: Callable[..., GravityDualAdapter],
) -> None:
    adapter = gravity_adapter_factory(model=FakePinModel(), gravity_comp=False)
    assert adapter.connect()
    assert adapter.activate()
    left_arm = cast("FakeArm", dual_robot["left_arm"])
    left_arm.position_values[0] = -1.001
    left_arm.velocity_values[0] = 0.3
    left_arm.torque_values[0] = 0.4

    states = adapter.read_motor_states()

    assert states[0] == MotorState(q=-1.0, dq=0.3, tau=0.4)


@pytest.mark.parametrize("value, expected", [(-1.05, -1.0), (1.05, 1.0)])
def test_read_motor_states_accepts_overshoot_at_clamp_margin(
    dual_robot: FakeRobot,
    gravity_adapter_factory: Callable[..., GravityDualAdapter],
    value: float,
    expected: float,
) -> None:
    adapter = gravity_adapter_factory(model=FakePinModel(), gravity_comp=False)
    assert adapter.connect()
    assert adapter.activate()
    cast("FakeArm", dual_robot["left_arm"]).position_values[0] = value

    assert adapter.read_motor_states()[0].q == pytest.approx(expected)


def test_gross_feedback_violation_disables_and_latches_adapter(
    dual_robot: FakeRobot,
    gravity_adapter_factory: Callable[..., GravityDualAdapter],
) -> None:
    adapter = gravity_adapter_factory(model=FakePinModel(), gravity_comp=False)
    assert adapter.connect()
    assert adapter.activate()
    cast("FakeArm", dual_robot["left_arm"]).position_values[0] = -1.051

    with pytest.raises(RuntimeError, match=r"left_arm/joint1.*-1.051.*\[-1.0, 1.0\]"):
        adapter.read_motor_states()

    assert not adapter.activate()
    assert not adapter.write_motor_commands([MotorCommand()] * 6)


def test_feedback_fault_fails_closed_when_disable_fails(
    dual_robot: FakeRobot,
    gravity_adapter_factory: Callable[..., GravityDualAdapter],
) -> None:
    adapter = gravity_adapter_factory(model=FakePinModel(), gravity_comp=False)
    assert adapter.connect()
    assert adapter.activate()
    dual_robot.disable_error = RuntimeError("disable failed")
    cast("FakeArm", dual_robot["left_arm"]).position_values[0] = -1.051

    with pytest.raises(RuntimeError, match="motor disable failed"):
        adapter.read_motor_states()

    assert not adapter._active
    assert not adapter.activate()


def test_reconnect_clears_feedback_fault(
    dual_robot: FakeRobot,
    gravity_adapter_factory: Callable[..., GravityDualAdapter],
) -> None:
    adapter = gravity_adapter_factory(model=FakePinModel(), gravity_comp=False)
    assert adapter.connect()
    assert adapter.activate()
    left_arm = cast("FakeArm", dual_robot["left_arm"])
    left_arm.position_values[0] = -1.051
    with pytest.raises(RuntimeError, match="feedback fault"):
        adapter.read_motor_states()
    adapter.disconnect()
    left_arm.position_values[0] = 0.0

    assert adapter.connect()
    assert adapter.activate()
    assert adapter.read_motor_states()[0].q == 0.0


def test_activate_nonfinite_arm_positions_returns_false(
    dual_robot: FakeRobot,
    gravity_adapter_factory: Callable[..., GravityDualAdapter],
) -> None:
    adapter = gravity_adapter_factory(model=FakePinModel())
    assert adapter.connect()
    cast("FakeArm", dual_robot["left_arm"]).position_values[0] = np.nan

    assert not adapter.activate()
    assert dual_robot.disable_count == 1


def test_activate_nonfinite_gravity_output_returns_false(
    dual_robot: FakeRobot,
    gravity_adapter_factory: Callable[..., GravityDualAdapter],
    mocker: MockerFixture,
) -> None:
    adapter = gravity_adapter_factory(model=FakePinModel())
    mocker.patch(
        "dimos.hardware.whole_body.damiao.adapter.pinocchio.computeGeneralizedGravity",
        return_value=np.asarray([1.0, 2.0, np.nan, 4.0]),
    )
    assert adapter.connect()

    assert not adapter.activate()
    assert dual_robot.disable_count == 1


def test_write_motor_commands_gravity_enabled_adds_computed_torque(
    dual_robot: FakeRobot,
    gravity_adapter_factory: Callable[..., GravityDualAdapter],
    mocker: MockerFixture,
) -> None:
    adapter = gravity_adapter_factory(model=FakePinModel())
    compute_gravity = mocker.patch(
        "dimos.hardware.whole_body.damiao.adapter.pinocchio.computeGeneralizedGravity",
        return_value=np.asarray([1.0, 2.0, 3.0, 4.0]),
    )
    assert adapter.connect()
    assert adapter.activate()
    compute_gravity.reset_mock()

    commands = [MotorCommand(q=0.0, tau=0.5)] * 4 + [MotorCommand(q=0.5)] * 2
    assert adapter.write_motor_commands(commands)

    assert compute_gravity.call_count == 1
    assert cast("FakeArm", dual_robot["left_arm"]).commands[-1][:, 4].tolist() == [1.5, 2.5]
    assert cast("FakeArm", dual_robot["right_arm"]).commands[-1][:, 4].tolist() == [3.5, 4.5]


def test_read_motor_states_inactive_gripper_reports_placeholder(
    connected_dual_adapter: DualAdapter,
    dual_robot: FakeRobot,
) -> None:
    """Gripper opening calibrates at activation; before that the read path
    must not touch it so read-only bring-up sessions still stream arm state."""
    cast("FakeGripper", dual_robot["left_gripper"]).opening = None
    cast("FakeGripper", dual_robot["right_gripper"]).opening = None

    states = connected_dual_adapter.read_motor_states()

    assert states[4:] == [MotorState(q=0.0), MotorState(q=0.0)]


def test_read_motor_states_inactive_adapter_pumps_feedback(
    connected_dual_adapter: DualAdapter,
    dual_robot: FakeRobot,
) -> None:
    """Without the active write path ticking the bus, the read path must
    refresh feedback itself or read-only sessions stream a frozen snapshot."""
    refreshes_before = dual_robot.refresh_count
    ticks_before = dual_robot.tick_count

    connected_dual_adapter.read_motor_states()
    connected_dual_adapter.read_motor_states()

    assert dual_robot.refresh_count == refreshes_before + 2
    assert dual_robot.tick_count == ticks_before + 2

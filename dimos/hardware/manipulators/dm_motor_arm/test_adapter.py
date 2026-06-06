# Copyright 2025-2026 Dimensional Inc.
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

from enum import IntEnum
import importlib
from types import ModuleType
from unittest.mock import MagicMock

import numpy as np
import pytest

from dimos.hardware.manipulators.dm_motor_arm.adapter import (
    DMMotorArm,
    DMMotorBindingUnavailableError,
    register,
)
from dimos.hardware.manipulators.spec import ControlMode, ManipulatorAdapter


class FakeMotorType(IntEnum):
    DM4310 = 1
    DM4340 = 3
    DM8006 = 6


class FakeDamiao(ModuleType):
    MotorType = FakeMotorType

    class DamiaoCodec:
        pass


class FakeMotorSpec:
    def __init__(self, name: str, type: FakeMotorType, send_id: int, recv_id: int) -> None:
        if not isinstance(type, FakeMotorType):
            raise TypeError("type must be FakeMotorType")
        self.name = name
        self.type = type
        self.send_id = send_id
        self.recv_id = recv_id


class FakeMotor:
    fault: int | None = None


class FakeArm:
    def __init__(self, dof: int) -> None:
        self.dof = dof
        self.positions_value = np.array([0.1 * i for i in range(dof)], dtype=np.float64)
        self.velocities_value = np.array([0.2 * i for i in range(dof)], dtype=np.float64)
        self.torques_value = np.array([0.3 * i for i in range(dof)], dtype=np.float64)
        self.mit_commands: list[np.ndarray] = []
        self.position_commands: list[np.ndarray] = []
        self.velocity_commands: list[np.ndarray] = []
        self.motors = {f"joint{i + 1}": FakeMotor() for i in range(dof)}

    def __len__(self) -> int:
        return self.dof

    def __getitem__(self, name: str) -> FakeMotor:
        return self.motors[name]

    def positions(self) -> np.ndarray:
        return self.positions_value

    def velocities(self) -> np.ndarray:
        return self.velocities_value

    def torques(self) -> np.ndarray:
        return self.torques_value

    def mit_control(self, cmds: np.ndarray) -> None:
        self.mit_commands.append(cmds.copy())

    def pos_vel_control(self, cmds: np.ndarray) -> None:
        self.position_commands.append(cmds.copy())

    def vel_control(self, cmds: np.ndarray) -> None:
        self.velocity_commands.append(cmds.copy())


class FakeRobot:
    last: FakeRobot | None = None

    def __init__(self, dof: int = 7, transport: object | None = None) -> None:
        FakeRobot.last = self
        self.arm = FakeArm(dof)
        self.transport = transport
        self.config_path: str | None = None
        self.connected = False
        self.enabled = False
        self.tick_count = 0
        self.disabled_count = 0

    @classmethod
    def builder(cls) -> FakeRobotBuilder:
        return FakeRobotBuilder()

    @classmethod
    def from_config(cls, path: str) -> FakeRobot:
        robot = cls()
        robot.config_path = path
        return robot

    def connect(self) -> None:
        self.connected = True

    def enable(self) -> None:
        self.enabled = True

    def disable(self) -> None:
        self.enabled = False
        self.disabled_count += 1

    def tick(self, per_bus_deadline_us: int) -> None:
        self.tick_count += 1

    def __getitem__(self, name: str) -> FakeArm:
        return self.arm


class FakeRobotBuilder:
    def __init__(self) -> None:
        self.motors: list[FakeMotorSpec] = []
        self.transport: object | None = None

    def add_bus(self, name: str, transport: object, codec: object) -> FakeRobotBuilder:
        self.transport = transport
        return self

    def add_arm(self, name: str, *, bus: str, motors: list[FakeMotorSpec]) -> FakeRobotBuilder:
        self.motors = motors
        return self

    def build(self) -> FakeRobot:
        return FakeRobot(len(self.motors), transport=self.transport)


class FakeDMControl(ModuleType):
    Robot = FakeRobot
    MotorSpec = FakeMotorSpec

    class MockCanBus:
        def __init__(self, name: str, fd: bool = False) -> None:
            self.name = name
            self.fd = fd

        @staticmethod
        def new_fd(name: str) -> FakeDMControl.MockCanBus:
            return FakeDMControl.MockCanBus(name, fd=True)

    class SocketCanBus:
        def __init__(self, interface: str, fd: bool = False) -> None:
            self.interface = interface
            self.fd = fd


@pytest.fixture(autouse=True)
def fake_can_motor_control(monkeypatch: pytest.MonkeyPatch) -> None:
    fake_dm = FakeDMControl("can_motor_control")
    fake_damiao = FakeDamiao("can_motor_control.damiao")
    monkeypatch.setitem(__import__("sys").modules, "can_motor_control", fake_dm)
    monkeypatch.setitem(__import__("sys").modules, "can_motor_control.damiao", fake_damiao)
    FakeRobot.last = None


def test_implements_manipulator_adapter() -> None:
    assert isinstance(DMMotorArm(use_mock_bus=True), ManipulatorAdapter)


def test_register() -> None:
    registry = MagicMock()
    register(registry)
    registry.register.assert_called_once_with("dm_motor_arm", DMMotorArm)


def test_defaults_match_openarm_ros2_hardware_presets() -> None:
    adapter = DMMotorArm(use_mock_bus=True)
    assert adapter._kp == pytest.approx([70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0])
    assert adapter._kd == pytest.approx([2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5])


def test_canfd_enabled_by_default_for_mock_and_socket_buses() -> None:
    mock_adapter = DMMotorArm(use_mock_bus=True)
    assert mock_adapter.connect() is True
    mock_robot = FakeRobot.last
    assert mock_robot is not None
    assert mock_adapter._fd is True
    assert isinstance(mock_robot.transport, FakeDMControl.MockCanBus)
    assert mock_robot.transport.fd is True

    socket_adapter = DMMotorArm(use_mock_bus=False)
    assert socket_adapter.connect() is True
    socket_robot = FakeRobot.last
    assert socket_robot is not None
    assert socket_adapter._fd is True
    assert isinstance(socket_robot.transport, FakeDMControl.SocketCanBus)
    assert socket_robot.transport.fd is True


def test_canfd_flag_and_legacy_fd_override_can_disable_fd() -> None:
    canfd_adapter = DMMotorArm(use_mock_bus=True, canfd=False)
    assert canfd_adapter.connect() is True
    canfd_robot = FakeRobot.last
    assert canfd_robot is not None
    assert canfd_adapter._fd is False
    assert isinstance(canfd_robot.transport, FakeDMControl.MockCanBus)
    assert canfd_robot.transport.fd is False

    fd_adapter = DMMotorArm(use_mock_bus=True, canfd=True, fd=False)
    assert fd_adapter.connect() is True
    fd_robot = FakeRobot.last
    assert fd_robot is not None
    assert fd_adapter._fd is False
    assert isinstance(fd_robot.transport, FakeDMControl.MockCanBus)
    assert fd_robot.transport.fd is False


def test_motor_specs_use_binding_motor_type_values() -> None:
    adapter = DMMotorArm(
        use_mock_bus=True,
        dof=2,
        motor_specs=[
            {"name": "joint1", "type": "DM4310", "send_id": 1, "recv_id": 17},
            {"name": "joint2", "type": 6, "send_id": 2, "recv_id": 18},
        ],
    )
    assert adapter.connect() is True
    assert adapter._robot is not None
    assert adapter._robot.arm.dof == 2


def test_renamed_can_motor_control_binding_is_used(monkeypatch: pytest.MonkeyPatch) -> None:
    imported: list[str] = []
    real_import_module = importlib.import_module

    def track_can_motor_control(name: str, package: str | None = None) -> ModuleType:
        if name.startswith("dm_control"):
            raise ImportError(name)
        imported.append(name)
        return real_import_module(name, package)

    monkeypatch.setattr(importlib, "import_module", track_can_motor_control)
    adapter = DMMotorArm(use_mock_bus=True)
    assert adapter.connect() is True
    assert imported[:2] == ["can_motor_control", "can_motor_control.damiao"]


def test_missing_binding_fails_only_when_selected(monkeypatch: pytest.MonkeyPatch) -> None:
    real_import_module = importlib.import_module

    def fail_can_motor_control(name: str, package: str | None = None) -> ModuleType:
        if name.startswith("can_motor_control"):
            raise ImportError(name)
        return real_import_module(name, package)

    monkeypatch.setattr(importlib, "import_module", fail_can_motor_control)
    adapter = DMMotorArm(use_mock_bus=True)
    with pytest.raises(DMMotorBindingUnavailableError, match="requires.*can-motor-control"):
        adapter.connect()


def test_lifecycle_read_write_disable() -> None:
    adapter = DMMotorArm(use_mock_bus=True, gravity_comp=False)
    assert adapter.connect() is True
    assert adapter.write_enable(True) is True
    assert adapter.read_enabled() is True
    assert adapter.read_joint_positions() == pytest.approx([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    assert adapter.write_joint_positions([0.1] * 7, velocity=0.5) is True
    robot = FakeRobot.last
    assert robot is not None
    assert robot.arm.position_commands == []
    assert robot.arm.velocity_commands == []
    cmd = robot.arm.mit_commands[-1]
    assert cmd[:, 2].tolist() == pytest.approx([0.1] * 7)
    assert cmd[:, 0].tolist() == pytest.approx([35.0, 35.0, 35.0, 30.0, 5.0, 5.0, 5.0])
    assert cmd[:, 1].tolist() == pytest.approx([2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5])
    assert cmd[:, 4].tolist() == pytest.approx([0.0] * 7)
    adapter.disconnect()
    assert robot.disabled_count >= 1


def test_state_reads_share_one_tick() -> None:
    adapter = DMMotorArm(use_mock_bus=True, state_cache_ttl_s=10.0)
    assert adapter.connect() is True
    robot = FakeRobot.last
    assert robot is not None
    robot.tick_count = 0
    adapter._state_cache = None
    adapter.read_joint_positions()
    adapter.read_joint_velocities()
    adapter.read_joint_efforts()
    assert robot.tick_count == 1


def test_default_gains_match_openarm_ros2_presets() -> None:
    adapter = DMMotorArm(use_mock_bus=True)
    assert adapter.connect() is True
    assert adapter.write_enable(True) is True
    assert adapter.write_joint_positions([0.1] * 7) is True
    robot = FakeRobot.last
    assert robot is not None
    cmd = robot.arm.mit_commands[-1]
    assert cmd[:, 0].tolist() == pytest.approx([70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0])
    assert cmd[:, 1].tolist() == pytest.approx([2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5])


def test_position_commands_use_in_place_gravity_compensation_by_default(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    adapter = DMMotorArm(use_mock_bus=True, kp=[10.0] * 7, kd=[0.2] * 7)
    assert adapter.connect() is True
    assert adapter.write_enable(True) is True
    monkeypatch.setattr(adapter, "compute_gravity_torques", lambda q: [1.0] * 7)
    assert adapter.write_joint_positions([0.1] * 7, velocity=0.5) is True
    robot = FakeRobot.last
    assert robot is not None
    assert robot.arm.position_commands == []
    cmd = robot.arm.mit_commands[-1]
    assert cmd[:, 2].tolist() == pytest.approx([0.1] * 7)
    assert cmd[:, 3].tolist() == pytest.approx([0.0] * 7)
    assert cmd[:, 0].tolist() == pytest.approx([5.0] * 7)
    assert cmd[:, 1].tolist() == pytest.approx([0.2] * 7)
    assert cmd[:, 4].tolist() == pytest.approx([1.0] * 7)
    assert adapter.get_control_mode() == ControlMode.POSITION


def test_velocity_mode_and_commands_are_unsupported() -> None:
    adapter = DMMotorArm(use_mock_bus=True)
    assert adapter.connect() is True
    assert adapter.write_enable(True) is True
    assert adapter.set_control_mode(ControlMode.VELOCITY) is False
    assert adapter.write_joint_velocities([0.2] * 7) is False
    robot = FakeRobot.last
    assert robot is not None
    assert robot.arm.mit_commands == []
    assert robot.arm.velocity_commands == []
    assert adapter.get_control_mode() == ControlMode.POSITION


def test_gravity_comp_false_uses_mit_position_without_effort() -> None:
    adapter = DMMotorArm(use_mock_bus=True, gravity_comp=False, kp=[10.0] * 7, kd=[0.2] * 7)
    assert adapter.connect() is True
    assert adapter.write_enable(True) is True
    assert adapter.write_joint_positions([0.1] * 7, velocity=0.5) is True
    robot = FakeRobot.last
    assert robot is not None
    assert robot.arm.position_commands == []
    assert robot.arm.velocity_commands == []
    cmd = robot.arm.mit_commands[-1]
    assert cmd[:, 2].tolist() == pytest.approx([0.1] * 7)
    assert cmd[:, 0].tolist() == pytest.approx([5.0] * 7)
    assert cmd[:, 1].tolist() == pytest.approx([0.2] * 7)
    assert cmd[:, 4].tolist() == pytest.approx([0.0] * 7)


def test_gravity_compensation_command_shape(monkeypatch: pytest.MonkeyPatch) -> None:
    adapter = DMMotorArm(use_mock_bus=True)
    assert adapter.connect() is True
    assert adapter.write_enable(True) is True
    monkeypatch.setattr(adapter, "compute_gravity_torques", lambda q: [1.0] * 7)
    assert adapter.write_gravity_compensation(damping=0.05) is True
    robot = FakeRobot.last
    assert robot is not None
    cmd = robot.arm.mit_commands[-1]
    assert cmd[:, 0].tolist() == pytest.approx([0.0] * 7)
    assert cmd[:, 1].tolist() == pytest.approx([0.05] * 7)
    assert cmd[:, 4].tolist() == pytest.approx([1.0] * 7)
    assert adapter.get_control_mode() == ControlMode.TORQUE


def test_non_openarm_dof_requires_explicit_motor_specs() -> None:
    with pytest.raises(ValueError, match="motor_specs is required"):
        DMMotorArm(dof=2, use_mock_bus=True)


def test_custom_non_openarm_dof_uses_explicit_metadata() -> None:
    adapter = DMMotorArm(
        dof=2,
        use_mock_bus=True,
        motor_specs=[
            {"name": "shoulder", "type": "DM4310", "send_id": 1, "recv_id": 17},
            {"name": "elbow", "type": "DM4310", "send_id": 2, "recv_id": 18},
        ],
        position_lower=[-0.5, -0.25],
        position_upper=[0.5, 0.25],
        velocity_max=[1.0, 2.0],
        kp=[3.0, 4.0],
        kd=[0.3, 0.4],
    )
    assert adapter.get_dof() == 2
    assert [motor.name for motor in adapter._motor_specs] == ["shoulder", "elbow"]
    assert adapter.get_limits().position_lower == [-0.5, -0.25]
    assert adapter._kp == [3.0, 4.0]
    assert adapter._kd == [0.3, 0.4]

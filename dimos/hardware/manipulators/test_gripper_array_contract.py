# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# See the License for the specific language governing permissions and
# limitations under the License.

"""One-array contract tests: the GRIPPER-SPEC 3.5 regression (850.0, not
722.5, reaching a stubbed xArm SDK) and array-shape conformance."""

from __future__ import annotations

import sys
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.control.hardware_interface import ConnectedHardware
from dimos.hardware.manipulators.spec import ControlMode

# Adapters the registry can build without a vendor SDK or a live device.
_LOCAL_ADAPTERS = ["mock"]


class _FakeXArmSDK:
    """Records exactly what the xArm SDK is handed."""

    def __init__(self, *_: object, **__: object) -> None:
        self.connected = True
        self.gripper_commands: list[float] = []
        self.state = 0
        self.mode = 0
        self.error_code = 0

    def connect(self) -> None: ...
    def set_gripper_enable(self, _on: bool) -> None: ...

    def set_mode(self, _mode: int) -> int:
        return 0

    def set_state(self, _state: int) -> int:
        return 0

    def get_servo_angle(self) -> tuple[int, list[float]]:
        return 0, [0.0] * 7

    def set_servo_angle_j(self, _angles: list[float], **__: object) -> int:
        return 0

    def set_gripper_position(self, position: float, **__: object) -> int:
        self.gripper_commands.append(position)
        return 0

    def get_gripper_position(self) -> tuple[int, float]:
        return 0, 0.0


@pytest.fixture
def xarm_hardware(monkeypatch: pytest.MonkeyPatch) -> tuple[ConnectedHardware, _FakeXArmSDK]:
    """A connected xArm7 with a gripper, wired to a recording fake SDK."""
    monkeypatch.setitem(sys.modules, "xarm.wrapper", SimpleNamespace(XArmAPI=_FakeXArmSDK))
    from dimos.hardware.manipulators.xarm import adapter as xarm_adapter

    monkeypatch.setattr(xarm_adapter, "XArmAPI", _FakeXArmSDK)

    adapter = xarm_adapter.XArmAdapter(address="127.0.0.1", dof=7, gripper_dof=1)
    assert adapter.connect()

    component = HardwareComponent(
        hardware_id="arm",
        hardware_type=HardwareType.MANIPULATOR,
        all_joints=[*make_joints("arm", 7), "arm/gripper"],
        gripper_dof=1,
        adapter_type="xarm",
    )
    return ConnectedHardware(adapter, component), adapter._arm


class TestSection35Regression:
    def test_fully_open_reaches_the_sdk_as_850_not_722_point_5(
        self, xarm_hardware: tuple[ConnectedHardware, _FakeXArmSDK]
    ) -> None:
        hardware, sdk = xarm_hardware

        # Fully open, in the unit the adapter declares.
        fully_open = hardware.adapter.get_limits().position_upper[-1]
        assert fully_open == 850.0

        hardware.write_command({"arm/gripper": fully_open}, ControlMode.SERVO_POSITION)

        assert sdk.gripper_commands == [850.0], (
            f"expected 850.0 at the SDK, got {sdk.gripper_commands} "
            "— 722.5 means the double conversion is back"
        )

    def test_no_intermediate_layer_scales_the_value(
        self, xarm_hardware: tuple[ConnectedHardware, _FakeXArmSDK]
    ) -> None:
        hardware, sdk = xarm_hardware

        for commanded in (0.0, 212.5, 425.0, 850.0):
            hardware.write_command({"arm/gripper": commanded}, ControlMode.SERVO_POSITION)
            assert sdk.gripper_commands[-1] == pytest.approx(commanded)

    def test_arm_and_gripper_travel_in_one_write(
        self, xarm_hardware: tuple[ConnectedHardware, _FakeXArmSDK]
    ) -> None:
        hardware, sdk = xarm_hardware
        commands = {f"arm/joint{i + 1}": 0.1 for i in range(7)}
        commands["arm/gripper"] = 400.0

        assert hardware.write_command(commands, ControlMode.SERVO_POSITION)
        assert sdk.gripper_commands == [400.0]


class TestArrayShapeAcrossAdapters:
    @pytest.mark.parametrize("name", _LOCAL_ADAPTERS)
    @pytest.mark.parametrize("gripper_dof", [0, 1])
    def test_array_lengths_agree(self, name: str, gripper_dof: int) -> None:
        """Reads and limits cover all joints; get_dof() is arm-only."""
        from dimos.hardware.manipulators.registry import adapter_registry

        adapter = adapter_registry.create(name, dof=6, gripper_dof=gripper_dof)
        adapter.connect()
        total = 6 + gripper_dof

        assert adapter.get_dof() == 6, "get_dof() must stay arm-only (R8)"
        assert adapter.get_gripper_dof() == gripper_dof

        assert len(adapter.read_joint_positions()) == total
        assert len(adapter.read_joint_velocities()) == total, "reads stay index-aligned"
        assert len(adapter.read_joint_efforts()) == total

        limits = adapter.get_limits()
        assert len(limits.position_lower) == total
        assert len(limits.position_upper) == total
        assert len(limits.velocity_max) == total

    @pytest.mark.parametrize("name", _LOCAL_ADAPTERS)
    def test_deleted_scalar_gripper_api_is_gone(self, name: str) -> None:
        from dimos.hardware.manipulators.registry import adapter_registry

        adapter = adapter_registry.create(name, dof=6, gripper_dof=1)
        assert not hasattr(adapter, "read_gripper_position")
        assert not hasattr(adapter, "write_gripper_position")

    def test_velocity_writes_carry_all_joints(self) -> None:
        """R4a: velocity writes are symmetric with positions and reads."""
        adapter = MagicMock()
        adapter.set_control_mode.return_value = True
        adapter.write_joint_velocities.return_value = True
        adapter.read_joint_positions.return_value = [0.0] * 7
        component = HardwareComponent(
            hardware_id="arm",
            hardware_type=HardwareType.MANIPULATOR,
            all_joints=[*make_joints("arm", 6), "arm/gripper"],
            gripper_dof=1,
        )
        hardware = ConnectedHardware(adapter, component)

        hardware.write_command({"arm/joint1": 0.5}, ControlMode.VELOCITY)

        sent = adapter.write_joint_velocities.call_args.args[0]
        assert len(sent) == 7

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

import math
from pathlib import Path
from typing import Any

import pytest

from dimos.msgs.sensor_msgs.JointState import JointState
import dimos.teleop.openarm_mini.adapter as adapter_module
from dimos.teleop.openarm_mini.adapter import OpenArmMiniTeleopAdapter
from dimos.teleop.openarm_mini.calibration import (
    FEETECH_POSITION_SPAN,
    OPENARM_MINI_ARM_JOINT_NAMES,
    OpenArmMiniCalibration,
    OpenArmMiniMotorCalibration,
    save_calibration,
)
from dimos.teleop.openarm_mini.config import OpenArmMiniTeleopConfig
from dimos.teleop.openarm_mini.feetech import (
    _calibrated_motor_radians,
    _normalize_motor_position,
)


class _FakeBus:
    def __init__(self, readings: dict[str, float]) -> None:
        self.readings = readings
        self.connected = False
        self.disconnected = False

    def connect(self) -> None:
        self.connected = True

    def disconnect(self) -> None:
        self.disconnected = True

    def read_positions(self) -> dict[str, float]:
        return self.readings


class _FailingBus:
    def connect(self) -> None:
        pass

    def disconnect(self) -> None:
        pass

    def read_positions(self) -> dict[str, float]:
        raise ValueError("read failure")


def _payload(command: object) -> JointState:
    assert command is not None
    payload = command.payload  # type: ignore[attr-defined]
    assert isinstance(payload, JointState)
    return payload


def _calibration(side: str) -> OpenArmMiniCalibration:
    return OpenArmMiniCalibration(
        side=side,
        motors={
            motor_name: OpenArmMiniMotorCalibration(
                id=index + 1,
                homing_offset=0,
                flip=False,
            )
            for index, motor_name in enumerate(OPENARM_MINI_ARM_JOINT_NAMES)
        },
    )


def _write_calibrations(tmp_path: Path) -> tuple[Path, Path]:
    left_path = tmp_path / "left"
    right_path = tmp_path / "right"
    save_calibration(left_path, _calibration("left"))
    save_calibration(right_path, _calibration("right"))
    return left_path, right_path


def _configured_config(
    left_path: Path,
    right_path: Path,
    **kwargs: Any,
) -> OpenArmMiniTeleopConfig:
    return OpenArmMiniTeleopConfig(
        port_left="left-port",
        port_right="right-port",
        left_calibration_path=left_path,
        right_calibration_path=right_path,
        baudrate=123,
        **kwargs,
    )


def _readings() -> dict[str, float]:
    return {
        "joint_1": 1.0,
        "joint_2": 2.0,
        "joint_3": 3.0,
        "joint_4": 4.0,
        "joint_5": 5.0,
        "joint_6": 0.6,
        "joint_7": 0.7,
    }


def test_adapter_loads_calibration_connects_both_buses_and_returns_joint_command(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    left_path, right_path = _write_calibrations(tmp_path)
    buses = {"left": _FakeBus(_readings()), "right": _FakeBus(_readings())}

    def bus_factory(
        side: str,
        port: str,
        calibration: OpenArmMiniCalibration,
        baudrate: int,
    ) -> _FakeBus:
        assert port in ("left-port", "right-port")
        assert baudrate == 123
        assert calibration.side == side
        assert "gripper" not in calibration.motors
        return buses[side]

    monkeypatch.setattr(adapter_module, "OpenArmMiniLeaderReader", bus_factory)

    adapter = OpenArmMiniTeleopAdapter(
        OpenArmMiniTeleopConfig(
            port_left="left-port",
            port_right="right-port",
            left_calibration_path=left_path,
            right_calibration_path=right_path,
            baudrate=123,
        )
    )

    adapter.connect()
    command = adapter.get_current_command()
    adapter.disconnect()

    joint = _payload(command)
    assert joint.name == [
        *[f"openarm_left_joint{i}" for i in range(1, 8)],
        *[f"openarm_right_joint{i}" for i in range(1, 8)],
    ]
    assert buses["left"].connected
    assert buses["right"].connected
    assert buses["left"].disconnected
    assert buses["right"].disconnected


def test_adapter_left_only_connects_left_bus_and_emits_left_joints(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    left_path, right_path = _write_calibrations(tmp_path)
    left_bus = _FakeBus(_readings())
    created_sides: list[str] = []

    def bus_factory(
        side: str,
        port: str,
        calibration: OpenArmMiniCalibration,
        baudrate: int,
    ) -> _FakeBus:
        created_sides.append(side)
        assert side == "left"
        assert port == "left-port"
        assert calibration.side == "left"
        return left_bus

    monkeypatch.setattr(adapter_module, "OpenArmMiniLeaderReader", bus_factory)

    adapter = OpenArmMiniTeleopAdapter(
        _configured_config(left_path, right_path, enabled_sides=("left",))
    )

    adapter.connect()
    command = adapter.get_current_command()
    adapter.disconnect()

    assert created_sides == ["left"]
    joint = _payload(command)
    assert joint.name == [f"openarm_left_joint{i}" for i in range(1, 8)]
    assert left_bus.connected
    assert left_bus.disconnected


def test_config_rejects_invalid_or_duplicate_enabled_sides() -> None:
    with pytest.raises(ValueError, match="at least one side"):
        OpenArmMiniTeleopConfig(enabled_sides=())
    with pytest.raises(ValueError, match="side must be"):
        OpenArmMiniTeleopConfig(enabled_sides=("center",))  # type: ignore[arg-type]
    with pytest.raises(ValueError, match="duplicate"):
        OpenArmMiniTeleopConfig(enabled_sides=("left", "left"))


def test_config_resolves_default_and_configured_target_joint_names() -> None:
    right_target_names = tuple(f"right_arm/openarm_right_joint{i}" for i in range(1, 8))
    config = OpenArmMiniTeleopConfig(target_joint_names_by_side={"right": right_target_names})

    assert config.target_joint_names("left") == tuple(f"openarm_left_joint{i}" for i in range(1, 8))
    assert config.target_joint_names("right") == right_target_names


def test_config_rejects_wrong_target_joint_name_count() -> None:
    with pytest.raises(ValueError, match="exactly 7"):
        OpenArmMiniTeleopConfig(target_joint_names_by_side={"right": ("only_one",)})


def test_adapter_returns_none_without_authority(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    left_path, right_path = _write_calibrations(tmp_path)
    buses = {"left": _FakeBus(_readings()), "right": _FakeBus(_readings())}

    def bus_factory(
        side: str,
        port: str,
        calibration: OpenArmMiniCalibration,
        baudrate: int,
    ) -> _FakeBus:
        return buses[side]

    monkeypatch.setattr(adapter_module, "OpenArmMiniLeaderReader", bus_factory)

    adapter = OpenArmMiniTeleopAdapter(
        _configured_config(left_path, right_path, authority_active=False)
    )

    adapter.connect()
    command = adapter.get_current_command()
    adapter.disconnect()

    assert command is None


def test_adapter_emits_configured_global_target_joint_names(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    left_path, right_path = _write_calibrations(tmp_path)
    right_bus = _FakeBus(_readings())
    target_names = tuple(f"right_arm/openarm_right_joint{i}" for i in range(1, 8))

    def bus_factory(
        side: str,
        port: str,
        calibration: OpenArmMiniCalibration,
        baudrate: int,
    ) -> _FakeBus:
        assert side == "right"
        return right_bus

    monkeypatch.setattr(adapter_module, "OpenArmMiniLeaderReader", bus_factory)

    adapter = OpenArmMiniTeleopAdapter(
        _configured_config(
            left_path,
            right_path,
            enabled_sides=("right",),
            target_joint_names_by_side={"right": target_names},
        )
    )

    adapter.connect()
    command = adapter.get_current_command()
    adapter.disconnect()

    joint = _payload(command)
    assert joint.name == list(target_names)


def test_adapter_rejects_jump_threshold_by_returning_no_command(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    left_path, right_path = _write_calibrations(tmp_path)
    left_bus = _FakeBus(_readings())
    right_bus = _FakeBus(_readings())
    buses = {"left": left_bus, "right": right_bus}

    def bus_factory(
        side: str,
        port: str,
        calibration: OpenArmMiniCalibration,
        baudrate: int,
    ) -> _FakeBus:
        return buses[side]

    monkeypatch.setattr(adapter_module, "OpenArmMiniLeaderReader", bus_factory)

    adapter = OpenArmMiniTeleopAdapter(
        _configured_config(left_path, right_path, max_joint_jump_radians=0.1)
    )

    adapter.connect()
    first = adapter.get_current_command()
    left_bus.readings = {**_readings(), "joint_2": -1.0}
    second = adapter.get_current_command()
    adapter.disconnect()

    assert first is not None
    assert second is None


def test_calibrated_motor_radians_uses_zero_offset_full_encoder_span_and_flip() -> None:
    calibration = OpenArmMiniMotorCalibration(
        id=1,
        homing_offset=2048,
        flip=True,
    )

    assert _calibrated_motor_radians(2200, calibration) == pytest.approx(
        -(2200 - 2048) * math.tau / FEETECH_POSITION_SPAN
    )
    assert _normalize_motor_position(2200, calibration) == pytest.approx(
        _calibrated_motor_radians(2200, calibration)
    )


def test_adapter_clamps_over_limit_sender_side(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    left_path, right_path = _write_calibrations(tmp_path)
    buses = {"left": _FakeBus({**_readings(), "joint_1": 5.0}), "right": _FakeBus(_readings())}

    def bus_factory(
        side: str,
        port: str,
        calibration: OpenArmMiniCalibration,
        baudrate: int,
    ) -> _FakeBus:
        return buses[side]

    monkeypatch.setattr(adapter_module, "OpenArmMiniLeaderReader", bus_factory)

    adapter = OpenArmMiniTeleopAdapter(_configured_config(left_path, right_path))

    adapter.connect()
    command = adapter.get_current_command()
    adapter.disconnect()

    joint = _payload(command)
    assert joint.position[0] == pytest.approx(1.35)


def test_calibration_can_assign_semantic_joint_to_nondefault_motor_id() -> None:
    calibration = OpenArmMiniMotorCalibration(
        id=42,
        homing_offset=1000,
        flip=False,
    )

    assert calibration.id == 42
    assert _calibrated_motor_radians(1001, calibration) == pytest.approx(
        math.tau / FEETECH_POSITION_SPAN
    )


def test_adapter_returns_none_when_bus_reports_invalid_reading(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    left_path, right_path = _write_calibrations(tmp_path)

    def bus_factory(
        side: str,
        port: str,
        calibration: OpenArmMiniCalibration,
        baudrate: int,
    ) -> _FailingBus | _FakeBus:
        return _FailingBus() if side == "left" else _FakeBus(_readings())

    monkeypatch.setattr(adapter_module, "OpenArmMiniLeaderReader", bus_factory)

    adapter = OpenArmMiniTeleopAdapter(_configured_config(left_path, right_path))

    adapter.connect()
    command = adapter.get_current_command()
    adapter.disconnect()

    assert command is None

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

from collections.abc import Iterator, Mapping
from contextlib import contextmanager
import math
from pathlib import Path
from typing import Any

import pytest

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.teleop.openarm_mini import teleop_module
from dimos.teleop.openarm_mini.calibration import (
    FEETECH_POSITION_SPAN,
    OPENARM_MINI_ARM_JOINT_NAMES,
    OpenArmMiniCalibration,
    OpenArmMiniMotorCalibration,
    save_calibration,
)
from dimos.teleop.openarm_mini.feetech import (
    _calibrated_motor_radians,
    _normalize_motor_position,
)
from dimos.teleop.openarm_mini.teleop_module import (
    OpenArmMiniTeleopModule,
    OpenArmMiniTeleopModuleConfig,
)
from dimos.teleop.runtime.types import TeleopCommand


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
    def __init__(self, exc: Exception | None = None) -> None:
        self._exc = exc if exc is not None else ValueError("read failure")

    def connect(self) -> None:
        pass

    def disconnect(self) -> None:
        pass

    def read_positions(self) -> dict[str, float]:
        raise self._exc


def _payload(command: TeleopCommand | None) -> JointState:
    assert command is not None
    payload = command.payload
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
) -> OpenArmMiniTeleopModuleConfig:
    return OpenArmMiniTeleopModuleConfig(
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


def _patch_buses(
    monkeypatch: pytest.MonkeyPatch,
    buses: Mapping[str, _FakeBus | _FailingBus],
) -> list[tuple[str, str, str, int]]:
    created: list[tuple[str, str, str, int]] = []

    def factory(
        side: str,
        port: str,
        calibration: OpenArmMiniCalibration,
        baudrate: int,
    ) -> _FakeBus | _FailingBus:
        created.append((side, port, calibration.side, baudrate))
        return buses[side]

    monkeypatch.setattr(teleop_module, "OpenArmMiniLeaderReader", factory)
    return created


def _module(config: OpenArmMiniTeleopModuleConfig) -> OpenArmMiniTeleopModule:
    return OpenArmMiniTeleopModule(**config.model_dump())


@contextmanager
def _connected_module(
    config: OpenArmMiniTeleopModuleConfig,
) -> Iterator[OpenArmMiniTeleopModule]:
    module = _module(config)
    try:
        module.connect_teleop()
        yield module
    finally:
        module.stop()


def test_teleop_module_loads_calibration_connects_both_buses_and_returns_joint_command(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    left_path, right_path = _write_calibrations(tmp_path)
    buses = {"left": _FakeBus(_readings()), "right": _FakeBus(_readings())}
    created = _patch_buses(monkeypatch, buses)

    with _connected_module(
        OpenArmMiniTeleopModuleConfig(
            port_left="left-port",
            port_right="right-port",
            left_calibration_path=left_path,
            right_calibration_path=right_path,
            baudrate=123,
            enabled_sides=("left", "right"),
        )
    ) as module:
        command = module.get_current_command()

    joint = _payload(command)
    assert joint.name == [
        *[f"openarm_left_joint{i}" for i in range(1, 8)],
        *[f"openarm_right_joint{i}" for i in range(1, 8)],
    ]
    assert created == [("left", "left-port", "left", 123), ("right", "right-port", "right", 123)]
    assert buses["left"].connected
    assert buses["right"].connected
    assert buses["left"].disconnected
    assert buses["right"].disconnected


def test_teleop_module_left_only_connects_left_bus_and_emits_left_joints(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    left_path, right_path = _write_calibrations(tmp_path)
    left_bus = _FakeBus(_readings())
    created = _patch_buses(monkeypatch, {"left": left_bus})

    with _connected_module(
        _configured_config(left_path, right_path, enabled_sides=("left",))
    ) as module:
        command = module.get_current_command()

    assert created == [("left", "left-port", "left", 123)]
    joint = _payload(command)
    assert joint.name == [f"openarm_left_joint{i}" for i in range(1, 8)]
    assert left_bus.connected
    assert left_bus.disconnected


def test_config_rejects_invalid_or_duplicate_enabled_sides() -> None:
    with pytest.raises(ValueError, match="at least 1"):
        OpenArmMiniTeleopModuleConfig(enabled_sides=())
    with pytest.raises(ValueError, match="Input should be 'left' or 'right'"):
        OpenArmMiniTeleopModuleConfig(enabled_sides=("center",))
    with pytest.raises(ValueError, match="duplicate"):
        OpenArmMiniTeleopModuleConfig(enabled_sides=("left", "left"))


def test_config_resolves_default_and_configured_target_joint_names() -> None:
    right_target_names = tuple(f"right_arm/openarm_right_joint{i}" for i in range(1, 8))
    config = OpenArmMiniTeleopModuleConfig(target_joint_names_by_side={"right": right_target_names})

    assert config.target_joint_names("left") == tuple(f"openarm_left_joint{i}" for i in range(1, 8))
    assert config.target_joint_names("right") == right_target_names


def test_config_rejects_wrong_target_joint_name_count() -> None:
    with pytest.raises(ValueError, match="at least 7"):
        OpenArmMiniTeleopModuleConfig(target_joint_names_by_side={"right": ("only_one",)})


def test_teleop_module_returns_none_without_authority(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    left_path, right_path = _write_calibrations(tmp_path)
    buses = {"left": _FakeBus(_readings()), "right": _FakeBus(_readings())}
    _patch_buses(monkeypatch, buses)

    with _connected_module(
        _configured_config(left_path, right_path, authority_active=False)
    ) as module:
        command = module.get_current_command()

    assert command is None


def test_teleop_module_emits_configured_global_target_joint_names(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    left_path, right_path = _write_calibrations(tmp_path)
    right_bus = _FakeBus(_readings())
    target_names = tuple(f"right_arm/openarm_right_joint{i}" for i in range(1, 8))
    created = _patch_buses(monkeypatch, {"right": right_bus})

    with _connected_module(
        _configured_config(
            left_path,
            right_path,
            enabled_sides=("right",),
            target_joint_names_by_side={"right": target_names},
        )
    ) as module:
        command = module.get_current_command()

    joint = _payload(command)
    assert joint.name == list(target_names)
    assert created == [("right", "right-port", "right", 123)]


def test_teleop_module_rejects_jump_threshold_by_returning_no_command(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    left_path, right_path = _write_calibrations(tmp_path)
    left_bus = _FakeBus(_readings())
    right_bus = _FakeBus(_readings())
    buses = {"left": left_bus, "right": right_bus}
    _patch_buses(monkeypatch, buses)

    with _connected_module(
        _configured_config(left_path, right_path, max_joint_jump_radians=0.1)
    ) as module:
        first = module.get_current_command()
        left_bus.readings = {**_readings(), "joint_2": -1.0}
        second = module.get_current_command()

    assert first is not None
    assert second is None


def test_calibrated_motor_radians_uses_zero_offset_full_encoder_span_and_flip() -> None:
    calibration = OpenArmMiniMotorCalibration(
        id=1,
        homing_offset=2048,
        flip=True,
    )

    assert _calibrated_motor_radians(2200, calibration) == pytest.approx(
        -(2200 - 2048) * math.tau / (FEETECH_POSITION_SPAN + 1)
    )
    assert _normalize_motor_position(2200, calibration) == pytest.approx(
        _calibrated_motor_radians(2200, calibration)
    )


def test_teleop_module_clamps_over_limit_sender_side(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    left_path, right_path = _write_calibrations(tmp_path)
    buses = {"left": _FakeBus({**_readings(), "joint_1": 5.0}), "right": _FakeBus(_readings())}
    _patch_buses(monkeypatch, buses)

    with _connected_module(_configured_config(left_path, right_path)) as module:
        command = module.get_current_command()

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
        math.tau / (FEETECH_POSITION_SPAN + 1)
    )


def test_calibrated_motor_radians_wraps_short_way_across_encoder_boundary() -> None:
    calibration = OpenArmMiniMotorCalibration(id=1, homing_offset=4090, flip=False)

    assert _calibrated_motor_radians(3, calibration) == pytest.approx(
        9 * math.tau / (FEETECH_POSITION_SPAN + 1)
    )


def test_teleop_module_returns_none_when_bus_reports_invalid_reading(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    left_path, right_path = _write_calibrations(tmp_path)
    _patch_buses(monkeypatch, {"left": _FailingBus(), "right": _FakeBus(_readings())})

    with _connected_module(_configured_config(left_path, right_path)) as module:
        command = module.get_current_command()

    assert command is None


def test_teleop_module_returns_none_when_bus_read_raises_runtime_error(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    left_path, right_path = _write_calibrations(tmp_path)
    _patch_buses(
        monkeypatch,
        {
            "left": _FailingBus(RuntimeError("Feetech motor read failed")),
            "right": _FakeBus(_readings()),
        },
    )

    with _connected_module(_configured_config(left_path, right_path)) as module:
        command = module.get_current_command()

    assert command is None


def test_teleop_module_publishes_joint_command_payload(mocker: Any) -> None:
    module = OpenArmMiniTeleopModule()
    try:
        joint = JointState({"name": ["openarm_left_joint1"], "position": [0.1]})
        publish = mocker.patch.object(module.joint_command, "publish")

        module.publish_command_payload(joint)

        publish.assert_called_once_with(joint)
    finally:
        module.stop()


def test_teleop_module_rejects_non_joint_command_payload() -> None:
    module = OpenArmMiniTeleopModule()
    try:
        with pytest.raises(TypeError, match="unsupported"):
            module.publish_command_payload("not-a-joint-state")
    finally:
        module.stop()

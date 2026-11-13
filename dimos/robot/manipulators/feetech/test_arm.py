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
import sys
from types import ModuleType

import pytest

from dimos.robot.manipulators.feetech.arm import (
    FeetechArm,
    _read_motor_position,
)
from dimos.robot.manipulators.feetech.calibration import (
    FEETECH_POSITION_SPAN,
    FeetechCalibration,
    FeetechCalibrationError,
    FeetechMotorCalibration,
)
from dimos.robot.manipulators.feetech.robot import FeetechBus, FeetechMotor, FeetechRobot


class _FakePortHandler:
    def __init__(self, port: str) -> None:
        self.port = port
        self.closed = False
        self.baudrate: int | None = None

    def openPort(self) -> bool:
        return True

    def setBaudRate(self, baudrate: int) -> bool:
        self.baudrate = baudrate
        return True

    def closePort(self) -> None:
        self.closed = True


class _FakePacketHandler:
    def __init__(self, port_handler: _FakePortHandler) -> None:
        self.port_handler = port_handler

    def ReadPos(self, motor_id: int) -> tuple[int, int, int]:
        return (1000 + motor_id, 0, 0)


class _PositionPacketHandler:
    def __init__(self, result: int | tuple[int, int, int]) -> None:
        self._result = result

    def ReadPos(self, motor_id: int) -> int | tuple[int, int, int]:
        return self._result


def _install_fake_sdk(monkeypatch: pytest.MonkeyPatch) -> None:
    sdk = ModuleType("scservo_sdk")
    sdk.__dict__.update({"PortHandler": _FakePortHandler, "sms_sts": _FakePacketHandler})
    monkeypatch.setitem(sys.modules, "scservo_sdk", sdk)


def test_arm_reads_each_open_bus_as_calibrated_radians(monkeypatch: pytest.MonkeyPatch) -> None:
    _install_fake_sdk(monkeypatch)

    def bus(side: str, *motors: str) -> FeetechBus:
        return FeetechBus(
            side, tuple(FeetechMotor(m, f"{side}_{m}", lower=-3.0, upper=3.0) for m in motors)
        )

    robot = FeetechRobot(buses=(bus("left", "joint_1", "joint_2"), bus("right", "joint_1")))
    # The fake bus reads 1000 + motor id; offsets put joint_1 at zero and
    # joint_2 a quarter turn (1024 ticks) away, read backwards.
    calibration = FeetechCalibration(
        side="left",
        motors={
            "joint_1": FeetechMotorCalibration(id=1, homing_offset=1001),
            "joint_2": FeetechMotorCalibration(id=7, homing_offset=1007 - 1024, flip=True),
        },
    )
    arm = FeetechArm(robot, {"left": "/dev/fake"}, {"left": calibration})
    arm.connect()
    try:
        readings = arm.read()
    finally:
        arm.disconnect()

    assert list(readings) == ["left"]
    assert readings["left"] == pytest.approx({"joint_1": 0.0, "joint_2": -math.pi / 2})


def test_a_calibration_with_two_motors_on_one_id_is_refused() -> None:
    with pytest.raises(FeetechCalibrationError, match="share one ID"):
        FeetechCalibration(
            side="left",
            motors={
                "joint_1": FeetechMotorCalibration(id=1, homing_offset=0),
                "joint_2": FeetechMotorCalibration(id=1, homing_offset=0),
            },
        )


@pytest.mark.parametrize(
    ("result", "expected"),
    [
        (-1, FEETECH_POSITION_SPAN),
        (FEETECH_POSITION_SPAN + 82, 81),
        ((-82, 0, 0), FEETECH_POSITION_SPAN - 81),
        ((FEETECH_POSITION_SPAN + 1, 0, 0), 0),
    ],
)
def test_read_motor_position_wraps_multi_turn_encoder_ticks(
    result: int | tuple[int, int, int],
    expected: int,
) -> None:
    assert _read_motor_position(_PositionPacketHandler(result), 3) == expected

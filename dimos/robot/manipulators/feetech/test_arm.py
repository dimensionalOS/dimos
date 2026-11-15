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

from dimos.robot.manipulators.feetech import arm as arm_module
from dimos.robot.manipulators.feetech.arm import (
    FeetechArm,
    _read_motor,
    calibrated_motor_radians,
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


class _SequencePacketHandler:
    def __init__(self, replies: list[tuple[int, int, int]]) -> None:
        self._replies = replies

    def ReadPos(self, motor_id: int) -> tuple[int, int, int]:
        return self._replies[-1]


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
    assert _read_motor(_PositionPacketHandler(result), 3)[0] == expected


def test_a_status_flag_keeps_a_plausible_position_and_a_comm_failure_still_fails(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    clock = [100.0]
    monkeypatch.setattr(arm_module.time, "monotonic", lambda: clock[0])
    robot = FeetechRobot(
        buses=(
            FeetechBus("left", (FeetechMotor("joint_1", "left_joint1", lower=-3.0, upper=3.0),)),
        )
    )
    calibration = FeetechCalibration(
        side="left", motors={"joint_1": FeetechMotorCalibration(id=1, homing_offset=0)}
    )
    arm = FeetechArm(robot, {"left": "/dev/fake"}, {"left": calibration})
    replies: list[tuple[int, int, int]] = []
    arm._handlers = {"left": (None, _SequencePacketHandler(replies))}

    def read(reply: tuple[int, int, int]) -> float:
        replies.append(reply)
        clock[0] += 0.01
        return arm.read()["left"]["joint_1"]

    assert read((3771, 0, 0)) == pytest.approx(
        calibrated_motor_radians(3771, calibration.motors["joint_1"])
    )
    # Voltage flag with a valid position: a normal reading, and it is counted.
    assert read((3775, 0, 1)) == pytest.approx(
        calibrated_motor_radians(3775, calibration.motors["joint_1"])
    )
    assert arm.flagged_read_counts() == {("left", 1): {"voltage": 1}}
    # A flagged read far from the last accepted position fails like any bad read.
    with pytest.raises(RuntimeError, match="flagged read"):
        read((1500, 0, 1))
    # A communication failure still fails, flagged or not.
    with pytest.raises(RuntimeError, match="failed with result"):
        read((0, -6, 0))
    # Unflagged reads are never second-guessed, however far they jump.
    assert read((1500, 0, 0)) == pytest.approx(
        calibrated_motor_radians(1500, calibration.motors["joint_1"])
    )

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

"""The Feetech motor layer: serial buses, position reads and calibration.

``FeetechArm`` owns everything that touches the Feetech SDK. It takes plain
:class:`~dimos.robot.manipulators.feetech.robot.FeetechRobot` data, a serial
port and a calibration per bus, and exposes :meth:`FeetechArm.connect`,
:meth:`FeetechArm.read` and :meth:`FeetechArm.disconnect`. The motors are only
read, never commanded.
"""

from __future__ import annotations

from collections.abc import Mapping
import math
from typing import Any

from dimos.robot.manipulators.feetech.calibration import (
    FEETECH_POSITION_SPAN,
    FeetechCalibration,
    FeetechMotorCalibration,
)
from dimos.robot.manipulators.feetech.robot import FeetechRobot

FEETECH_EXTRA = "leader-teleop"
FEETECH_COMM_SUCCESS = 0
_FEETECH_ENCODER_TICKS = FEETECH_POSITION_SPAN + 1


class FeetechDependencyError(ImportError):
    """Raised when the optional Feetech SDK is not installed."""


def missing_dependency_error() -> FeetechDependencyError:
    """The error for a missing Feetech SDK, with the install command."""
    return FeetechDependencyError(
        "Feetech motors need the Feetech SDK. Install it with "
        f"`uv sync --extra {FEETECH_EXTRA}`, or `pip install 'dimos[{FEETECH_EXTRA}]'`."
    )


def _create_sdk_handlers(port: str) -> tuple[Any, Any]:
    """Create Feetech SDK port and packet handlers at the hardware boundary."""
    try:
        from scservo_sdk import (  # type: ignore[import-untyped,import-not-found]
            PortHandler,
            sms_sts,
        )
    except ImportError as exc:
        raise missing_dependency_error() from exc
    port_handler = PortHandler(port)
    return port_handler, sms_sts(port_handler)


def _read_motor_position(packet_handler: Any, motor_id: int) -> int:
    result = packet_handler.ReadPos(motor_id)
    if isinstance(result, tuple):
        values: list[Any] = list(result)
        if not values:
            raise RuntimeError(f"Feetech motor {motor_id} position read returned no data")
        if len(values) >= 3:
            comm_result = values[-2]
            error = values[-1]
            if comm_result != FEETECH_COMM_SUCCESS or error != 0:
                raise RuntimeError(
                    f"Feetech motor {motor_id} position read failed with result {values!r}"
                )
        position = values[0]
    else:
        position = result
    # STS3215 firmware accumulates multi-turn ticks, so reads legitimately
    # leave 0..4095 whenever a joint crosses the encoder boundary; wrap to
    # one turn instead of rejecting.
    return int(position) % _FEETECH_ENCODER_TICKS


def calibrated_motor_radians(raw_position: int, calibration: FeetechMotorCalibration) -> float:
    """Turn a raw encoder reading into the joint angle, in radians.

    Args:
        raw_position: The motor's reading, in encoder ticks (0 to 4095 per turn).
        calibration: The motor's zero offset and direction.
    """
    centered = (raw_position - calibration.homing_offset) % _FEETECH_ENCODER_TICKS
    if centered > _FEETECH_ENCODER_TICKS / 2:
        centered -= _FEETECH_ENCODER_TICKS
    radians = centered * math.tau / _FEETECH_ENCODER_TICKS
    if calibration.flip:
        radians = -radians
    return radians


class FeetechArm:
    """Read every motor on the given Feetech buses as calibrated joint angles.

    Args:
        robot: The buses and the motors on each.
        ports: Serial port per bus name, e.g. ``{"left": "/dev/ttyACM1"}``.
            Only these buses are opened and read.
        calibrations: Calibration per bus name, for every bus in ``ports``.
    """

    def __init__(
        self,
        robot: FeetechRobot,
        ports: Mapping[str, str],
        calibrations: Mapping[str, FeetechCalibration],
    ) -> None:
        for bus in ports:
            robot.bus(bus)
            if bus not in calibrations:
                raise ValueError(f"no calibration for Feetech bus {bus!r}")
        self._robot = robot
        self._ports = dict(ports)
        self._calibrations = {
            bus: calibrations[bus].require_motors(robot.bus(bus).motor_names) for bus in ports
        }
        self._handlers: dict[str, tuple[Any, Any]] = {}

    def connect(self) -> None:
        """Open every bus's serial port at the robot's baudrate."""
        try:
            for bus, port in self._ports.items():
                port_handler, packet_handler = _create_sdk_handlers(port)
                if not port_handler.openPort():
                    raise RuntimeError(f"failed to open Feetech {bus} port {port}")
                if not port_handler.setBaudRate(self._robot.baudrate):
                    port_handler.closePort()
                    raise RuntimeError(
                        f"failed to set Feetech {bus} baudrate {self._robot.baudrate}"
                    )
                self._handlers[bus] = (port_handler, packet_handler)
        except Exception:
            self.disconnect()
            raise

    def read(self) -> dict[str, dict[str, float]]:
        """Every motor's angle, in radians, by bus name, then motor name."""
        if not self._handlers:
            raise RuntimeError("Feetech arm is not connected")
        readings: dict[str, dict[str, float]] = {}
        for bus in self._robot.buses:
            if bus.name not in self._handlers:
                continue
            _, packet_handler = self._handlers[bus.name]
            calibration = self._calibrations[bus.name]
            readings[bus.name] = {
                motor: calibrated_motor_radians(
                    _read_motor_position(packet_handler, calibration.motors[motor].id),
                    calibration.motors[motor],
                )
                for motor in bus.motor_names
            }
        return readings

    def disconnect(self) -> None:
        """Close every open serial port; safe to call when nothing is open."""
        for port_handler, _ in self._handlers.values():
            close_port = getattr(port_handler, "closePort", None)
            if callable(close_port):
                close_port()
        self._handlers = {}


class MockFeetechArm:
    """Fixed readings without any hardware, for ``--simulation`` runs and tests.

    Args:
        robot: The buses and the motors on each; every bus is "read".
        positions: Angle per follower joint, in radians; joints left out
            read 0. Triggers read 0, released.
    """

    def __init__(self, robot: FeetechRobot, positions: Mapping[str, float]) -> None:
        self._readings: dict[str, dict[str, float]] = {}
        for bus in robot.buses:
            readings = {motor.name: float(positions.get(motor.joint, 0.0)) for motor in bus.motors}
            if bus.trigger is not None:
                readings[bus.trigger.name] = 0.0
            self._readings[bus.name] = readings
        self._connected = False

    def connect(self) -> None:
        self._connected = True

    def read(self) -> dict[str, dict[str, float]]:
        if not self._connected:
            raise RuntimeError("mock Feetech arm is not connected")
        return {bus: dict(readings) for bus, readings in self._readings.items()}

    def disconnect(self) -> None:
        self._connected = False

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
import time
from typing import Any

from dimos.robot.manipulators.feetech.calibration import (
    FEETECH_POSITION_SPAN,
    FeetechCalibration,
    FeetechMotorCalibration,
)
from dimos.robot.manipulators.feetech.robot import FeetechRobot
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

FEETECH_EXTRA = "leader-teleop"
FEETECH_COMM_SUCCESS = 0
_FEETECH_ENCODER_TICKS = FEETECH_POSITION_SPAN + 1

# Bits of the servo status byte, as in scservo_sdk's ERRBIT_* constants.
_FEETECH_STATUS_FLAGS = (
    (1, "voltage"),
    (2, "angle_sensor"),
    (4, "overheat"),
    (8, "over_current"),
    (32, "overload"),
)
# Largest believable hand motion, for reads that carry a status flag: 4 turns
# per second (a fast flick of a hand-moved leader joint), plus a small margin
# for encoder noise. At the 100 Hz read loop that allows about 230 ticks
# (20 degrees) per cycle, and proportionally more after skipped cycles.
_MAX_HAND_TICKS_PER_S = 4 * _FEETECH_ENCODER_TICKS
_PLAUSIBILITY_MARGIN_TICKS = 64
# At most one warning per motor and flag set this often, in seconds.
_FLAG_LOG_INTERVAL_S = 5.0


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


def decode_status_flags(status: int) -> list[str]:
    """Names of the bits set in a servo status byte."""
    names = [name for bit, name in _FEETECH_STATUS_FLAGS if status & bit]
    unknown = status & ~sum(bit for bit, _ in _FEETECH_STATUS_FLAGS)
    if unknown:
        names.append(f"unknown(0x{unknown:02x})")
    return names


def _read_motor(packet_handler: Any, motor_id: int) -> tuple[int, int]:
    """One motor's position (wrapped to one turn) and its servo status byte.

    Raises only when the communication failed. A successful reply with a
    status flag still carries a valid position, so it is returned with the
    flag for the caller to judge.
    """
    result = packet_handler.ReadPos(motor_id)
    status = 0
    if isinstance(result, tuple):
        values: list[Any] = list(result)
        if not values:
            raise RuntimeError(f"Feetech motor {motor_id} position read returned no data")
        if len(values) >= 3:
            comm_result = values[-2]
            status = int(values[-1])
            if comm_result != FEETECH_COMM_SUCCESS:
                raise RuntimeError(
                    f"Feetech motor {motor_id} position read failed with result {values!r}"
                )
        position = values[0]
    else:
        position = result
    # STS3215 firmware accumulates multi-turn ticks, so reads legitimately
    # leave 0..4095 whenever a joint crosses the encoder boundary; wrap to
    # one turn instead of rejecting.
    return int(position) % _FEETECH_ENCODER_TICKS, status


class _MotorHealth:
    """Per-motor record of flagged reads and of the last accepted position."""

    def __init__(self) -> None:
        self.last_position: int | None = None
        self.last_time = 0.0
        self.flagged_total: dict[str, int] = {}
        self.flags = 0
        self.logged_this_episode = False
        self._since_log: dict[int, int] = {}
        self._last_log: dict[int, float] = {}


class _StatusMonitor:
    """Counts flagged reads, warns about them at a limited rate, and rejects
    implausible jumps in flagged reads."""

    def __init__(self) -> None:
        self._motors: dict[tuple[str, int], _MotorHealth] = {}

    def counts(self) -> dict[tuple[str, int], dict[str, int]]:
        """Flagged reads so far, per (bus, motor id), by flag name."""
        return {key: dict(h.flagged_total) for key, h in self._motors.items() if h.flagged_total}

    def accept(self, bus: str, motor_id: int, position: int, status: int) -> int:
        """Return the position, or raise if a flagged read is implausible."""
        health = self._motors.setdefault((bus, motor_id), _MotorHealth())
        now = time.monotonic()
        if status:
            self._note_flags(bus, motor_id, health, status, now)
            last = health.last_position
            if last is not None:
                jump = abs(
                    (position - last + _FEETECH_ENCODER_TICKS // 2) % _FEETECH_ENCODER_TICKS
                    - _FEETECH_ENCODER_TICKS // 2
                )
                allowed = _PLAUSIBILITY_MARGIN_TICKS + _MAX_HAND_TICKS_PER_S * (
                    now - health.last_time
                )
                if jump > allowed:
                    raise RuntimeError(
                        f"Feetech {bus} motor {motor_id} flagged read "
                        f"({'+'.join(decode_status_flags(status))}) jumped {jump} ticks "
                        f"from the last accepted position, more than the {allowed:.0f} allowed"
                    )
        elif health.flags:
            if health.logged_this_episode:
                logger.warning(
                    f"Feetech {bus} motor {motor_id}: status flags cleared "
                    f"({'+'.join(decode_status_flags(health.flags))})"
                )
            health.flags = 0
            health.logged_this_episode = False
        health.last_position = position
        health.last_time = now
        return position

    @staticmethod
    def _note_flags(bus: str, motor_id: int, health: _MotorHealth, status: int, now: float) -> None:
        names = decode_status_flags(status)
        for name in names:
            health.flagged_total[name] = health.flagged_total.get(name, 0) + 1
        health.flags = status
        health._since_log[status] = health._since_log.get(status, 0) + 1
        if now - health._last_log.get(status, float("-inf")) < _FLAG_LOG_INTERVAL_S:
            return
        health._last_log[status] = now
        count = health._since_log.pop(status)
        health.logged_this_episode = True
        logger.warning(
            f"Feetech {bus} bus motor {motor_id}: status flag {'+'.join(names)} "
            f"(0x{status:02x}) on {count} reads since the last report; positions are still used"
        )


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
        self._monitor = _StatusMonitor()

    def flagged_read_counts(self) -> dict[tuple[str, int], dict[str, int]]:
        """Reads that carried a servo status flag, per (bus, motor id), by flag name."""
        return self._monitor.counts()

    def _read_motor_position(self, bus: str, packet_handler: Any, motor_id: int) -> int:
        position, status = _read_motor(packet_handler, motor_id)
        return self._monitor.accept(bus, motor_id, position, status)

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
                    self._read_motor_position(
                        bus.name, packet_handler, calibration.motors[motor].id
                    ),
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

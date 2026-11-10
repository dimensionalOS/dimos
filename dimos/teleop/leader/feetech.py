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

"""Feetech serial readers for leader arms, and a mock reader for runs without boards."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import math
from typing import Any, Protocol

from dimos.teleop.leader.calibration import (
    FEETECH_POSITION_SPAN,
    LeaderCalibration,
    LeaderMotorCalibration,
    LeaderSide,
)

LEADER_TELEOP_EXTRA = "leader-teleop"
FEETECH_DEFAULT_BAUDRATE = 1_000_000
FEETECH_COMM_SUCCESS = 0
_FEETECH_ENCODER_TICKS = FEETECH_POSITION_SPAN + 1


class LeaderDependencyError(ImportError):
    """Raised when the optional Feetech SDK dependency is unavailable."""


def missing_dependency_error() -> LeaderDependencyError:
    """The error for a missing Feetech SDK, with the install command."""
    return LeaderDependencyError(
        "Leader teleop requires the Feetech SDK. Install it with "
        f"`uv sync --extra {LEADER_TELEOP_EXTRA}`, or `pip install 'dimos[{LEADER_TELEOP_EXTRA}]'`."
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


class LeaderReader(Protocol):
    """One leader side as the teleop module sees it: calibrated radians per motor."""

    def connect(self) -> None: ...

    def disconnect(self) -> None: ...

    def read_positions(self) -> dict[str, float]: ...


class FeetechLeaderReader:
    """Raw Feetech positions on one serial bus."""

    def __init__(self, port: str, baudrate: int, *, label: str = "Feetech") -> None:
        self._port = port
        self._baudrate = baudrate
        self._label = label
        self._port_handler: Any | None = None
        self._packet_handler: Any | None = None

    def connect(self) -> None:
        port_handler, packet_handler = _create_sdk_handlers(self._port)
        if not port_handler.openPort():
            raise RuntimeError(f"failed to open {self._label} port {self._port}")
        if not port_handler.setBaudRate(self._baudrate):
            port_handler.closePort()
            raise RuntimeError(f"failed to set {self._label} baudrate {self._baudrate}")
        self._port_handler = port_handler
        self._packet_handler = packet_handler

    def disconnect(self) -> None:
        if self._port_handler is None:
            return
        close_port = getattr(self._port_handler, "closePort", None)
        if callable(close_port):
            close_port()
        self._port_handler = None
        self._packet_handler = None

    def read_raw_positions(self, motor_ids_by_name: Mapping[str, int]) -> dict[str, int]:
        if self._packet_handler is None:
            raise RuntimeError(f"{self._label} reader is not connected")
        return {
            motor_name: _read_motor_position(self._packet_handler, motor_id)
            for motor_name, motor_id in motor_ids_by_name.items()
        }


class CalibratedLeaderReader:
    """Calibrated radians for the motors of one leader side on a Feetech bus."""

    def __init__(
        self,
        side: LeaderSide,
        port: str,
        calibration: LeaderCalibration,
        baudrate: int,
        motor_names: Sequence[str],
        *,
        label: str = "leader",
    ) -> None:
        self._calibration = calibration.require_motors(motor_names)
        self._motor_names = tuple(motor_names)
        self._reader = FeetechLeaderReader(port, baudrate, label=f"{label} {side} Feetech")

    def connect(self) -> None:
        self._reader.connect()

    def disconnect(self) -> None:
        self._reader.disconnect()

    def read_positions(self) -> dict[str, float]:
        motor_ids_by_name = {
            motor_name: self._calibration.motors[motor_name].id for motor_name in self._motor_names
        }
        raw_positions = self._reader.read_raw_positions(motor_ids_by_name)
        return {
            motor_name: calibrated_motor_radians(
                raw_positions[motor_name], self._calibration.motors[motor_name]
            )
            for motor_name in self._motor_names
        }


class MockLeaderReader:
    """Fixed calibrated radians, for ``--simulation`` runs and tests."""

    def __init__(self, positions: Mapping[str, float]) -> None:
        self._positions = dict(positions)
        self.connected = False

    def connect(self) -> None:
        self.connected = True

    def disconnect(self) -> None:
        self.connected = False

    def read_positions(self) -> dict[str, float]:
        if not self.connected:
            raise RuntimeError("mock leader reader is not connected")
        return dict(self._positions)


def calibrated_motor_radians(raw_position: int, calibration: LeaderMotorCalibration) -> float:
    """Encoder ticks to radians about the homing offset, flipped where calibrated."""
    centered = (raw_position - calibration.homing_offset) % _FEETECH_ENCODER_TICKS
    if centered > _FEETECH_ENCODER_TICKS / 2:
        centered -= _FEETECH_ENCODER_TICKS
    radians = centered * math.tau / _FEETECH_ENCODER_TICKS
    if calibration.flip:
        radians = -radians
    return radians

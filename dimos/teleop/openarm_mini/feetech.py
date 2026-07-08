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

"""Shared Feetech SDK helpers for OpenArm Mini leader tools and adapters."""

from __future__ import annotations

from collections.abc import Mapping
import math
from typing import Any

from dimos.teleop.openarm_mini.calibration import (
    FEETECH_POSITION_SPAN,
    OPENARM_MINI_ARM_JOINT_NAMES,
    OpenArmMiniCalibration,
    OpenArmMiniMotorCalibration,
)
from dimos.teleop.openarm_mini.config import missing_dependency_error, validate_side


def _create_sdk_handlers(port: str) -> tuple[Any, Any]:
    """Create optional Feetech SDK port and packet handlers at the hardware boundary."""
    try:
        from scservo_sdk import PortHandler, sms_sts  # type: ignore[import-untyped]
    except ImportError as exc:
        raise missing_dependency_error() from exc
    port_handler = PortHandler(port)
    return port_handler, sms_sts(port_handler)


def _read_motor_position(packet_handler: Any, motor_id: int) -> int:
    result = packet_handler.ReadPos(motor_id)
    if isinstance(result, tuple):
        position = result[0]
    else:
        position = result
    return int(position)


class FeetechLeaderReader:
    """Concrete reader for raw Feetech positions on one OpenArm Mini leader bus."""

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
            joint_name: _read_motor_position(self._packet_handler, motor_id)
            for joint_name, motor_id in motor_ids_by_name.items()
        }


class OpenArmMiniLeaderReader:
    """Concrete calibrated reader for one OpenArm Mini leader side."""

    def __init__(
        self,
        side: str,
        port: str,
        calibration: OpenArmMiniCalibration,
        baudrate: int,
    ) -> None:
        validate_side(side)
        self._calibration = calibration
        self._reader = FeetechLeaderReader(
            port,
            baudrate,
            label=f"OpenArm Mini {side} Feetech",
        )

    def connect(self) -> None:
        self._reader.connect()

    def disconnect(self) -> None:
        self._reader.disconnect()

    def read_positions(self) -> dict[str, float]:
        motor_ids_by_name = {
            joint_name: self._calibration.motors[joint_name].id
            for joint_name in OPENARM_MINI_ARM_JOINT_NAMES
        }
        raw_positions = self._reader.read_raw_positions(motor_ids_by_name)
        return {
            joint_name: _calibrated_motor_radians(
                raw_positions[joint_name],
                self._calibration.motors[joint_name],
            )
            for joint_name in OPENARM_MINI_ARM_JOINT_NAMES
        }


def _calibrated_motor_radians(raw_position: int, calibration: OpenArmMiniMotorCalibration) -> float:
    centered = raw_position - calibration.homing_offset
    radians = centered * math.tau / FEETECH_POSITION_SPAN
    if calibration.flip:
        radians = -radians
    return radians


def _normalize_motor_position(raw_position: int, calibration: OpenArmMiniMotorCalibration) -> float:
    """Backward-compatible helper for tests; returns calibrated radians."""
    return _calibrated_motor_radians(raw_position, calibration)

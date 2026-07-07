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

"""OpenArm Mini teleop adapter."""

from __future__ import annotations

from collections.abc import Callable
import importlib
import math
from typing import Protocol, cast

from dimos.teleop.openarm_mini.calibration import (
    FEETECH_POSITION_SPAN,
    OPENARM_MINI_ARM_JOINT_NAMES,
    OpenArmMiniCalibration,
    OpenArmMiniMotorCalibration,
    load_calibration,
)
from dimos.teleop.openarm_mini.config import (
    OpenArmMiniTeleopConfig,
    missing_dependency_error,
    validate_side,
)
from dimos.teleop.openarm_mini.mapping import (
    OpenArmMiniMappingError,
    combine_side_commands,
    map_side_readings,
)
from dimos.teleop.runtime.types import TeleopCommand, TeleopCommandMetadata, TeleopPrimaryOutput


class OpenArmMiniSideBus(Protocol):
    """Runtime operations needed from one OpenArm Mini Feetech bus."""

    def connect(self) -> None: ...

    def disconnect(self) -> None: ...

    def read_positions(self) -> dict[str, float]: ...


OpenArmMiniSideBusFactory = Callable[[str, str, OpenArmMiniCalibration, int], OpenArmMiniSideBus]


class _ScservoPortHandler(Protocol):
    def openPort(self) -> bool: ...

    def setBaudRate(self, baudrate: int) -> bool: ...

    def closePort(self) -> None: ...


class _ScservoPacketHandler(Protocol):
    def ReadPos(self, motor_id: int) -> int | tuple[int, ...]: ...


class _ScservoSdk(Protocol):
    def PortHandler(self, port: str) -> _ScservoPortHandler: ...

    def sms_sts(self, port_handler: _ScservoPortHandler) -> _ScservoPacketHandler: ...


class OpenArmMiniTeleopAdapter:
    """TeleopAdapter implementation for selected OpenArm Mini leader sides."""

    primary_output: TeleopPrimaryOutput = "joint"

    def __init__(
        self,
        config: OpenArmMiniTeleopConfig | None = None,
        *,
        bus_factory: OpenArmMiniSideBusFactory | None = None,
    ) -> None:
        self.config = config if config is not None else OpenArmMiniTeleopConfig()
        self._bus_factory = bus_factory if bus_factory is not None else _default_bus_factory
        self._buses: dict[str, OpenArmMiniSideBus] = {}
        self._previous_positions_by_side: dict[str, dict[str, float]] = {}
        self._connected = False

    def connect(self) -> None:
        """Load calibration and connect configured OpenArm Mini leader sides."""
        if self._connected:
            return
        buses: dict[str, OpenArmMiniSideBus] = {}
        try:
            for side in self.config.sides():
                calibration = load_calibration(self.config.calibration_path(side), side)
                bus = self._bus_factory(
                    side, self.config.port(side), calibration, self.config.baudrate
                )
                bus.connect()
                buses[side] = bus
        except Exception:
            for bus in buses.values():
                bus.disconnect()
            raise
        self._buses = buses
        self._connected = True

    def disconnect(self) -> None:
        """Disconnect any connected Feetech buses."""
        for bus in self._buses.values():
            bus.disconnect()
        self._buses = {}
        self._connected = False
        self._previous_positions_by_side = {}

    def get_current_command(self) -> TeleopCommand | None:
        """Return the current JointState command envelope when readings are valid."""
        if not self._connected or not self.config.authority_active:
            return None

        side_commands = []
        next_previous_positions_by_side: dict[str, dict[str, float]] = {}
        try:
            for side in self.config.sides():
                bus = self._buses[side]
                side_command = map_side_readings(
                    side,
                    bus.read_positions(),
                    target_joint_names=self.config.target_joint_names(side),
                    previous_positions_by_joint=self._previous_positions_by_side.get(side),
                    max_joint_jump_radians=self.config.max_joint_jump_radians,
                )
                side_commands.append(side_command)
                next_previous_positions_by_side[side] = side_command.positions_by_joint
        except (KeyError, OpenArmMiniMappingError, ValueError):
            return None

        self._previous_positions_by_side = next_previous_positions_by_side
        return TeleopCommand(
            metadata=TeleopCommandMetadata(primary_output="joint"),
            joint=combine_side_commands(side_commands),
        )


class _ScservoSideBus:
    """Small Feetech SDK wrapper for OpenArm Mini leader position reads."""

    def __init__(
        self,
        side: str,
        port: str,
        calibration: OpenArmMiniCalibration,
        baudrate: int,
    ) -> None:
        validate_side(side)
        self._side = side
        self._port = port
        self._calibration = calibration
        self._baudrate = baudrate
        self._port_handler: _ScservoPortHandler | None = None
        self._packet_handler: _ScservoPacketHandler | None = None

    def connect(self) -> None:
        sdk = _load_scservo_sdk()
        port_handler = sdk.PortHandler(self._port)
        packet_handler = sdk.sms_sts(port_handler)
        if not port_handler.openPort():
            raise RuntimeError(
                f"failed to open OpenArm Mini {self._side} Feetech port {self._port}"
            )
        if not port_handler.setBaudRate(self._baudrate):
            port_handler.closePort()
            raise RuntimeError(
                f"failed to set OpenArm Mini {self._side} Feetech baudrate {self._baudrate}"
            )
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

    def read_positions(self) -> dict[str, float]:
        if self._packet_handler is None:
            raise RuntimeError(f"OpenArm Mini {self._side} Feetech bus is not connected")
        positions: dict[str, float] = {}
        for joint_name in OPENARM_MINI_ARM_JOINT_NAMES:
            motor_calibration = self._calibration.motors[joint_name]
            raw_position = _read_motor_position(self._packet_handler, motor_calibration.id)
            positions[joint_name] = _calibrated_motor_radians(raw_position, motor_calibration)
        return positions


def _default_bus_factory(
    side: str,
    port: str,
    calibration: OpenArmMiniCalibration,
    baudrate: int,
) -> OpenArmMiniSideBus:
    return _ScservoSideBus(side, port, calibration, baudrate)


def _load_scservo_sdk() -> _ScservoSdk:
    try:
        scservo_sdk = importlib.import_module("scservo_sdk")
    except ImportError as exc:
        raise missing_dependency_error() from exc
    return cast("_ScservoSdk", scservo_sdk)


def _read_motor_position(packet_handler: _ScservoPacketHandler, motor_id: int) -> int:
    result = packet_handler.ReadPos(motor_id)
    if isinstance(result, tuple):
        position = result[0]
    else:
        position = result
    return int(position)


def _calibrated_motor_radians(raw_position: int, calibration: OpenArmMiniMotorCalibration) -> float:
    centered = raw_position - calibration.homing_offset
    radians = centered * math.tau / FEETECH_POSITION_SPAN
    if calibration.flip:
        radians = -radians
    return radians


def _normalize_motor_position(raw_position: int, calibration: OpenArmMiniMotorCalibration) -> float:
    """Backward-compatible helper for tests; returns calibrated radians."""
    return _calibrated_motor_radians(raw_position, calibration)

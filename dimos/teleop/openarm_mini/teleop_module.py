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

"""OpenArm Mini teleop module using the shared teleop runtime."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Annotated, Literal, Self

from pydantic import Field, model_validator

from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.openarm.config import openarm_joints
from dimos.teleop.openarm_mini.calibration import (
    OpenArmMiniCalibrationError,
    OpenArmMiniSide,
    default_calibration_path,
    load_calibration,
)
from dimos.teleop.openarm_mini.feetech import (
    OPENARM_MINI_DEFAULT_BAUDRATE,
    OpenArmMiniDependencyError,
    OpenArmMiniLeaderReader,
)
from dimos.teleop.openarm_mini.mapping import combine_side_commands, map_side_readings
from dimos.teleop.runtime.teleop_module import TeleopModule, TeleopModuleConfig
from dimos.teleop.runtime.types import TeleopCommand
from dimos.utils.logging_config import setup_logger

logger = setup_logger()
OPENARM_MINI_UNCONFIGURED_PORT = ""
OpenArmMiniTargetJointNames = Annotated[tuple[str, ...], Field(min_length=7, max_length=7)]


class OpenArmMiniTeleopModuleConfig(TeleopModuleConfig):
    """Config for OpenArm Mini leader teleoperation.

    Runtime startup is intentionally non-interactive: calibration paths point to
    pre-existing side-specific calibration directories created by the package
    calibration utility.
    """

    # Default to one side so running the concrete module directly only requires
    # one leader calibration/port override. Dual-arm blueprints opt into both.
    backend: Literal["openarm_mini"] = "openarm_mini"
    port_left: str = OPENARM_MINI_UNCONFIGURED_PORT
    port_right: str = OPENARM_MINI_UNCONFIGURED_PORT
    left_calibration_path: Path | None = None
    right_calibration_path: Path | None = None
    baudrate: int = Field(default=OPENARM_MINI_DEFAULT_BAUDRATE, gt=0)
    max_joint_jump_radians: float = 0.75
    authority_active: bool = True
    enabled_sides: tuple[OpenArmMiniSide, ...] = Field(default=("left",), min_length=1)
    target_joint_names_by_side: Mapping[OpenArmMiniSide, OpenArmMiniTargetJointNames] | None = None

    @model_validator(mode="after")
    def _validate_openarm_mini_config(self) -> Self:
        """Validate OpenArm Mini-specific configuration."""
        if len(set(self.enabled_sides)) != len(self.enabled_sides):
            raise ValueError("enabled_sides must not contain duplicate sides")
        return self

    def calibration_path(self, side: OpenArmMiniSide) -> Path:
        """Return the configured or default calibration directory for a side."""
        if side == "left" and self.left_calibration_path is not None:
            return self.left_calibration_path
        if side == "right" and self.right_calibration_path is not None:
            return self.right_calibration_path
        return default_calibration_path(side)

    def port(self, side: OpenArmMiniSide) -> str:
        """Return the configured serial port for a side."""
        port = self.port_left if side == "left" else self.port_right
        if not port:
            raise ValueError(f"port_{side} must be configured for OpenArm Mini teleop")
        return port

    def connection_baudrate(self) -> int:
        """Return the configured Feetech serial baudrate."""
        return self.baudrate

    def sides(self) -> tuple[OpenArmMiniSide, ...]:
        """Return the selected leader sides in runtime order."""
        return self.enabled_sides

    def target_joint_names(self, side: OpenArmMiniSide) -> tuple[str, ...]:
        """Return the follower joint names emitted for a leader side."""
        if self.target_joint_names_by_side is None:
            return tuple(openarm_joints(side))
        configured = self.target_joint_names_by_side.get(side)
        if configured is None:
            return tuple(openarm_joints(side))
        return tuple(configured)


class OpenArmMiniTeleopModule(TeleopModule):
    """Teleop module for OpenArm Mini leader devices."""

    config: OpenArmMiniTeleopModuleConfig  # type: ignore[assignment]
    joint_command: Out[JointState]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._buses: dict[OpenArmMiniSide, OpenArmMiniLeaderReader] = {}
        self._previous_positions_by_side: dict[OpenArmMiniSide, dict[str, float]] = {}
        self._last_read_error: str | None = None
        self._teleop_connected = False

    @property
    def openarm_mini_config(self) -> OpenArmMiniTeleopModuleConfig:
        return self.config

    def connect_teleop(self) -> None:
        if self._teleop_connected:
            return
        openarm_mini = self.openarm_mini_config
        buses: dict[OpenArmMiniSide, OpenArmMiniLeaderReader] = {}
        try:
            baudrate = openarm_mini.connection_baudrate()
            for side in openarm_mini.sides():
                calibration = load_calibration(openarm_mini.calibration_path(side), side)
                bus = OpenArmMiniLeaderReader(
                    side,
                    openarm_mini.port(side),
                    calibration,
                    baudrate,
                )
                bus.connect()
                buses[side] = bus
        except (
            OpenArmMiniCalibrationError,
            OpenArmMiniDependencyError,
            ValueError,
            RuntimeError,
            OSError,
        ):
            for bus in buses.values():
                bus.disconnect()
            raise

        self._buses = buses
        self._teleop_connected = True

    def disconnect_teleop(self) -> None:
        for bus in self._buses.values():
            bus.disconnect()
        self._buses = {}
        self._previous_positions_by_side = {}
        self._last_read_error = None
        self._teleop_connected = False

    def get_current_command(self) -> TeleopCommand | None:
        openarm_mini = self.openarm_mini_config
        if not self._teleop_connected or not openarm_mini.authority_active:
            return None

        side_commands = []
        next_previous_positions_by_side: dict[OpenArmMiniSide, dict[str, float]] = {}
        try:
            for side in openarm_mini.sides():
                bus = self._buses[side]
                side_command = map_side_readings(
                    side,
                    bus.read_positions(),
                    target_joint_names=openarm_mini.target_joint_names(side),
                    previous_positions_by_joint=self._previous_positions_by_side.get(side),
                    max_joint_jump_radians=openarm_mini.max_joint_jump_radians,
                )
                side_commands.append(side_command)
                next_previous_positions_by_side[side] = side_command.positions_by_joint
        except (KeyError, ValueError, RuntimeError, OSError) as exc:
            error_message = str(exc)
            if error_message != self._last_read_error:
                logger.warning(
                    "OpenArm Mini teleop read failed; dropping command: %s",
                    error_message,
                )
                self._last_read_error = error_message
            return None

        self._last_read_error = None
        self._previous_positions_by_side = next_previous_positions_by_side
        return TeleopCommand(payload=combine_side_commands(side_commands))

    def publish_command_payload(self, payload: object) -> None:
        """Publish OpenArm Mini teleop commands to the coordinator joint stream."""
        if not isinstance(payload, JointState):
            raise TypeError(
                f"unsupported OpenArm Mini teleop payload type: {type(payload).__name__}"
            )
        self.joint_command.publish(payload)

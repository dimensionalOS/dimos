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

"""One teleop module for every Feetech leader pair; the robot is in the config."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import replace
from pathlib import Path
import sys
import threading
import time
from typing import Annotated, Literal

if sys.version_info >= (3, 11):
    from typing import Self
else:
    from typing_extensions import Self

from pydantic import Field, model_validator
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.teleop.leader.calibration import (
    LEADER_SIDES,
    LEADER_STATE_DIR,
    LeaderCalibrationError,
    LeaderSide,
    default_calibration_path,
    load_calibration,
)
from dimos.teleop.leader.feetech import (
    FEETECH_DEFAULT_BAUDRATE,
    CalibratedLeaderReader,
    LeaderDependencyError,
    LeaderReader,
    MockLeaderReader,
)
from dimos.teleop.leader.mapping import (
    SideCommand,
    SideMapping,
    combine_side_commands,
    low_pass_alpha,
    map_side_readings,
    smooth_positions,
)
from dimos.teleop.webxr.controller_types import Buttons
from dimos.utils.logging_config import setup_logger

logger = setup_logger()
UNCONFIGURED_PORT = ""
# The trajectory task treats a JointState in this frame as a streamed jog that
# expires shortly after the last message, so the leader's claim ends when it stops.
GUARDED_JOINT_FRAME_ID = "dimos/guarded-joint"
ReaderKind = Literal["feetech", "mock"]


class LeaderTeleopModuleConfig(ModuleConfig):
    """What a leader pair reads and which follower joints it drives.

    Startup is non-interactive: calibration directories must already exist, written
    by ``dimos hardware feetech-leader calibrate``.
    """

    # Follower joints and gripper per leader side; the robot folder supplies these.
    mappings: dict[LeaderSide, SideMapping]
    tick_period_s: float = Field(default=0.02, gt=0.0)
    port_left: str = UNCONFIGURED_PORT
    port_right: str = UNCONFIGURED_PORT
    baudrate: int = Field(default=FEETECH_DEFAULT_BAUDRATE, gt=0)
    calibration_root: Path = LEADER_STATE_DIR
    left_calibration_path: Path | None = None
    right_calibration_path: Path | None = None
    # None is for raw input to a downstream authority/realignment guard.
    max_joint_jump_radians: float | None = Field(default=0.75, gt=0.0, allow_inf_nan=False)
    authority_active: bool = True
    # Unset: the sides whose serial port is configured drive the follower.
    enabled_sides: Annotated[tuple[LeaderSide, ...], Field(min_length=1)] | None = None
    # First-order low-pass on the mapped arm targets; None passes readings through.
    command_filter_cutoff_hz: float | None = None
    guarded: bool = True
    reader: ReaderKind = "feetech"
    # Calibrated radians the mock reader reports, per motor; unlisted motors read zero.
    mock_positions: dict[str, float] = Field(default_factory=dict)

    @model_validator(mode="after")
    def _validate(self) -> Self:
        if self.enabled_sides is not None and len(set(self.enabled_sides)) != len(
            self.enabled_sides
        ):
            raise ValueError("enabled_sides must not contain duplicate sides")
        if self.command_filter_cutoff_hz is not None and self.command_filter_cutoff_hz <= 0.0:
            raise ValueError("command_filter_cutoff_hz must be positive or None")
        for side in self._configured_sides():
            if side not in self.mappings:
                raise ValueError(f"no mapping configured for the {side} leader")
        return self

    def calibration_path(self, side: LeaderSide) -> Path:
        if side == "left" and self.left_calibration_path is not None:
            return self.left_calibration_path
        if side == "right" and self.right_calibration_path is not None:
            return self.right_calibration_path
        return default_calibration_path(side, self.calibration_root)

    def port(self, side: LeaderSide) -> str:
        port = self.port_left if side == "left" else self.port_right
        if not port:
            raise ValueError(f"port_{side} must be configured for leader teleop")
        return port

    def _configured_sides(self) -> tuple[LeaderSide, ...]:
        if self.enabled_sides is not None:
            return self.enabled_sides
        if self.reader == "mock":
            return tuple(side for side in LEADER_SIDES if side in self.mappings)
        return tuple(
            side for side in LEADER_SIDES if (self.port_left if side == "left" else self.port_right)
        )

    def sides(self) -> tuple[LeaderSide, ...]:
        """Leader sides in runtime order; the ports given select them.

        Ports arrive on the command line, so a config without any is valid
        until the module connects.
        """
        sides = self._configured_sides()
        if not sides:
            raise ValueError("configure port_left and/or port_right (or enabled_sides)")
        return sides


class LeaderTeleopModule(Module):
    """Follower joint targets and gripper openings (1 = open) read from leader arms."""

    config: LeaderTeleopModuleConfig  # type: ignore[assignment]

    leader_joint_command: Out[JointState]
    left_gripper_command: Out[Float32]
    right_gripper_command: Out[Float32]
    # Optional: a teleop_buttons producer (Joy-Con, WebXR) pauses and resumes following.
    teleop_buttons: In[Buttons]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._authority = bool(self.leader_config.authority_active)
        self._menu_was_down = False
        self._readers: dict[LeaderSide, LeaderReader] = {}
        self._previous_positions_by_side: dict[LeaderSide, dict[str, float]] = {}
        self._filtered_by_joint: dict[str, float] = {}
        self._last_read_error: str | None = None
        self._connected = False
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def leader_config(self) -> LeaderTeleopModuleConfig:
        return self.config

    @rpc
    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            logger.warning("leader teleop polling worker is already running")
            return
        super().start()
        self._stop_event.clear()
        try:
            self.connect_teleop()
            if getattr(self.teleop_buttons, "transport", None) is not None:
                self.register_disposable(
                    Disposable(self.teleop_buttons.subscribe(self._on_buttons))
                )
            self._thread = threading.Thread(target=self._run_loop, daemon=True)
            self._thread.start()
        except Exception:
            self._stop_event.set()
            self._thread = None
            self.disconnect_teleop()
            raise

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(DEFAULT_THREAD_JOIN_TIMEOUT)
            if self._thread.is_alive():
                logger.warning("leader teleop polling worker did not exit in time")
            else:
                self._thread = None
        self.disconnect_teleop()
        super().stop()

    @rpc
    def set_authority(self, active: bool) -> bool:
        """Publish leader targets (True) or leave the follower where it is (False)."""
        self._authority = bool(active)
        logger.info("leader teleop authority %s", "on" if self._authority else "off")
        return self._authority

    @rpc
    def get_authority(self) -> bool:
        return self._authority

    def _on_buttons(self, msg: Buttons) -> None:
        """The right menu button (Joy-Con plus) toggles authority on its rising edge."""
        down = bool(msg.right_menu)
        if down and not self._menu_was_down:
            self.set_authority(not self._authority)
        self._menu_was_down = down

    def connect_teleop(self) -> None:
        if self._connected:
            return
        config = self.leader_config
        readers: dict[LeaderSide, LeaderReader] = {}
        try:
            for side in config.sides():
                reader = self._reader(side)
                reader.connect()
                readers[side] = reader
        except (LeaderCalibrationError, LeaderDependencyError, ValueError, RuntimeError, OSError):
            for reader in readers.values():
                reader.disconnect()
            raise
        self._readers = readers
        self._connected = True

    def disconnect_teleop(self) -> None:
        for reader in self._readers.values():
            reader.disconnect()
        self._readers = {}
        self._previous_positions_by_side = {}
        self._filtered_by_joint = {}
        self._last_read_error = None
        self._connected = False

    def _reader(self, side: LeaderSide) -> LeaderReader:
        config = self.leader_config
        motor_names = config.mappings[side].motor_names()
        if config.reader == "mock":
            return MockLeaderReader(
                {motor: config.mock_positions.get(motor, 0.0) for motor in motor_names}
            )
        calibration = load_calibration(config.calibration_path(side), side, motor_names)
        return CalibratedLeaderReader(
            side, config.port(side), calibration, config.baudrate, motor_names
        )

    def get_current_commands(self) -> list[SideCommand] | None:
        """Read every enabled leader once; None when a read fails or authority is off."""
        config = self.leader_config
        if not self._connected or not self._authority:
            return None
        commands: list[SideCommand] = []
        previous: dict[LeaderSide, dict[str, float]] = {}
        try:
            for side in config.sides():
                command = map_side_readings(
                    side,
                    config.mappings[side],
                    self._readers[side].read_positions(),
                    previous_positions_by_joint=self._previous_positions_by_side.get(side),
                    max_joint_jump_radians=config.max_joint_jump_radians,
                )
                commands.append(command)
                previous[side] = command.positions_by_joint
        except (KeyError, ValueError, RuntimeError, OSError) as exc:
            message = str(exc)
            if message != self._last_read_error:
                logger.warning("leader read failed; dropping command: %s", message)
                self._last_read_error = message
            return None
        self._last_read_error = None
        self._previous_positions_by_side = previous
        return self._filtered(commands)

    def _filtered(self, commands: list[SideCommand]) -> list[SideCommand]:
        """Low-pass the mapped arm targets; the jump guard already saw the raw values."""
        config = self.leader_config
        if config.command_filter_cutoff_hz is None:
            return commands
        alpha = low_pass_alpha(config.command_filter_cutoff_hz, config.tick_period_s)
        filtered: list[SideCommand] = []
        for command in commands:
            positions = smooth_positions(self._filtered_by_joint, command.positions_by_joint, alpha)
            self._filtered_by_joint.update(positions)
            filtered.append(replace(command, positions_by_joint=positions))
        return filtered

    def get_current_command(self) -> JointState | None:
        commands = self.get_current_commands()
        return None if commands is None else self._joint_state(commands)

    def _joint_state(self, commands: list[SideCommand]) -> JointState:
        frame_id = GUARDED_JOINT_FRAME_ID if self.leader_config.guarded else ""
        return combine_side_commands(commands, frame_id=frame_id)

    def tick(self) -> None:
        """One polling iteration: arm joints as one JointState, grippers per side."""
        if self._stop_event.is_set():
            return
        commands = self.get_current_commands()
        if commands is None or self._stop_event.is_set():
            return
        self.leader_joint_command.publish(self._joint_state(commands))
        outputs: Mapping[LeaderSide, Out[Float32]] = {
            "left": self.left_gripper_command,
            "right": self.right_gripper_command,
        }
        for command in commands:
            if command.gripper_opening is not None:
                outputs[command.side].publish(Float32(data=command.gripper_opening))

    def _run_loop(self) -> None:
        next_tick = time.monotonic()
        while not self._stop_event.is_set():
            try:
                self.tick()
            except Exception:
                logger.exception("Unexpected leader teleop polling worker error")
            next_tick += self.leader_config.tick_period_s
            self._stop_event.wait(max(0.0, next_tick - time.monotonic()))

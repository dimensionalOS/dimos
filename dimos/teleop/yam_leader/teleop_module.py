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

"""YAM leader teleop module: two Feetech leader arms drive the dual OpenYAM."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import replace
from pathlib import Path
import sys
import threading
import time
from typing import Annotated, Protocol

if sys.version_info >= (3, 11):
    from typing import Self
else:
    from typing_extensions import Self

from pydantic import Field, model_validator

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT, STATE_DIR
from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.spec.utils import Spec
from dimos.teleop.openarm_mini.calibration import (
    OpenArmMiniCalibration,
    OpenArmMiniCalibrationError,
    load_calibration,
)
from dimos.teleop.openarm_mini.feetech import (
    OPENARM_MINI_DEFAULT_BAUDRATE,
    FeetechLeaderReader,
    OpenArmMiniDependencyError,
    OpenArmMiniLeaderReader,
)
from dimos.teleop.yam_leader.mapping import (
    YAM_LEADER_SIDES,
    YamLeaderSide,
    YamLeaderSideCommand,
    combine_side_commands,
    low_pass_alpha,
    map_side_readings,
    smooth_positions,
    streamed_trajectory,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

YAM_LEADER_STATE_DIR = STATE_DIR / "teleop" / "yam_leader"
YAM_LEADER_UNCONFIGURED_PORT = ""


class YamLeaderControlSpec(Spec, Protocol):
    """Coordinator operation the leader uses to stream targets as trajectories."""

    def execute_trajectory(self, trajectory: JointTrajectory) -> TrajectoryExecutionResult: ...


def default_calibration_path(side: YamLeaderSide) -> Path:
    """Default calibration directory for a YAM leader side."""
    return YAM_LEADER_STATE_DIR / side


class YamLeaderReader(OpenArmMiniLeaderReader):
    """The OpenArm Mini calibrated reader with a YAM label in its errors."""

    def __init__(
        self,
        side: YamLeaderSide,
        port: str,
        calibration: OpenArmMiniCalibration,
        baudrate: int,
    ) -> None:
        super().__init__(side, port, calibration, baudrate)
        self._reader = FeetechLeaderReader(port, baudrate, label=f"YAM {side} leader Feetech")


class YamLeaderTeleopModuleConfig(ModuleConfig):
    """YAM leader teleop config; calibrate first with ``dimos hardware feetech-leader calibrate``."""

    tick_period_s: float = Field(default=0.02, gt=0.0)
    port_left: str = YAM_LEADER_UNCONFIGURED_PORT
    port_right: str = YAM_LEADER_UNCONFIGURED_PORT
    left_calibration_path: Path | None = None
    right_calibration_path: Path | None = None
    baudrate: int = Field(default=OPENARM_MINI_DEFAULT_BAUDRATE, gt=0)
    max_joint_jump_radians: float = Field(default=0.75, gt=0.0)
    # Trigger angle from released to fully squeezed; measure it with the joint TUI.
    gripper_travel_radians: float = Field(default=1.0, gt=0.0)
    authority_active: bool = True
    enabled_sides: Annotated[tuple[YamLeaderSide, ...], Field(min_length=1)] | None = None
    # First-order low-pass on the mapped arm targets; None passes readings through.
    command_filter_cutoff_hz: float | None = 6.0
    # Two-point trajectory from the last target over this horizon; None streams joint_command.
    trajectory_horizon_s: float | None = 0.04

    @model_validator(mode="after")
    def _validate_sides(self) -> Self:
        if self.enabled_sides is not None and len(set(self.enabled_sides)) != len(
            self.enabled_sides
        ):
            raise ValueError("enabled_sides must not contain duplicate sides")
        if self.command_filter_cutoff_hz is not None and self.command_filter_cutoff_hz <= 0.0:
            raise ValueError("command_filter_cutoff_hz must be positive or None")
        if self.trajectory_horizon_s is not None and self.trajectory_horizon_s <= 0.0:
            raise ValueError("trajectory_horizon_s must be positive or None")
        return self

    def calibration_path(self, side: YamLeaderSide) -> Path:
        if side == "left" and self.left_calibration_path is not None:
            return self.left_calibration_path
        if side == "right" and self.right_calibration_path is not None:
            return self.right_calibration_path
        return default_calibration_path(side)

    def port(self, side: YamLeaderSide) -> str:
        port = self.port_left if side == "left" else self.port_right
        if not port:
            raise ValueError(f"port_{side} must be configured for YAM leader teleop")
        return port

    def sides(self) -> tuple[YamLeaderSide, ...]:
        """Leader sides in runtime order; the ports given select them."""
        if self.enabled_sides is not None:
            return self.enabled_sides
        configured = tuple(
            side
            for side in YAM_LEADER_SIDES
            if (self.port_left if side == "left" else self.port_right)
        )
        if not configured:
            raise ValueError("configure port_left and/or port_right (or enabled_sides)")
        return configured


class YamLeaderTeleopModule(Module):
    """Follower joint targets and gripper openings (1 = open) read from the YAM leaders."""

    config: YamLeaderTeleopModuleConfig  # type: ignore[assignment]

    joint_command: Out[JointState]
    left_gripper_command: Out[Float32]
    right_gripper_command: Out[Float32]

    _control: YamLeaderControlSpec | None = None

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._buses: dict[YamLeaderSide, YamLeaderReader] = {}
        self._previous_positions_by_side: dict[YamLeaderSide, dict[str, float]] = {}
        self._filtered_by_joint: dict[str, float] = {}
        self._last_sent_by_joint: dict[str, float] = {}
        self._last_read_error: str | None = None
        self._last_stream_error: str | None = None
        self._teleop_connected = False
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def leader_config(self) -> YamLeaderTeleopModuleConfig:
        return self.config

    @rpc
    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            logger.warning("YAM leader polling worker is already running")
            return
        super().start()
        self._stop_event.clear()
        try:
            self.connect_teleop()
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
                logger.warning("YAM leader polling worker did not exit in time")
            else:
                self._thread = None
        self.disconnect_teleop()
        super().stop()

    def connect_teleop(self) -> None:
        if self._teleop_connected:
            return
        config = self.leader_config
        buses: dict[YamLeaderSide, YamLeaderReader] = {}
        try:
            for side in config.sides():
                calibration = load_calibration(config.calibration_path(side), side)
                bus = YamLeaderReader(side, config.port(side), calibration, config.baudrate)
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
        self._filtered_by_joint = {}
        self._last_sent_by_joint = {}
        self._last_read_error = None
        self._last_stream_error = None
        self._teleop_connected = False

    def get_current_commands(self) -> list[YamLeaderSideCommand] | None:
        """Read every enabled leader once; None when a read fails or authority is off."""
        config = self.leader_config
        if not self._teleop_connected or not config.authority_active:
            return None
        commands: list[YamLeaderSideCommand] = []
        previous: dict[YamLeaderSide, dict[str, float]] = {}
        try:
            for side in config.sides():
                command = map_side_readings(
                    side,
                    self._buses[side].read_positions(),
                    gripper_travel_radians=config.gripper_travel_radians,
                    previous_positions_by_joint=self._previous_positions_by_side.get(side),
                    max_joint_jump_radians=config.max_joint_jump_radians,
                )
                commands.append(command)
                previous[side] = command.positions_by_joint
        except (KeyError, ValueError, RuntimeError, OSError) as exc:
            message = str(exc)
            if message != self._last_read_error:
                logger.warning("YAM leader read failed; dropping command: %s", message)
                self._last_read_error = message
            return None
        self._last_read_error = None
        self._previous_positions_by_side = previous
        return self._filtered(commands)

    def _filtered(self, commands: list[YamLeaderSideCommand]) -> list[YamLeaderSideCommand]:
        """Low-pass the mapped arm targets; the jump guard already saw the raw values."""
        config = self.leader_config
        if config.command_filter_cutoff_hz is None:
            return commands
        alpha = low_pass_alpha(config.command_filter_cutoff_hz, config.tick_period_s)
        filtered: list[YamLeaderSideCommand] = []
        for command in commands:
            positions = smooth_positions(self._filtered_by_joint, command.positions_by_joint, alpha)
            self._filtered_by_joint.update(positions)
            filtered.append(replace(command, positions_by_joint=positions))
        return filtered

    def stream_trajectory(self, joint_state: JointState) -> bool:
        """Send the arm targets as a trajectory; False when joint_command should carry them."""
        control = self._control
        horizon = self.leader_config.trajectory_horizon_s
        if control is None or horizon is None:
            return False
        positions = dict(zip(joint_state.name, joint_state.position, strict=True))
        last_sent = self._last_sent_by_joint or None
        result = control.execute_trajectory(streamed_trajectory(last_sent, positions, horizon))
        if last_sent is not None and not _accepted(result):
            # The anchor no longer matches the measured start; slew from it this tick.
            result = control.execute_trajectory(streamed_trajectory(None, positions, horizon))
        if _accepted(result):
            self._last_sent_by_joint = positions
            self._last_stream_error = None
            return True
        self._last_sent_by_joint = {}
        message = "no result" if result is None else f"{result.status.name}: {result.message}"
        if message != self._last_stream_error:
            logger.warning("YAM leader trajectory rejected; streaming joint_command: %s", message)
            self._last_stream_error = message
        return False

    def get_current_command(self) -> JointState | None:
        commands = self.get_current_commands()
        return None if commands is None else combine_side_commands(commands)

    def tick(self) -> None:
        """One polling iteration: arm joints as one JointState, grippers per side."""
        if self._stop_event.is_set():
            return
        commands = self.get_current_commands()
        if commands is None or self._stop_event.is_set():
            return
        joint_state = combine_side_commands(commands)
        if not self.stream_trajectory(joint_state):
            self.joint_command.publish(joint_state)
        outputs: Mapping[YamLeaderSide, Out[Float32]] = {
            "left": self.left_gripper_command,
            "right": self.right_gripper_command,
        }
        for command in commands:
            outputs[command.side].publish(Float32(data=command.gripper_opening))

    def _run_loop(self) -> None:
        next_tick = time.monotonic()
        while not self._stop_event.is_set():
            try:
                self.tick()
            except Exception:
                logger.exception("Unexpected YAM leader polling worker error")
            next_tick += self.leader_config.tick_period_s
            self._stop_event.wait(max(0.0, next_tick - time.monotonic()))


def _accepted(result: TrajectoryExecutionResult | None) -> bool:
    return result is not None and result.status == TrajectoryExecutionStatus.ACCEPTED

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

"""OpenArm Mini teleop module."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import replace
from pathlib import Path
import sys
import threading
import time
from typing import Annotated, Any, Protocol

if sys.version_info >= (3, 11):
    from typing import Self
else:
    from typing_extensions import Self

from pydantic import Field, model_validator

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.rpc_client import RPCClient
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.robot.manipulators.openarm.config import openarm_urdf_joints
from dimos.spec.utils import Spec
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
from dimos.teleop.openarm_mini.mapping import (
    OpenArmMiniSideCommand,
    combine_side_commands,
    low_pass_alpha,
    map_side_readings,
    smooth_positions,
    streamed_trajectory,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()
OPENARM_MINI_UNCONFIGURED_PORT = ""
OpenArmMiniTargetJointNames = Annotated[tuple[str, ...], Field(min_length=7, max_length=7)]
# Consecutive trajectory rejections before the module falls back to the joint_command stream.
_STREAM_FAILURES_BEFORE_FALLBACK = 25
TrajectoryCall = Callable[[JointTrajectory, Callable[[Any], None]], None]


class OpenArmMiniControlSpec(Spec, Protocol):
    """The coordinator RPC the leader streams trajectories into."""

    def execute_trajectory(self, trajectory: JointTrajectory) -> TrajectoryExecutionResult: ...


class OpenArmMiniTeleopModuleConfig(ModuleConfig):
    """Config for OpenArm Mini leader teleoperation.

    Runtime startup is intentionally non-interactive: calibration paths point to
    pre-existing side-specific calibration directories created by the package
    calibration utility.
    """

    tick_period_s: float = Field(default=0.02, gt=0.0)
    port_left: str = OPENARM_MINI_UNCONFIGURED_PORT
    port_right: str = OPENARM_MINI_UNCONFIGURED_PORT
    left_calibration_path: Path | None = None
    right_calibration_path: Path | None = None
    baudrate: int = Field(default=OPENARM_MINI_DEFAULT_BAUDRATE, gt=0)
    max_joint_jump_radians: float = 0.75
    # Trigger angle from released to fully squeezed; read it off the calibrate live readout.
    # The per-side values override the shared one for leaders whose triggers differ.
    gripper_travel_radians: float = Field(default=1.0, gt=0.0)
    gripper_travel_radians_by_side: Mapping[OpenArmMiniSide, Annotated[float, Field(gt=0.0)]] = (
        Field(default_factory=dict)
    )
    authority_active: bool = True
    # First-order low-pass on the mapped arm targets; None passes readings through.
    command_filter_cutoff_hz: float | None = 6.0
    # Each target goes out as a two-point trajectory from the last one over this
    # horizon, through the coordinator's execute_trajectory RPC; None streams
    # joint_command one target at a time. Off: the trajectory task restarts its
    # clock on every replacement, which on Alfred felt worse than the stream.
    trajectory_horizon_s: float | None = None
    # The coordinator to stream into when no module in the blueprint provides it.
    coordinator_name: str = "ControlCoordinator"
    # Unset: the sides whose serial port is configured drive the follower.
    enabled_sides: Annotated[tuple[OpenArmMiniSide, ...], Field(min_length=1)] | None = None
    target_joint_names_by_side: Mapping[OpenArmMiniSide, OpenArmMiniTargetJointNames] | None = None

    @model_validator(mode="after")
    def _validate_openarm_mini_config(self) -> Self:
        """Validate OpenArm Mini-specific configuration."""
        if self.enabled_sides is not None and len(set(self.enabled_sides)) != len(
            self.enabled_sides
        ):
            raise ValueError("enabled_sides must not contain duplicate sides")
        if self.command_filter_cutoff_hz is not None and self.command_filter_cutoff_hz <= 0.0:
            raise ValueError("command_filter_cutoff_hz must be positive or None")
        if self.trajectory_horizon_s is not None and self.trajectory_horizon_s <= 0.0:
            raise ValueError("trajectory_horizon_s must be positive or None")
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

    def gripper_travel(self, side: OpenArmMiniSide) -> float:
        """Return the trigger travel for a side, the shared value unless overridden."""
        return self.gripper_travel_radians_by_side.get(side, self.gripper_travel_radians)

    def connection_baudrate(self) -> int:
        """Return the configured Feetech serial baudrate."""
        return self.baudrate

    def sides(self) -> tuple[OpenArmMiniSide, ...]:
        """Return the selected leader sides in runtime order."""
        if self.enabled_sides is not None:
            return self.enabled_sides
        configured: list[OpenArmMiniSide] = []
        if self.port_left:
            configured.append("left")
        if self.port_right:
            configured.append("right")
        if not configured:
            raise ValueError("configure port_left and/or port_right (or enabled_sides)")
        return tuple(configured)

    def target_joint_names(self, side: OpenArmMiniSide) -> tuple[str, ...]:
        """Return the follower joint names emitted for a leader side."""
        if self.target_joint_names_by_side is None:
            return tuple(openarm_urdf_joints(side))
        configured = self.target_joint_names_by_side.get(side)
        if configured is None:
            return tuple(openarm_urdf_joints(side))
        return tuple(configured)


class OpenArmMiniTeleopModule(Module):
    """Teleop module for OpenArm Mini leader devices."""

    config: OpenArmMiniTeleopModuleConfig  # type: ignore[assignment]
    joint_command: Out[JointState]
    left_gripper_command: Out[Float32]
    right_gripper_command: Out[Float32]

    # Injected when the coordinator shares the blueprint; otherwise named by config.
    _control: OpenArmMiniControlSpec | None = None

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._buses: dict[OpenArmMiniSide, OpenArmMiniLeaderReader] = {}
        self._previous_positions_by_side: dict[OpenArmMiniSide, dict[str, float]] = {}
        self._filtered_by_joint: dict[str, float] = {}
        self._last_sent_by_joint: dict[str, float] = {}
        self._stream_failures = 0
        self._stream_fallback = False
        self._last_read_error: str | None = None
        self._last_stream_error: str | None = None
        self._trajectory_call: TrajectoryCall | None = None
        self._trajectory_client: RPCClient | None = None
        self._teleop_connected = False
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def openarm_mini_config(self) -> OpenArmMiniTeleopModuleConfig:
        return self.config

    @rpc
    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            logger.warning("OpenArm Mini teleop polling worker is already running")
            return
        super().start()
        self._stop_event.clear()
        try:
            self.connect_teleop()
            self._connect_trajectory_stream()
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
                logger.warning("OpenArm Mini teleop polling worker did not exit in time")
            else:
                self._thread = None
        self.disconnect_teleop()
        super().stop()

    def _connect_trajectory_stream(self) -> None:
        """Open a non-blocking RPC link to the coordinator's trajectory task."""
        if (
            self.openarm_mini_config.trajectory_horizon_s is None
            or self._trajectory_call is not None
        ):
            return
        remote_name = getattr(self._control, "remote_name", None) or (
            self.openarm_mini_config.coordinator_name
        )
        client = RPCClient(None, None, remote_name, ["execute_trajectory"])
        rpc_name = f"{remote_name}/execute_trajectory"

        def call(trajectory: JointTrajectory, on_result: Callable[[Any], None]) -> None:
            client.rpc.call(rpc_name, ((trajectory,), {}), on_result)

        self._trajectory_client = client
        self._trajectory_call = call

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
        self._filtered_by_joint = {}
        self._last_sent_by_joint = {}
        self._stream_failures = 0
        self._stream_fallback = False
        self._last_read_error = None
        self._last_stream_error = None
        if self._trajectory_client is not None:
            self._trajectory_client.stop_rpc_client()
        self._trajectory_client = None
        self._trajectory_call = None
        self._teleop_connected = False

    def get_current_commands(self) -> list[OpenArmMiniSideCommand] | None:
        """Read every enabled leader once; None when a read fails or authority is off."""
        openarm_mini = self.openarm_mini_config
        if not self._teleop_connected or not openarm_mini.authority_active:
            return None

        side_commands: list[OpenArmMiniSideCommand] = []
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
                    gripper_travel_radians=openarm_mini.gripper_travel(side),
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
        return self._filtered(side_commands)

    def _filtered(
        self, side_commands: list[OpenArmMiniSideCommand]
    ) -> list[OpenArmMiniSideCommand]:
        """Low-pass the mapped arm targets; the jump guard already saw the raw values."""
        config = self.openarm_mini_config
        if config.command_filter_cutoff_hz is None:
            return side_commands
        alpha = low_pass_alpha(config.command_filter_cutoff_hz, config.tick_period_s)
        filtered: list[OpenArmMiniSideCommand] = []
        for side_command in side_commands:
            positions = smooth_positions(
                self._filtered_by_joint, side_command.positions_by_joint, alpha
            )
            self._filtered_by_joint.update(positions)
            filtered.append(replace(side_command, positions_by_joint=positions))
        return filtered

    def stream_trajectory(self, joint_state: JointState) -> bool:
        """Send the targets as a trajectory; False when joint_command should carry them."""
        horizon = self.openarm_mini_config.trajectory_horizon_s
        call = self._trajectory_call
        if call is None or horizon is None or self._stream_fallback:
            return False
        positions = dict(zip(joint_state.name, joint_state.position, strict=True))
        trajectory = streamed_trajectory(self._last_sent_by_joint or None, positions, horizon)
        # Anchor first: a rejection, even one answered synchronously, clears it.
        self._last_sent_by_joint = positions
        try:
            call(trajectory, self._on_trajectory_result)
        except Exception as exc:
            self._note_stream_failure(f"{type(exc).__name__}: {exc}")
            return False
        return True

    def _on_trajectory_result(self, result: Any) -> None:
        if isinstance(result, BaseException):
            self._note_stream_failure(f"{type(result).__name__}: {result}")
            return
        status = getattr(result, "status", None)
        if status is TrajectoryExecutionStatus.ACCEPTED:
            self._stream_failures = 0
            self._last_stream_error = None
            return
        message = getattr(result, "message", "") or "no result"
        name = getattr(status, "name", str(status))
        self._note_stream_failure(f"{name}: {message}")

    def _note_stream_failure(self, message: str) -> None:
        """A rejected target restarts from the measured pose; a dead link falls back."""
        self._last_sent_by_joint = {}
        self._stream_failures += 1
        if message != self._last_stream_error:
            logger.warning("OpenArm Mini trajectory rejected: %s", message)
            self._last_stream_error = message
        if self._stream_failures >= _STREAM_FAILURES_BEFORE_FALLBACK and not self._stream_fallback:
            self._stream_fallback = True
            logger.warning(
                "OpenArm Mini trajectory stream unserved %d times; streaming joint_command",
                self._stream_failures,
            )

    def get_current_command(self) -> JointState | None:
        side_commands = self.get_current_commands()
        return None if side_commands is None else combine_side_commands(side_commands)

    def tick(self) -> None:
        """One polling iteration: arm joints as one JointState, gripper openings per side."""
        if self._stop_event.is_set():
            return
        side_commands = self.get_current_commands()
        if side_commands is None or self._stop_event.is_set():
            return
        joint_state = combine_side_commands(side_commands)
        if not self.stream_trajectory(joint_state):
            self.joint_command.publish(joint_state)
        gripper_outputs: Mapping[OpenArmMiniSide, Out[Float32]] = {
            "left": self.left_gripper_command,
            "right": self.right_gripper_command,
        }
        for side_command in side_commands:
            if side_command.gripper_opening is not None:
                gripper_outputs[side_command.side].publish(
                    Float32(data=side_command.gripper_opening)
                )

    def _run_loop(self) -> None:
        next_tick_time = time.monotonic()
        while not self._stop_event.is_set():
            try:
                self.tick()
            except Exception:
                logger.exception("Unexpected OpenArm Mini teleop polling worker error")
            next_tick_time += self.openarm_mini_config.tick_period_s
            sleep_s = max(0.0, next_tick_time - time.monotonic())
            self._stop_event.wait(sleep_s)

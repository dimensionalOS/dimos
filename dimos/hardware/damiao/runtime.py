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

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import importlib
from pathlib import Path
import time
from typing import Any, cast

import numpy as np

from dimos.hardware.damiao.specs import DamiaoJointGroupSpec, DamiaoRobotSpec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_DEFAULT_TICK_DEADLINE_US = 1_000
_DEFAULT_STATE_CACHE_TTL_S = 0.002
_DEFAULT_ADDRESS = "can0"


class DamiaoBindingUnavailableError(RuntimeError):
    """Raised when the optional can_motor_control binding is unavailable."""


@dataclass(frozen=True)
class DamiaoGroupState:
    """State vectors for one Damiao joint group."""

    q: list[float]
    dq: list[float]
    tau: list[float]


def _load_can_motor_control(
    *,
    adapter_type: str,
    error_type: type[RuntimeError] = DamiaoBindingUnavailableError,
) -> tuple[Any, Any]:
    """Lazily load the optional Rust-backed binding and Damiao codec module."""

    try:
        can_motor_control = importlib.import_module("can_motor_control")
        damiao = importlib.import_module("can_motor_control.damiao")
    except ImportError as exc:
        raise error_type(
            f"The selected '{adapter_type}' adapter requires the Rust-backed "
            "can-motor-control Python binding in the active environment. Install "
            f"dimos[manipulation] before selecting adapter_type='{adapter_type}'."
        ) from exc
    return can_motor_control, damiao


def _dynamic_attr(value: object, name: str) -> Any:
    return getattr(value, name)


class DamiaoRobotRuntime:
    """Binding-backed runtime for one Damiao-based robot spec."""

    def __init__(
        self,
        *,
        robot_spec: DamiaoRobotSpec,
        adapter_type: str = "damiao",
        binding_error_type: type[RuntimeError] = DamiaoBindingUnavailableError,
        use_mock_bus: bool = False,
        config_path: str | Path | None = None,
        tick_deadline_us: int = _DEFAULT_TICK_DEADLINE_US,
        state_cache_ttl_s: float = _DEFAULT_STATE_CACHE_TTL_S,
    ) -> None:
        robot_spec.validate()
        self._robot_spec = robot_spec
        self._adapter_type = adapter_type
        self._binding_error_type = binding_error_type
        self._use_mock_bus = use_mock_bus
        self._config_path = str(config_path) if config_path is not None else None
        self._tick_deadline_us = tick_deadline_us
        self._state_cache_ttl_s = state_cache_ttl_s
        self._robot: Any | None = None
        self._groups: dict[str, Any] = {}
        self._state_cache: dict[str, DamiaoGroupState] = {}
        self._state_cache_time: dict[str, float] = {}
        self._can_motor_control: Any | None = None
        self._damiao: Any | None = None
        self._connected = False
        self._enabled = False

    @property
    def robot_spec(self) -> DamiaoRobotSpec:
        return self._robot_spec

    def connect(self) -> bool:
        """Connect the binding robot and cache group handles."""

        try:
            self._can_motor_control, self._damiao = _load_can_motor_control(
                adapter_type=self._adapter_type,
                error_type=self._binding_error_type,
            )
            robot = self._build_robot()
            robot.connect()
            groups: dict[str, Any] = {}
            for group_name, group_spec in self._robot_spec.groups.items():
                group = robot[group_name]
                if len(group) != group_spec.dof:
                    raise RuntimeError(
                        f"can_motor_control group {group_name!r} has {len(group)} joints, "
                        f"expected {group_spec.dof}"
                    )
                groups[group_name] = group
            self._robot = robot
            self._groups = groups
            self._connected = True
            for group_name in self._robot_spec.groups:
                self.refresh_group_state(group_name, force=True)
        except self._binding_error_type:
            raise
        except Exception:
            logger.exception("damiao runtime connect failed", adapter=self._adapter_type)
            self.disconnect()
            return False
        return True

    def _build_robot(self) -> Any:
        if self._can_motor_control is None or self._damiao is None:
            raise RuntimeError("can_motor_control binding is not loaded")
        if self._config_path is not None:
            return self._can_motor_control.Robot.from_config(self._config_path)
        builder = self._can_motor_control.Robot.builder()
        codec = self._damiao.DamiaoCodec()
        for bus_name, bus_spec in self._robot_spec.buses.items():
            address = str(bus_spec.address or _DEFAULT_ADDRESS)
            transport = (
                self._can_motor_control.MockCanBus.new_fd(address)
                if self._use_mock_bus and bus_spec.fd
                else self._can_motor_control.MockCanBus(address)
                if self._use_mock_bus
                else self._can_motor_control.SocketCanBus(address, fd=bus_spec.fd)
            )
            builder = builder.add_bus(bus_name, transport, codec)
        for group_name, group_spec in self._robot_spec.groups.items():
            binding_specs = [
                self._can_motor_control.MotorSpec(
                    motor.name,
                    cast("int", self._resolve_motor_type(motor.type)),
                    motor.send_id,
                    motor.effective_recv_id,
                )
                for motor in group_spec.motors
            ]
            builder = builder.add_arm(group_name, bus=group_spec.bus_name, motors=binding_specs)
        return builder.build()

    def _resolve_motor_type(self, motor_type: object) -> object:
        if self._damiao is None:
            raise RuntimeError("Damiao binding module is not loaded")
        if isinstance(motor_type, str):
            try:
                return getattr(self._damiao.MotorType, motor_type)
            except AttributeError as exc:
                raise ValueError(f"Unknown Damiao motor type {motor_type!r}") from exc
        if not isinstance(motor_type, int):
            return motor_type
        for name in dir(self._damiao.MotorType):
            if name.startswith("_"):
                continue
            candidate = getattr(self._damiao.MotorType, name)
            try:
                candidate_value = int(candidate)
            except (TypeError, ValueError):
                continue
            if candidate_value == motor_type:
                return candidate
        raise ValueError(f"Unknown Damiao motor type value {motor_type!r}")

    def disconnect(self) -> None:
        """Disable and drop the underlying binding robot."""

        if self._robot is not None:
            try:
                self._robot.disable()
            except Exception:
                logger.warning("damiao runtime disable on disconnect failed", exc_info=True)
        self._enabled = False
        self._connected = False
        self._robot = None
        self._groups = {}
        self._state_cache = {}
        self._state_cache_time = {}

    def is_connected(self) -> bool:
        return self._connected

    def enable(self) -> bool:
        if self._robot is None:
            return False
        try:
            self._robot.enable()
        except Exception:
            logger.exception("damiao runtime enable failed", adapter=self._adapter_type)
            return False
        self._enabled = True
        return True

    def disable(self) -> bool:
        if self._robot is None:
            return False
        try:
            self._robot.disable()
        except Exception:
            logger.exception("damiao runtime disable failed", adapter=self._adapter_type)
            return False
        self._enabled = False
        return True

    def is_enabled(self) -> bool:
        return self._enabled

    def group_spec(self, group_name: str) -> DamiaoJointGroupSpec:
        try:
            return self._robot_spec.groups[group_name]
        except KeyError as exc:
            raise ValueError(f"unknown Damiao group {group_name!r}") from exc

    def refresh_group_state(self, group_name: str, *, force: bool = False) -> DamiaoGroupState:
        group_spec = self.group_spec(group_name)
        group = self._groups.get(group_name)
        if self._robot is None or group is None:
            raise RuntimeError("DamiaoRobotRuntime is not connected")
        now = time.monotonic()
        cached = self._state_cache.get(group_name)
        cached_at = self._state_cache_time.get(group_name, 0.0)
        if not force and cached is not None and now - cached_at <= self._state_cache_ttl_s:
            return cached
        group.refresh()
        self._robot.tick(self._tick_deadline_us)
        state = DamiaoGroupState(
            q=group.positions().astype(np.float64).tolist(),
            dq=group.velocities().astype(np.float64).tolist(),
            tau=group.torques().astype(np.float64).tolist(),
        )
        if any(len(values) != group_spec.dof for values in (state.q, state.dq, state.tau)):
            raise RuntimeError(
                f"state length does not match configured DOF for group {group_name!r}"
            )
        self._state_cache[group_name] = state
        self._state_cache_time[group_name] = time.monotonic()
        return state

    def has_group_states(self, group_names: Sequence[str]) -> bool:
        """Return true only when every requested group has a fresh complete state."""

        try:
            for group_name in group_names:
                self.refresh_group_state(group_name, force=False)
        except Exception:
            return False
        return True

    def read_group_states(self, group_names: Sequence[str]) -> list[DamiaoGroupState]:
        """Read state for groups in the requested order."""

        return [self.refresh_group_state(group_name, force=False) for group_name in group_names]

    def write_group_mit_commands(
        self,
        *,
        group_name: str,
        q: Sequence[float],
        dq: Sequence[float],
        kp: Sequence[float],
        kd: Sequence[float],
        tau: Sequence[float],
    ) -> bool:
        """Write one MIT command frame to a group."""

        group_spec = self.group_spec(group_name)
        group = self._groups.get(group_name)
        if self._robot is None or group is None or not self._enabled:
            return False
        if any(len(values) != group_spec.dof for values in (q, dq, kp, kd, tau)):
            raise ValueError(
                f"command length does not match configured DOF for group {group_name!r}"
            )
        try:
            group.mit_control(np.column_stack([kp, kd, q, dq, tau]).astype(np.float64))
            self._robot.tick(self._tick_deadline_us)
        except Exception:
            logger.exception("damiao runtime MIT command failed", group_name=group_name)
            return False
        self._state_cache.pop(group_name, None)
        self._state_cache_time.pop(group_name, None)
        return True

    def write_groups_mit_commands(
        self,
        commands: Mapping[
            str,
            tuple[
                Sequence[float], Sequence[float], Sequence[float], Sequence[float], Sequence[float]
            ],
        ],
    ) -> bool:
        """Stage MIT commands for multiple groups and tick once.

        The binding's group ``mit_control`` call stages commands; ``robot.tick``
        sends them. Validate all groups and command lengths before staging so a
        bad frame is rejected without sending a partial whole-body command.
        """

        if self._robot is None or not self._enabled:
            return False
        for group_name, values in commands.items():
            group_spec = self.group_spec(group_name)
            group = self._groups.get(group_name)
            if group is None:
                return False
            q, dq, kp, kd, tau = values
            if any(len(vector) != group_spec.dof for vector in (q, dq, kp, kd, tau)):
                raise ValueError(
                    f"command length does not match configured DOF for group {group_name!r}"
                )
        try:
            for group_name, values in commands.items():
                q, dq, kp, kd, tau = values
                self._groups[group_name].mit_control(
                    np.column_stack([kp, kd, q, dq, tau]).astype(np.float64)
                )
            self._robot.tick(self._tick_deadline_us)
        except Exception:
            logger.exception("damiao runtime batched MIT command failed")
            return False
        for group_name in commands:
            self._state_cache.pop(group_name, None)
            self._state_cache_time.pop(group_name, None)
        return True

    def load_gravity_model(
        self,
        group_name: str,
        model_path: str | Path | None = None,
    ) -> tuple[object, object] | None:
        """Load a Pinocchio gravity model for a configured group, if present."""

        resolved_model_path = (
            model_path if model_path is not None else self.group_spec(group_name).gravity_model_path
        )
        if resolved_model_path is None:
            return None
        import pinocchio  # type: ignore[import-not-found]

        build_model_from_urdf = _dynamic_attr(pinocchio, "buildModelFromUrdf")
        model = build_model_from_urdf(str(resolved_model_path))
        return model, _dynamic_attr(model, "createData")()


__all__ = [
    "_DEFAULT_ADDRESS",
    "_DEFAULT_STATE_CACHE_TTL_S",
    "_DEFAULT_TICK_DEADLINE_US",
    "DamiaoBindingUnavailableError",
    "DamiaoGroupState",
    "DamiaoRobotRuntime",
]

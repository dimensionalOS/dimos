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

from pathlib import Path
from typing import Any

import numpy as np

from dimos.hardware.damiao.runtime import (
    _DEFAULT_ADDRESS,
    _DEFAULT_STATE_CACHE_TTL_S,
    _DEFAULT_TICK_DEADLINE_US,
    DamiaoBindingUnavailableError,
    DamiaoRobotRuntime,
)
from dimos.hardware.damiao.specs import DamiaoArmSpec, DamiaoRobotSpec
from dimos.hardware.manipulators.spec import ControlMode, JointLimits, ManipulatorInfo
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_CONTROL_MODE_INDEX = {mode: index for index, mode in enumerate(ControlMode)}


def _dynamic_attr(value: object, name: str) -> Any:
    return getattr(value, name)


class DamiaoArmAdapter:
    """ManipulatorAdapter facade over one Damiao joint group."""

    _adapter_type: str = "damiao"
    _binding_error_type: type[RuntimeError] = DamiaoBindingUnavailableError
    _supported_control_modes: tuple[ControlMode, ...] = (
        ControlMode.POSITION,
        ControlMode.SERVO_POSITION,
        ControlMode.TORQUE,
    )

    def __init__(
        self,
        *,
        robot_spec: DamiaoRobotSpec,
        group_name: str,
        dof: int | None = None,
        hardware_id: str = "arm",
        kp: list[float] | None = None,
        kd: list[float] | None = None,
        gravity_comp: bool = True,
        gravity_model_path: str | Path | None = None,
        gravity_torque_limits: list[float] | tuple[float, ...] | None = None,
        supported_control_modes: tuple[ControlMode, ...] | None = None,
        use_mock_bus: bool = False,
        config_path: str | Path | None = None,
        tick_deadline_us: int = _DEFAULT_TICK_DEADLINE_US,
        state_cache_ttl_s: float = _DEFAULT_STATE_CACHE_TTL_S,
    ) -> None:
        robot_spec.validate()
        if group_name not in robot_spec.groups:
            raise ValueError(f"unknown Damiao group {group_name!r}")
        group_spec = robot_spec.groups[group_name]
        if dof is not None and dof != group_spec.dof:
            raise ValueError(
                f"{type(self).__name__} only supports {group_spec.dof} DOF (got {dof})"
            )
        self._robot_spec = robot_spec
        self._group_name = group_name
        self._group_spec = group_spec
        self._hardware_id = hardware_id
        self._dof = group_spec.dof
        self._position_lower = list(group_spec.position_lower)
        self._position_upper = list(group_spec.position_upper)
        self._velocity_max = list(group_spec.velocity_max)
        self._kp = list(kp) if kp is not None else list(group_spec.kp)
        self._kd = list(kd) if kd is not None else list(group_spec.kd)
        self._validate_length("kp", self._kp)
        self._validate_length("kd", self._kd)
        self._gravity_comp = gravity_comp
        resolved_gravity_model = (
            gravity_model_path if gravity_model_path is not None else group_spec.gravity_model_path
        )
        self._gravity_model_path = str(resolved_gravity_model) if resolved_gravity_model else None
        resolved_torque_limits = (
            gravity_torque_limits
            if gravity_torque_limits is not None
            else group_spec.gravity_torque_limits
        )
        self._gravity_torque_limits = (
            list(resolved_torque_limits) if resolved_torque_limits else None
        )
        if self._gravity_torque_limits is not None:
            self._validate_length("gravity_torque_limits", self._gravity_torque_limits)
        self._supported_control_modes = (
            supported_control_modes or type(self)._supported_control_modes
        )
        self._control_mode = ControlMode.POSITION
        self._last_positions: list[float] | None = None
        self._pin_model: object | None = None
        self._pin_data: object | None = None
        self._use_mock_bus = use_mock_bus
        self._config_path = config_path
        self._tick_deadline_us = tick_deadline_us
        self._state_cache_ttl_s = state_cache_ttl_s
        self._runtime: DamiaoRobotRuntime | None = None
        self._connected = False
        self._enabled = False

    @classmethod
    def from_arm_spec(
        cls,
        *,
        arm_spec: DamiaoArmSpec,
        address: str | Path | None = _DEFAULT_ADDRESS,
        **kwargs: Any,
    ) -> DamiaoArmAdapter:
        """Build a one-group adapter from a compatibility arm spec."""

        robot_spec = DamiaoRobotSpec.from_arm_spec(
            arm_spec,
            address=str(address) if address is not None else _DEFAULT_ADDRESS,
        )
        return cls(robot_spec=robot_spec, group_name=arm_spec.arm_name, **kwargs)

    def _create_runtime(self) -> DamiaoRobotRuntime:
        return DamiaoRobotRuntime(
            robot_spec=self._robot_spec,
            adapter_type=self._adapter_type,
            binding_error_type=self._binding_error_type,
            use_mock_bus=self._use_mock_bus,
            config_path=self._config_path,
            tick_deadline_us=self._tick_deadline_us,
            state_cache_ttl_s=self._state_cache_ttl_s,
        )

    def _validate_length(self, name: str, values: list[float]) -> None:
        if len(values) != self._dof:
            raise ValueError(f"{name} length {len(values)} does not match dof {self._dof}")

    def _validate_command_lengths(self, **commands: list[float]) -> None:
        for name, values in commands.items():
            self._validate_length(name, values)

    def _zero_vector(self) -> list[float]:
        return [0.0] * self._dof

    def connect(self) -> bool:
        try:
            runtime = self._create_runtime()
            if not runtime.connect():
                return False
            self._runtime = runtime
            self._load_gravity_model()
            self._connected = True
            self.refresh_state(force=True)
        except self._binding_error_type:
            raise
        except Exception:
            logger.exception(
                "damiao arm adapter connect failed",
                adapter=type(self).__name__,
                hardware_id=self._hardware_id,
            )
            self.disconnect()
            return False
        return True

    def disconnect(self) -> None:
        if self._runtime is not None:
            self._runtime.disconnect()
        self._runtime = None
        self._connected = False
        self._enabled = False

    def is_connected(self) -> bool:
        return self._connected

    def activate(self) -> bool:
        return self.write_enable(True)

    def deactivate(self) -> bool:
        stopped = self.write_stop()
        disabled = self.write_enable(False)
        return stopped and disabled

    def get_info(self) -> ManipulatorInfo:
        return ManipulatorInfo(
            vendor=self._robot_spec.vendor,
            model=self._robot_spec.model,
            dof=self._dof,
            firmware_version=None,
            serial_number=None,
        )

    def get_dof(self) -> int:
        return self._dof

    def get_limits(self) -> JointLimits:
        return JointLimits(
            position_lower=list(self._position_lower),
            position_upper=list(self._position_upper),
            velocity_max=list(self._velocity_max),
        )

    def set_control_mode(self, mode: ControlMode) -> bool:
        if mode not in self._supported_control_modes:
            return False
        self._control_mode = mode
        return True

    def get_control_mode(self) -> ControlMode:
        return self._control_mode

    def read_enabled(self) -> bool:
        return self._enabled

    def refresh_state(self, *, force: bool = False) -> tuple[list[float], list[float], list[float]]:
        if self._runtime is None:
            raise RuntimeError(f"{type(self).__name__} is not connected")
        state = self._runtime.refresh_group_state(self._group_name, force=force)
        self._last_positions = list(state.q)
        return list(state.q), list(state.dq), list(state.tau)

    def read_joint_positions(self) -> list[float]:
        return list(self.refresh_state()[0])

    def read_joint_velocities(self) -> list[float]:
        return list(self.refresh_state()[1])

    def read_joint_efforts(self) -> list[float]:
        return list(self.refresh_state()[2])

    def read_state(self) -> dict[str, int]:
        return {"state": 1 if self._enabled else 0, "mode": _CONTROL_MODE_INDEX[self._control_mode]}

    def read_error(self) -> tuple[int, str]:
        return 0, ""

    def read_cartesian_position(self) -> dict[str, float] | None:
        return None

    def write_cartesian_position(self, pose: dict[str, float], velocity: float = 1.0) -> bool:
        return False

    def read_gripper_position(self) -> float | None:
        return None

    def write_gripper_position(self, position: float) -> bool:
        return False

    def read_force_torque(self) -> list[float] | None:
        return None

    def write_joint_positions(self, positions: list[float], velocity: float = 1.0) -> bool:
        if self._runtime is None or not self._enabled or len(positions) != self._dof:
            return False
        velocity = max(0.0, min(1.0, velocity))
        if self._gravity_comp:
            try:
                tau = self.compute_gravity_torques(self.read_joint_positions())
            except RuntimeError:
                logger.warning(
                    "damiao arm adapter dropping gravity feed-forward; state read failed",
                    adapter=type(self).__name__,
                    hardware_id=self._hardware_id,
                    exc_info=True,
                )
                tau = self._zero_vector()
        else:
            tau = self._zero_vector()
        return self.write_mit_commands(
            q=list(positions),
            dq=self._zero_vector(),
            kp=[kp * velocity for kp in self._kp],
            kd=list(self._kd),
            tau=tau,
        )

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        return False

    def write_joint_torques(self, efforts: list[float]) -> bool:
        if self._runtime is None or not self._enabled or len(efforts) != self._dof:
            return False
        q = (
            self._last_positions
            if self._last_positions is not None
            else self.read_joint_positions()
        )
        return self.write_mit_commands(
            q=q,
            dq=self._zero_vector(),
            kp=self._zero_vector(),
            kd=self._zero_vector(),
            tau=efforts,
        )

    def write_mit_commands(
        self,
        *,
        q: list[float],
        dq: list[float],
        kp: list[float],
        kd: list[float],
        tau: list[float],
    ) -> bool:
        if self._runtime is None or not self._enabled:
            return False
        self._validate_command_lengths(q=q, dq=dq, kp=kp, kd=kd, tau=tau)
        ok = self._runtime.write_group_mit_commands(
            group_name=self._group_name,
            q=q,
            dq=dq,
            kp=kp,
            kd=kd,
            tau=tau,
        )
        if ok:
            self._last_positions = list(q)
            self._control_mode = (
                ControlMode.TORQUE if all(k == 0.0 for k in kp) else ControlMode.POSITION
            )
        return ok

    def write_stop(self) -> bool:
        if self._runtime is None:
            return False
        if self._gravity_comp and self._enabled:
            try:
                q_now = self.read_joint_positions()
            except RuntimeError:
                self._runtime.disable()
                self._enabled = False
                return False
            return self.write_mit_commands(
                q=q_now,
                dq=self._zero_vector(),
                kp=list(self._kp),
                kd=list(self._kd),
                tau=self.compute_gravity_torques(q_now),
            )
        disabled = self._runtime.disable()
        if disabled:
            self._enabled = False
        return disabled

    def write_enable(self, enable: bool) -> bool:
        if self._runtime is None:
            return False
        ok = self._runtime.enable() if enable else self._runtime.disable()
        if not ok:
            return False
        self._enabled = enable
        if enable:
            positions = self.read_joint_positions()
            if not self.write_joint_positions(positions):
                self._runtime.disable()
                self._enabled = False
                return False
        return True

    def write_clear_errors(self) -> bool:
        if self._runtime is None:
            return False
        if not self._runtime.disable() or not self._runtime.enable():
            return False
        self._enabled = True
        return self.write_joint_positions(self.read_joint_positions())

    def _load_gravity_model(self) -> None:
        if not self._gravity_comp or self._gravity_model_path is None or self._runtime is None:
            return
        loaded = self._runtime.load_gravity_model(self._group_name, self._gravity_model_path)
        if loaded is not None:
            self._pin_model, self._pin_data = loaded

    def compute_gravity_torques(self, q: list[float]) -> list[float]:
        self._validate_length("q", q)
        if self._pin_model is None or self._pin_data is None:
            return self._zero_vector()
        import pinocchio  # type: ignore[import-not-found]

        compute_generalized_gravity = _dynamic_attr(pinocchio, "computeGeneralizedGravity")
        tau = compute_generalized_gravity(
            self._pin_model, self._pin_data, np.array(q, dtype=np.float64)
        )
        values = [float(tau[i]) for i in range(self._dof)]
        if self._gravity_torque_limits is None:
            return values
        return [
            float(np.clip(value, -limit, limit))
            for value, limit in zip(values, self._gravity_torque_limits, strict=False)
        ]


__all__ = ["DamiaoArmAdapter"]

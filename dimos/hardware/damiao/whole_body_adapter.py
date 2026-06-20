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

from collections.abc import Sequence
from typing import Any

import numpy as np

from dimos.hardware.damiao.runtime import (
    _DEFAULT_STATE_CACHE_TTL_S,
    _DEFAULT_TICK_DEADLINE_US,
    DamiaoBindingUnavailableError,
    DamiaoRobotRuntime,
)
from dimos.hardware.damiao.specs import DamiaoRobotSpec
from dimos.hardware.whole_body.spec import IMUState, MotorCommand, MotorState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _dynamic_attr(value: object, name: str) -> Any:
    return getattr(value, name)


class DamiaoWholeBodyAdapter:
    """Position-level WholeBodyAdapter facade over ordered Damiao groups."""

    _adapter_type: str = "damiao_whole_body"
    _binding_error_type: type[RuntimeError] = DamiaoBindingUnavailableError

    def __init__(
        self,
        *,
        robot_spec: DamiaoRobotSpec,
        group_names: Sequence[str],
        dof: int | None = None,
        hardware_id: str = "body",
        gravity_comp: bool = True,
        use_mock_bus: bool = False,
        tick_deadline_us: int = _DEFAULT_TICK_DEADLINE_US,
        state_cache_ttl_s: float = _DEFAULT_STATE_CACHE_TTL_S,
    ) -> None:
        robot_spec.validate()
        if not group_names:
            raise ValueError("DamiaoWholeBodyAdapter requires at least one group")
        unknown_groups = [name for name in group_names if name not in robot_spec.groups]
        if unknown_groups:
            raise ValueError(f"unknown Damiao groups: {unknown_groups}")
        self._robot_spec = robot_spec
        self._group_names = tuple(group_names)
        self._hardware_id = hardware_id
        self._gravity_comp = gravity_comp
        self._use_mock_bus = use_mock_bus
        self._tick_deadline_us = tick_deadline_us
        self._state_cache_ttl_s = state_cache_ttl_s
        self._dof = sum(robot_spec.groups[name].dof for name in self._group_names)
        if dof is not None and dof != self._dof:
            raise ValueError(f"{type(self).__name__} supports {self._dof} DOF (got {dof})")
        self._position_lower = [
            value
            for group_name in self._group_names
            for value in robot_spec.groups[group_name].position_lower
        ]
        self._position_upper = [
            value
            for group_name in self._group_names
            for value in robot_spec.groups[group_name].position_upper
        ]
        self._kp = [
            value for group_name in self._group_names for value in robot_spec.groups[group_name].kp
        ]
        self._kd = [
            value for group_name in self._group_names for value in robot_spec.groups[group_name].kd
        ]
        self._runtime: DamiaoRobotRuntime | None = None
        self._connected = False
        self._enabled = False
        self._pin_models: dict[str, object] = {}
        self._pin_data: dict[str, object] = {}

    @property
    def joint_names(self) -> tuple[str, ...]:
        return self._robot_spec.group_joint_names(self._group_names)

    def _create_runtime(self) -> DamiaoRobotRuntime:
        return DamiaoRobotRuntime(
            robot_spec=self._robot_spec,
            adapter_type=self._adapter_type,
            binding_error_type=self._binding_error_type,
            use_mock_bus=self._use_mock_bus,
            tick_deadline_us=self._tick_deadline_us,
            state_cache_ttl_s=self._state_cache_ttl_s,
        )

    def connect(self) -> bool:
        try:
            runtime = self._create_runtime()
            if not runtime.connect():
                return False
            self._runtime = runtime
            self._load_gravity_models()
            self._connected = True
            # Whole-body v1 intentionally exposes no optional lifecycle hooks,
            # so make the position surface usable after coordinator connect().
            self._enabled = runtime.enable()
            if not self._enabled:
                self.disconnect()
                return False
            current_positions = [state.q for state in self.read_motor_states()]
            if not self.write_joint_positions(current_positions):
                logger.error("damiao whole-body startup hold failed", hardware_id=self._hardware_id)
                self.disconnect()
                return False
        except self._binding_error_type:
            raise
        except Exception:
            logger.exception(
                "damiao whole-body adapter connect failed", hardware_id=self._hardware_id
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
        self._pin_models = {}
        self._pin_data = {}

    def is_connected(self) -> bool:
        return self._connected

    def has_motor_states(self) -> bool:
        if self._runtime is None:
            return False
        return self._runtime.has_group_states(self._group_names)

    def read_motor_states(self) -> list[MotorState]:
        if self._runtime is None:
            raise RuntimeError(f"{type(self).__name__} is not connected")
        group_states = self._runtime.read_group_states(self._group_names)
        states: list[MotorState] = []
        for group_state in group_states:
            states.extend(
                MotorState(q=q, dq=dq, tau=tau)
                for q, dq, tau in zip(group_state.q, group_state.dq, group_state.tau, strict=True)
            )
        if len(states) != self._dof:
            raise RuntimeError(f"expected {self._dof} motor states, got {len(states)}")
        return states

    def read_imu(self) -> IMUState:
        return IMUState()

    def write_joint_positions(self, positions: Sequence[float]) -> bool:
        """Write a full ordered position frame using configured PD gains."""

        if self._runtime is None or not self._enabled or len(positions) != self._dof:
            return False
        q_all = [float(value) for value in positions]
        if not self._within_limits(q_all):
            return False
        if not self.has_motor_states():
            return False

        offset = 0
        frames: dict[
            str, tuple[list[float], list[float], list[float], list[float], list[float]]
        ] = {}
        for group_name in self._group_names:
            group_spec = self._robot_spec.groups[group_name]
            width = group_spec.dof
            q = q_all[offset : offset + width]
            tau = (
                self.compute_gravity_torques(group_name, q) if self._gravity_comp else [0.0] * width
            )
            frames[group_name] = (q, [0.0] * width, list(group_spec.kp), list(group_spec.kd), tau)
            offset += width
        return self._runtime.write_groups_mit_commands(frames)

    def write_motor_commands(self, commands: list[MotorCommand]) -> bool:
        raise NotImplementedError(
            "TODO: Implement Damiao raw MotorCommand support after the Damiao whole-body "
            "command interface is refactored."
        )

    def _within_limits(self, positions: Sequence[float]) -> bool:
        for value, lower, upper in zip(
            positions, self._position_lower, self._position_upper, strict=True
        ):
            if value < lower or value > upper:
                logger.warning(
                    "damiao whole-body position outside limits",
                    hardware_id=self._hardware_id,
                    value=value,
                    lower=lower,
                    upper=upper,
                )
                return False
        return True

    def _load_gravity_models(self) -> None:
        if not self._gravity_comp or self._runtime is None:
            return
        for group_name in self._group_names:
            loaded = self._runtime.load_gravity_model(group_name)
            if loaded is None:
                continue
            self._pin_models[group_name], self._pin_data[group_name] = loaded

    def compute_gravity_torques(self, group_name: str, q: list[float]) -> list[float]:
        group_spec = self._robot_spec.groups[group_name]
        if group_name not in self._pin_models or group_name not in self._pin_data:
            return [0.0] * group_spec.dof
        if len(q) != group_spec.dof:
            raise ValueError(f"q length does not match dof for group {group_name!r}")
        import pinocchio  # type: ignore[import-not-found]

        compute_generalized_gravity = _dynamic_attr(pinocchio, "computeGeneralizedGravity")
        tau = compute_generalized_gravity(
            self._pin_models[group_name],
            self._pin_data[group_name],
            np.array(q, dtype=np.float64),
        )
        values = [float(tau[i]) for i in range(group_spec.dof)]
        if group_spec.gravity_torque_limits is None:
            return values
        return [
            float(np.clip(value, -limit, limit))
            for value, limit in zip(values, group_spec.gravity_torque_limits, strict=False)
        ]


__all__ = ["DamiaoWholeBodyAdapter"]

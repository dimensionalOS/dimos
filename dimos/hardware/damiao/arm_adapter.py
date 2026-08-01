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

from typing import Any

import numpy as np
import pinocchio  # type: ignore[import-not-found]

from dimos.hardware.damiao.config import DamiaoArmConfig, DamiaoRuntimeConfig
from dimos.hardware.damiao.runtime import DamiaoArmRuntime
from dimos.hardware.manipulators.spec import ControlMode, JointLimits, ManipulatorInfo
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_CONTROL_MODE_INDEX = {mode: index for index, mode in enumerate(ControlMode)}


class DamiaoArmAdapter:
    """ManipulatorAdapter facade over one Damiao arm runtime."""

    _adapter_type: str = "damiao"

    def __init__(
        self,
        *,
        arm_config: DamiaoArmConfig,
        runtime_config: DamiaoRuntimeConfig | None = None,
        dof: int | None = None,
        hardware_id: str = "arm",
    ) -> None:
        runtime_config = runtime_config or DamiaoRuntimeConfig()
        if dof is not None and dof != arm_config.dof:
            raise ValueError(
                f"{type(self).__name__} only supports {arm_config.dof} DOF (got {dof})"
            )
        self._arm_config = arm_config
        self._runtime_config = runtime_config
        self._hardware_id = hardware_id
        self._dof = arm_config.dof
        self._position_lower = list(arm_config.position_lower)
        self._position_upper = list(arm_config.position_upper)
        self._velocity_max = list(arm_config.velocity_max)
        self._kp = list(runtime_config.kp_override or arm_config.kp)
        self._kd = list(runtime_config.kd_override or arm_config.kd)
        self._validate_length("kp", self._kp)
        self._validate_length("kd", self._kd)
        self._gravity_comp = runtime_config.gravity_comp
        self._gravity_model_path = (
            str(runtime_config.gravity_model_path) if runtime_config.gravity_model_path else None
        )
        resolved_torque_limits = arm_config.gravity_torque_limits
        self._gravity_torque_limits = (
            list(resolved_torque_limits) if resolved_torque_limits else None
        )
        if self._gravity_torque_limits is not None:
            self._validate_length("gravity_torque_limits", self._gravity_torque_limits)
        self._supported_control_modes = arm_config.supported_control_modes
        self._control_mode = ControlMode.POSITION
        self._last_positions: list[float] | None = None
        self._pin_model: Any | None = None
        self._pin_data: Any | None = None
        self._runtime: DamiaoArmRuntime | None = None
        self._connected = False
        self._enabled = False

    def _create_runtime(self) -> DamiaoArmRuntime:
        return DamiaoArmRuntime(
            arm_config=self._arm_config,
            runtime_config=self._runtime_config,
            adapter_type=self._adapter_type,
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
            self._enabled = self._runtime.is_enabled()
        self._runtime = None
        self._connected = False

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
            vendor=self._arm_config.vendor,
            model=self._arm_config.model,
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
        state = self._runtime.refresh_state(force=force)
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
            except Exception:
                logger.warning(
                    "damiao arm adapter gravity command safety failure",
                    adapter=type(self).__name__,
                    hardware_id=self._hardware_id,
                    exc_info=True,
                )
                return self._disable_after_safety_failure()
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
        try:
            q = (
                self._last_positions
                if self._last_positions is not None
                else self.read_joint_positions()
            )
        except Exception:
            if self._gravity_comp:
                return self._disable_after_safety_failure()
            raise
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
        ok = self._runtime.write_mit_commands(
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
            except Exception:
                logger.warning(
                    "damiao arm adapter gravity stop safety failure",
                    adapter=type(self).__name__,
                    hardware_id=self._hardware_id,
                    exc_info=True,
                )
                return self._disable_after_safety_failure()
            try:
                tau = self.compute_gravity_torques(q_now)
            except Exception:
                logger.warning(
                    "damiao arm adapter gravity stop safety failure",
                    adapter=type(self).__name__,
                    hardware_id=self._hardware_id,
                    exc_info=True,
                )
                return self._disable_after_safety_failure()
            return self.write_mit_commands(
                q=q_now,
                dq=self._zero_vector(),
                kp=list(self._kp),
                kd=list(self._kd),
                tau=tau,
            )
        disabled = self._runtime.disable()
        if disabled:
            self._enabled = False
        return disabled

    def write_enable(self, enable: bool) -> bool:
        if self._runtime is None:
            return False
        if not enable:
            ok = self._runtime.disable()
            if ok:
                self._enabled = False
            else:
                self._enabled = True
            return ok

        # Do every model/state check while the motors are still disabled.
        # This is deliberately kept in the generic adapter rather than in a
        # robot-specific implementation: a bad URDF must not result in a
        # live actuator state.
        try:
            self._preflight_gravity()
        except Exception:
            logger.exception(
                "damiao arm adapter rejected enable preflight",
                adapter=type(self).__name__,
                hardware_id=self._hardware_id,
            )
            return False

        ok = self._runtime.enable()
        if not ok:
            return False
        self._enabled = enable
        try:
            positions = self.read_joint_positions()
            if not self.write_joint_positions(positions):
                self._rollback_enable()
                return False
        except Exception:
            logger.exception(
                "damiao arm adapter enable hold failed",
                adapter=type(self).__name__,
                hardware_id=self._hardware_id,
            )
            self._rollback_enable()
            return False
        return True

    def _rollback_enable(self) -> None:
        """Disable after a failure occurring after runtime enable."""

        if self._runtime is not None:
            try:
                disabled = self._runtime.disable()
            except Exception:
                logger.warning("damiao arm adapter enable rollback failed", exc_info=True)
                disabled = False
            if disabled:
                self._enabled = False
                return
            logger.error("damiao arm adapter enable rollback could not disable hardware")
        self._enabled = True

    def _disable_after_safety_failure(self) -> bool:
        """Disable without sending a fallback (possibly zero-torque) command."""

        if self._runtime is not None:
            try:
                disabled = self._runtime.disable()
            except Exception:
                logger.exception(
                    "damiao arm adapter safety disable failed",
                    adapter=type(self).__name__,
                    hardware_id=self._hardware_id,
                )
                disabled = False
            if not disabled:
                logger.error("damiao arm adapter safety disable could not disable hardware")
                self._enabled = True
                return False
        self._enabled = False
        return False

    def _preflight_gravity(self) -> None:
        """Validate state and gravity output before enabling any motor.

        Pinocchio models expose their joint order and generalized dimensions;
        checking both here prevents silently applying a valid-looking torque
        vector to the wrong joints.
        """

        if self._gravity_comp and (self._pin_model is None or self._pin_data is None):
            raise ValueError("gravity compensation requires a loaded gravity model")

        q, _, _ = self.refresh_state(force=True)
        if len(q) != self._dof or not np.isfinite(np.asarray(q, dtype=np.float64)).all():
            raise ValueError(
                "gravity preflight requires finite positions in configured joint order"
            )

        if self._pin_model is not None:
            nq = self._pin_model.nq
            nv = self._pin_model.nv
            if nq != self._dof or nv != self._dof:
                raise ValueError(
                    f"gravity model dimensions ({nq}, {nv}) do not match adapter DOF {self._dof}"
                )
            model_names = tuple(str(name) for name in self._pin_model.names)
            if model_names and model_names[0] == "universe":
                model_names = model_names[1:]
            if model_names != self._arm_config.joint_names:
                raise ValueError(
                    f"gravity model joint order {model_names!r} does not match "
                    f"configured order {self._arm_config.joint_names!r}"
                )

        tau = self.compute_gravity_torques(q) if self._gravity_comp else self._zero_vector()
        if len(tau) != self._dof or not np.isfinite(np.asarray(tau, dtype=np.float64)).all():
            raise ValueError("gravity preflight requires finite torque values matching adapter DOF")
        if self._gravity_torque_limits is not None and any(
            not np.isfinite(limit) or limit < 0.0 for limit in self._gravity_torque_limits
        ):
            raise ValueError("gravity torque limits must be finite and non-negative")

    def write_clear_errors(self) -> bool:
        if self._runtime is None:
            return False
        if not self._runtime.disable():
            self._enabled = True
            return False
        self._enabled = False
        try:
            self._preflight_gravity()
        except Exception:
            logger.exception(
                "damiao arm adapter rejected error-recovery enable preflight",
                adapter=type(self).__name__,
                hardware_id=self._hardware_id,
            )
            return False
        if not self._runtime.enable():
            return False
        self._enabled = True
        try:
            ok = self.write_joint_positions(self.read_joint_positions())
        except Exception:
            self._rollback_enable()
            return False
        if not ok:
            self._rollback_enable()
        return ok

    def _load_gravity_model(self) -> None:
        if not self._gravity_comp or self._gravity_model_path is None or self._runtime is None:
            return
        self._pin_model, self._pin_data = self._runtime.load_gravity_model(self._gravity_model_path)

    def compute_gravity_torques(self, q: list[float]) -> list[float]:
        self._validate_length("q", q)
        if self._pin_model is None or self._pin_data is None:
            raise RuntimeError("gravity compensation model is not loaded")
        tau = pinocchio.computeGeneralizedGravity(
            self._pin_model, self._pin_data, np.array(q, dtype=np.float64)
        )
        values = [float(tau[i]) for i in range(self._dof)]
        if not np.isfinite(np.asarray(values, dtype=np.float64)).all():
            raise RuntimeError("gravity computation returned non-finite torque values")
        if self._gravity_torque_limits is None:
            return values
        return [
            float(np.clip(value, -limit, limit))
            for value, limit in zip(values, self._gravity_torque_limits, strict=False)
        ]

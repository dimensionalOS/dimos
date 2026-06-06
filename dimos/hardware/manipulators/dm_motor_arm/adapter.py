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

import importlib
from pathlib import Path
import time
from types import ModuleType
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.hardware.manipulators.damiao.base_adapter import DamiaoArmAdapterBase
from dimos.hardware.manipulators.damiao.specs import DamiaoArmSpec, DamiaoMotorSpec
from dimos.hardware.manipulators.spec import ControlMode
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.hardware.manipulators.registry import AdapterRegistry

logger = setup_logger()

DMMotorSpecConfig = DamiaoMotorSpec


class DMMotorBindingUnavailableError(RuntimeError):
    pass


def _load_can_motor_control() -> tuple[ModuleType, ModuleType]:
    try:
        can_motor_control = importlib.import_module("can_motor_control")
        damiao = importlib.import_module("can_motor_control.damiao")
    except ImportError as exc:
        raise DMMotorBindingUnavailableError(
            "The selected 'dm_motor_arm' adapter requires the Rust-backed "
            "can-motor-control Python binding in the active environment. Install "
            "dimos[manipulation] before selecting adapter_type='dm_motor_arm'."
        ) from exc
    return can_motor_control, damiao


def _resolve_motor_type(damiao: ModuleType, motor_type: str | int | Any) -> Any:
    if isinstance(motor_type, str):
        try:
            return getattr(damiao.MotorType, motor_type)
        except AttributeError as exc:
            raise ValueError(f"Unknown Damiao motor type {motor_type!r}") from exc
    if not isinstance(motor_type, int):
        return motor_type
    for name in dir(damiao.MotorType):
        if name.startswith("_"):
            continue
        candidate = getattr(damiao.MotorType, name)
        try:
            candidate_value = int(candidate)
        except (TypeError, ValueError):
            continue
        if candidate_value == motor_type:
            return candidate
    raise ValueError(f"Unknown Damiao motor type value {motor_type!r}")


class DMMotorArm(DamiaoArmAdapterBase):
    _DEFAULT_OPENARM_MOTORS: tuple[DamiaoMotorSpec, ...] = (
        DamiaoMotorSpec("joint1", "DM8006", 0x01, 0x11),
        DamiaoMotorSpec("joint2", "DM8006", 0x02, 0x12),
        DamiaoMotorSpec("joint3", "DM4340", 0x03, 0x13),
        DamiaoMotorSpec("joint4", "DM4340", 0x04, 0x14),
        DamiaoMotorSpec("joint5", "DM4310", 0x05, 0x15),
        DamiaoMotorSpec("joint6", "DM4310", 0x06, 0x16),
        DamiaoMotorSpec("joint7", "DM4310", 0x07, 0x17),
    )
    _DEFAULT_POSITION_LOWER: tuple[float, ...] = (-3.45, -3.30, -1.50, -0.01, -1.50, -0.75, -1.50)
    _DEFAULT_POSITION_UPPER: tuple[float, ...] = (1.35, 0.15, 1.50, 2.40, 1.50, 0.75, 1.50)
    _DEFAULT_VELOCITY_MAX: tuple[float, ...] = (45.0, 45.0, 8.0, 8.0, 30.0, 30.0, 30.0)
    _DEFAULT_KP: tuple[float, ...] = (70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0)
    _DEFAULT_KD: tuple[float, ...] = (2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5)

    def __init__(
        self,
        address: str | Path | None = "can0",
        dof: int = 7,
        *,
        hardware_id: str = "arm",
        config_path: str | Path | None = None,
        arm_name: str = "arm",
        bus_name: str = "can",
        fd: bool | None = None,
        canfd: bool = True,
        use_mock_bus: bool = False,
        motor_specs: list[dict[str, Any] | DamiaoMotorSpec] | None = None,
        position_lower: list[float] | None = None,
        position_upper: list[float] | None = None,
        velocity_max: list[float] | None = None,
        kp: list[float] | None = None,
        kd: list[float] | None = None,
        gravity_comp: bool = True,
        tick_deadline_us: int = 1_000,
        state_cache_ttl_s: float = 0.002,
        gravity_model_path: str | Path | None = None,
        gravity_torque_limits: list[float] | None = None,
        **_: Any,
    ) -> None:
        specs = self._coerce_motor_specs(motor_specs, dof)
        arm_spec = DamiaoArmSpec.from_values(
            name="dm_motor_arm",
            vendor="Damiao",
            model="DMMotorArm",
            motors=tuple(specs),
            position_lower=position_lower if position_lower is not None else self._DEFAULT_POSITION_LOWER[:dof],
            position_upper=position_upper if position_upper is not None else self._DEFAULT_POSITION_UPPER[:dof],
            velocity_max=velocity_max if velocity_max is not None else self._DEFAULT_VELOCITY_MAX[:dof],
            kp=kp if kp is not None else self._DEFAULT_KP[:dof],
            kd=kd if kd is not None else self._DEFAULT_KD[:dof],
            gravity_model_path=gravity_model_path,
            gravity_torque_limits=gravity_torque_limits,
            bus_name=bus_name,
            arm_name=arm_name,
            fd=canfd if fd is None else fd,
        )
        super().__init__(
            arm_spec=arm_spec,
            hardware_id=hardware_id,
            gravity_comp=gravity_comp,
            supported_control_modes=(
                ControlMode.POSITION,
                ControlMode.SERVO_POSITION,
                ControlMode.TORQUE,
            ),
        )
        self._address = str(address) if address is not None else "can0"
        self._config_path = str(config_path) if config_path is not None else None
        self._arm_name = arm_name
        self._bus_name = bus_name
        self._fd = canfd if fd is None else fd
        self._use_mock_bus = use_mock_bus
        self._tick_deadline_us = tick_deadline_us
        self._state_cache_ttl_s = state_cache_ttl_s

        self._can_motor_control: ModuleType | None = None
        self._damiao: ModuleType | None = None
        self._robot: Any = None
        self._arm: Any = None
        self._connected = False
        self._state_cache: tuple[list[float], list[float], list[float]] | None = None
        self._state_cache_time = 0.0

    @classmethod
    def _coerce_motor_specs(
        cls,
        motor_specs: list[dict[str, Any] | DamiaoMotorSpec] | None,
        dof: int,
    ) -> list[DamiaoMotorSpec]:
        if motor_specs is None:
            if dof != len(cls._DEFAULT_OPENARM_MOTORS):
                raise ValueError(
                    "motor_specs is required when constructing a dm_motor_arm with "
                    f"{dof} DOF; the built-in default only describes OpenArm-style 7 DOF"
                )
            return list(cls._DEFAULT_OPENARM_MOTORS)
        specs: list[DamiaoMotorSpec] = []
        for spec in motor_specs:
            if isinstance(spec, DamiaoMotorSpec):
                specs.append(spec)
            else:
                specs.append(DamiaoMotorSpec(**spec))
        if len(specs) != dof:
            raise ValueError(f"motor_specs length {len(specs)} does not match dof {dof}")
        return specs

    def connect(self) -> bool:
        try:
            self._can_motor_control, self._damiao = _load_can_motor_control()
            self._robot = self._build_robot()
            self._robot.connect()
            self._arm = self._robot[self._arm_name]
            if len(self._arm) != self._dof:
                raise RuntimeError(
                    f"can_motor_control arm group {self._arm_name!r} has {len(self._arm)} joints, "
                    f"expected {self._dof}"
                )
            self._load_gravity_model()
            self._connected = True
            self.refresh_state(force=True)
        except DMMotorBindingUnavailableError:
            raise
        except Exception as exc:
            logger.error(f"DMMotorArm {self._hardware_id}@{self._address} connect failed: {exc}")
            self._robot = None
            self._arm = None
            self._connected = False
            return False
        return True

    def _build_robot(self) -> Any:
        assert self._can_motor_control is not None
        assert self._damiao is not None
        if self._config_path is not None:
            return self._can_motor_control.Robot.from_config(self._config_path)

        transport = (
            self._can_motor_control.MockCanBus.new_fd(self._address)
            if self._use_mock_bus and self._fd
            else self._can_motor_control.MockCanBus(self._address)
            if self._use_mock_bus
            else self._can_motor_control.SocketCanBus(self._address, fd=self._fd)
        )
        codec = self._damiao.DamiaoCodec()
        binding_specs = [
            self._can_motor_control.MotorSpec(
                spec.name,
                _resolve_motor_type(self._damiao, spec.type),
                spec.send_id,
                spec.effective_recv_id,
            )
            for spec in self._motor_specs
        ]
        return (
            self._can_motor_control.Robot.builder()
            .add_bus(self._bus_name, transport, codec)
            .add_arm(self._arm_name, bus=self._bus_name, motors=binding_specs)
            .build()
        )

    def disconnect(self) -> None:
        if self._robot is not None:
            try:
                self._robot.disable()
            except Exception as exc:
                logger.warning(f"DMMotorArm {self._hardware_id} disable on disconnect failed: {exc}")
        self._enabled = False
        self._connected = False
        self._robot = None
        self._arm = None
        self._state_cache = None

    def is_connected(self) -> bool:
        return self._connected

    def refresh_state(
        self, *, force: bool = False
    ) -> tuple[list[float], list[float], list[float]]:
        if self._robot is None or self._arm is None:
            raise RuntimeError("DMMotorArm is not connected")
        now = time.monotonic()
        if (
            not force
            and self._state_cache is not None
            and now - self._state_cache_time <= self._state_cache_ttl_s
        ):
            return self._state_cache
        self._robot.tick(self._tick_deadline_us)
        state = (
            self._arm.positions().astype(float).tolist(),
            self._arm.velocities().astype(float).tolist(),
            self._arm.torques().astype(float).tolist(),
        )
        if any(len(values) != self._dof for values in state):
            raise RuntimeError("can_motor_control state length does not match configured DOF")
        self._state_cache = state
        self._state_cache_time = time.monotonic()
        self._last_positions = list(state[0])
        return state

    def read_joint_positions(self) -> list[float]:
        return list(self.refresh_state()[0])

    def read_joint_velocities(self) -> list[float]:
        return list(self.refresh_state()[1])

    def read_joint_efforts(self) -> list[float]:
        return list(self.refresh_state()[2])

    def read_state(self) -> dict[str, int]:
        return {
            "state": 1 if self._enabled else 0,
            "mode": list(ControlMode).index(self._control_mode),
        }

    def read_error(self) -> tuple[int, str]:
        if self._arm is None:
            return 0, ""
        faults = []
        for spec in self._motor_specs:
            motor = self._arm[spec.name]
            fault = getattr(motor, "fault", None)
            if fault is not None:
                faults.append(f"{spec.name}: {fault}")
        if not faults:
            return 0, ""
        return 1, "; ".join(faults)

    def write_joint_positions(self, positions: list[float], velocity: float = 1.0) -> bool:
        if self._arm is None or self._robot is None or not self._enabled:
            return False
        if len(positions) != self._dof:
            return False
        velocity = max(0.0, min(1.0, velocity))
        if self._gravity_comp:
            try:
                q_current = self.read_joint_positions()
                tau = self.compute_gravity_torques(q_current)
            except RuntimeError:
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
        if self._arm is None or self._robot is None or not self._enabled:
            return False
        if len(efforts) != self._dof:
            return False
        q = self._last_positions if self._last_positions is not None else self.read_joint_positions()
        return self.write_mit_commands(
            q=q,
            dq=self._zero_vector(),
            kp=self._zero_vector(),
            kd=self._zero_vector(),
            tau=efforts,
        )

    def write_gravity_compensation(self, damping: float | list[float] = 0.0) -> bool:
        try:
            q, dq, _ = self.refresh_state(force=True)
            tau = self.compute_gravity_torques(q)
        except Exception as exc:
            logger.warning(f"Skipping DMMotor gravity compensation due to invalid state: {exc}")
            return False
        kd = [float(damping)] * self._dof if isinstance(damping, int | float) else list(damping)
        return self.write_mit_commands(
            q=q, dq=dq, kp=self._zero_vector(), kd=kd, tau=tau
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
        if self._arm is None or self._robot is None or not self._enabled:
            return False
        rows = self._mit_command_rows(q=q, dq=dq, kp=kp, kd=kd, tau=tau)
        cmds = np.array([(row[2], row[3], row[0], row[1], row[4]) for row in rows], dtype=np.float64)
        self._arm.mit_control(cmds)
        self._robot.tick(self._tick_deadline_us)
        self._state_cache = None
        self._last_positions = list(q)
        self._control_mode = (
            ControlMode.TORQUE if all(k == 0.0 for k in kp) else ControlMode.POSITION
        )
        return True

    def write_stop(self) -> bool:
        if self._arm is None or self._robot is None:
            return False
        if self._gravity_comp and self._enabled:
            try:
                q_now = self.read_joint_positions()
            except RuntimeError:
                return False
            tau = self.compute_gravity_torques(q_now)
            return self.write_mit_commands(
                q=q_now,
                dq=self._zero_vector(),
                kp=list(self._kp),
                kd=list(self._kd),
                tau=tau,
            )
        try:
            self._robot.disable()
        except Exception as exc:
            logger.warning(f"DMMotorArm {self._hardware_id} stop disable failed: {exc}")
            return False
        self._enabled = False
        return True

    def write_enable(self, enable: bool) -> bool:
        if self._robot is None:
            return False
        try:
            if enable:
                self._robot.enable()
            else:
                self._robot.disable()
        except Exception as exc:
            logger.error(f"DMMotorArm {self._hardware_id} enable={enable} failed: {exc}")
            return False
        self._enabled = enable
        return True

    def write_clear_errors(self) -> bool:
        if self._robot is None:
            return False
        try:
            self._robot.disable()
            self._robot.enable()
        except Exception as exc:
            logger.error(f"DMMotorArm {self._hardware_id} clear errors failed: {exc}")
            return False
        self._enabled = True
        return True


def register(registry: AdapterRegistry) -> None:
    registry.register("dm_motor_arm", DMMotorArm)


__all__ = ["DMMotorArm", "DMMotorBindingUnavailableError", "DMMotorSpecConfig", "register"]

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

"""Simulation-agnostic manipulator interface."""

from __future__ import annotations

import logging
import math
from typing import TYPE_CHECKING

from dimos.hardware.manipulators.spec import ControlMode, JointLimits, ManipulatorInfo
from dimos.msgs.sensor_msgs import JointState

if TYPE_CHECKING:
    from dimos.simulation.engines.base import SimulationEngine


class SimManipInterface:
    """Adapter wrapper around a simulation engine to provide a uniform manipulator API."""

    def __init__(self, engine: SimulationEngine) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)
        self._engine = engine
        self._joint_names = list(engine.joint_names)
        self._dof = len(self._joint_names)
        self._connected = False
        self._servos_enabled = False
        self._control_mode = ControlMode.POSITION
        self._error_code = 0
        self._error_message = ""

    def connect(self) -> bool:
        """Connect to the simulation engine."""
        try:
            self.logger.info("Connecting to simulation engine...")
            if not self._engine.connect():
                self.logger.error("Failed to connect to simulation engine")
                return False
            if self._engine.connected:
                self._connected = True
                self._servos_enabled = True
                self._joint_names = list(self._engine.joint_names)
                self._dof = len(self._joint_names)
                self.logger.info(
                    "Successfully connected to simulation",
                    extra={"dof": self._dof},
                )
                return True
            self.logger.error("Failed to connect to simulation engine")
            return False
        except Exception as exc:
            self.logger.error(f"Sim connection failed: {exc}")
            return False

    def disconnect(self) -> bool:
        """Disconnect from simulation."""
        try:
            return self._engine.disconnect()
        except Exception as exc:
            self._connected = False
            self.logger.error(f"Sim disconnection failed: {exc}")
            return False

    def is_connected(self) -> bool:
        return bool(self._connected and self._engine.connected)

    def get_info(self) -> ManipulatorInfo:
        vendor = "Simulation"
        model = "Simulation"
        dof = self._dof
        return ManipulatorInfo(
            vendor=vendor,
            model=model,
            dof=dof,
            firmware_version=None,
            serial_number=None,
        )

    def get_dof(self) -> int:
        return self._dof

    def get_joint_names(self) -> list[str]:
        return list(self._joint_names)

    def get_limits(self) -> JointLimits:
        lower = [-math.pi] * self._dof
        upper = [math.pi] * self._dof
        max_vel_rad = math.radians(180.0)
        return JointLimits(
            position_lower=lower,
            position_upper=upper,
            velocity_max=[max_vel_rad] * self._dof,
        )

    def set_control_mode(self, mode: ControlMode) -> bool:
        self._control_mode = mode
        return True

    def get_control_mode(self) -> ControlMode:
        return self._control_mode

    def read_joint_positions(self) -> list[float]:
        positions = self._engine.read_joint_positions()
        return positions[: self._dof]

    def read_joint_velocities(self) -> list[float]:
        velocities = self._engine.read_joint_velocities()
        return velocities[: self._dof]

    def read_joint_efforts(self) -> list[float]:
        efforts = self._engine.read_joint_efforts()
        return efforts[: self._dof]

    def read_state(self) -> dict[str, int]:
        velocities = self.read_joint_velocities()
        is_moving = any(abs(v) > 1e-4 for v in velocities)
        mode_int = list(ControlMode).index(self._control_mode)
        return {
            "state": 1 if is_moving else 0,
            "mode": mode_int,
        }

    def read_error(self) -> tuple[int, str]:
        return self._error_code, self._error_message

    def write_joint_positions(self, positions: list[float]) -> bool:
        if not self._servos_enabled:
            return False
        self._control_mode = ControlMode.POSITION
        self._engine.write_joint_command(JointState(position=positions[: self._dof]))
        return True

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        if not self._servos_enabled:
            return False
        self._control_mode = ControlMode.VELOCITY
        self._engine.write_joint_command(JointState(velocity=velocities[: self._dof]))
        return True

    def write_joint_efforts(self, efforts: list[float]) -> bool:
        if not self._servos_enabled:
            return False
        self._control_mode = ControlMode.TORQUE
        self._engine.write_joint_command(JointState(effort=efforts[: self._dof]))
        return True

    def write_stop(self) -> bool:
        self._engine.hold_current_position()
        return True

    def write_enable(self, enable: bool) -> bool:
        self._servos_enabled = enable
        return True

    def read_enabled(self) -> bool:
        return self._servos_enabled

    def write_clear_errors(self) -> bool:
        self._error_code = 0
        self._error_message = ""
        return True

    def read_cartesian_position(self) -> dict[str, float] | None:
        return None

    def write_cartesian_position(
        self,
        pose: dict[str, float],
        velocity: float = 1.0,
    ) -> bool:
        _pose = pose
        _velocity = velocity
        return False

    def read_gripper_position(self) -> float | None:
        return None

    def write_gripper_position(self, position: float) -> bool:
        _ = position
        return False

    def read_force_torque(self) -> list[float] | None:
        return None


__all__ = [
    "SimManipInterface",
]

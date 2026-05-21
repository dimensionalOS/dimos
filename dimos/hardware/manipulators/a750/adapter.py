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

"""A-750 adapter registration and protocol scaffold."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from dimos.hardware.manipulators.registry import AdapterRegistry

from dimos.hardware.manipulators.spec import (
    ControlMode,
    JointLimits,
    ManipulatorInfo,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

GRIPPER_MAX_OPENING_M = 0.06


class A750Adapter:
    """A-750 manipulator adapter.

    The adapter is registered under ``a750`` and has the same runtime shape as
    the other manipulator adapters. This is currently a tracing stub: it accepts
    any device path and prints whenever its methods are called.
    """

    def __init__(
        self,
        address: str = "/dev/ttyACM0",
        dof: int = 6,
        initial_positions: list[float] | None = None,
        **_: object,
    ) -> None:
        if dof != 6:
            raise ValueError(f"A750Adapter only supports 6 DOF (got {dof})")
        self._device_path = address or "/dev/ttyACM0"
        self._dof = dof
        self._positions = list(initial_positions) if initial_positions is not None else [0.0] * dof
        self._connected = False
        self._enabled = False
        self._control_mode = ControlMode.POSITION
        self._error_code = 0
        self._error_message = ""
        self._robot: Any = None

    def connect(self) -> bool:
        """Connect to the A-750 serial device."""
        try:
            import a750_control
        except ImportError as exc:
            self._set_error(1, "a750_control is not installed in this Python environment")
            logger.error(f"{self._error_message}: {exc}")
            return False

        try:
            self._robot = a750_control.Robot(self._device_path)
            self._robot.connect()
            self._connected = True
        except Exception as exc:
            logger.error(f"Failed to connect to A-750: {exc}")
            self._connected = False
            self._set_error(2, str(exc))
            return False

        self._set_error(0, "")
        logger.info(f"Connected to A-750 at {self._device_path}")
        return self._connected

    def disconnect(self) -> None:
        """Disconnect from hardware."""
        if self._robot is not None:
            self._robot.disconnect()
        self._connected = False
        self._enabled = False

    def is_connected(self) -> bool:
        """Check if connected."""
        self._trace("is_connected")
        if self._robot is not None:
            self._connected = bool(self._robot.is_connected())
        return self._connected

    def get_info(self) -> ManipulatorInfo:
        """Get A-750 information."""
        self._trace("get_info")
        return ManipulatorInfo(vendor="Dobkin", model="A-750", dof=self._dof)

    def get_dof(self) -> int:
        """Get degrees of freedom."""
        self._trace("get_dof")
        return self._dof

    def get_limits(self) -> JointLimits:
        """Get approximate joint limits."""
        self._trace("get_limits")
        return JointLimits(
            position_lower=[-math.pi] * self._dof,
            position_upper=[math.pi] * self._dof,
            velocity_max=[math.pi] * self._dof,
        )

    def set_control_mode(self, mode: ControlMode) -> bool:
        """Set the cached control mode."""
        self._trace("set_control_mode", mode=mode.value)
        self._control_mode = mode
        return True

    def get_control_mode(self) -> ControlMode:
        """Get current control mode."""
        self._trace("get_control_mode")
        return self._control_mode

    def read_joint_positions(self) -> list[float]:
        """Read current joint positions in radians."""
        if not self._connected:
            raise RuntimeError("Not connected")

        state = self._robot.get_current_state()
        return [
            state.joint1.pos_rad,
            state.joint2.pos_rad,
            state.joint3.pos_rad,
            state.joint4.pos_rad,
            state.joint5.pos_rad,
            state.joint6.pos_rad,
        ]

    def read_joint_velocities(self) -> list[float]:
        """Read current joint velocities in radians/second."""
        if not self._connected:
            raise RuntimeError("Not connected")

        state = self._robot.get_current_state()
        return [
            state.joint1.vel_rads,
            state.joint2.vel_rads,
            state.joint3.vel_rads,
            state.joint4.vel_rads,
            state.joint5.vel_rads,
            state.joint6.vel_rads,
        ]

    def read_joint_efforts(self) -> list[float]:
        """Read current joint efforts in Nm."""
        if not self._connected:
            raise RuntimeError("Not connected")

        state = self._robot.get_current_state()
        return [
            state.joint1.torque_nm,
            state.joint2.torque_nm,
            state.joint3.torque_nm,
            state.joint4.torque_nm,
            state.joint5.torque_nm,
            state.joint6.torque_nm,
        ]

    def read_state(self) -> dict[str, int]:
        """Read robot state."""
        self._trace("read_state")
        return {
            "state": 0 if self._enabled else 1,
            "mode": 0,
            "error_code": self._error_code,
        }

    def read_error(self) -> tuple[int, str]:
        """Read error code and message."""
        self._trace("read_error")
        return self._error_code, self._error_message

    def write_joint_positions(
        self,
        positions: list[float],
        velocity: float = 1.0,
    ) -> bool:
        """Command joint positions."""
        assert len(positions) == self._dof

        if not self._enabled:
            return False

        self._robot.command_joint_positions(positions, velocity)
        return True

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        """Command joint velocities."""
        self._trace("write_joint_velocities", velocities=velocities)
        return self._connected and len(velocities) == self._dof

    def write_stop(self) -> bool:
        """Stop all motion."""
        self._trace("write_stop")
        return self._connected

    def write_enable(self, enable: bool) -> bool:
        """Enable or disable servos."""
        if not self._connected:
            return False

        if self._enabled == enable:
            return True

        if enable:
            self._robot.start_control_loop()
        else:
            self._robot.stop_control_loop()

        self._enabled = enable

        return True

    def read_enabled(self) -> bool:
        """Check if servos are enabled."""
        self._trace("read_enabled")
        return self._enabled

    def write_clear_errors(self) -> bool:
        """Clear error state."""
        self._trace("write_clear_errors")
        self._set_error(0, "")
        return True

    def read_cartesian_position(self) -> dict[str, float] | None:
        """Read end-effector pose if supported."""
        self._trace("read_cartesian_position")
        return None

    def write_cartesian_position(
        self,
        pose: dict[str, float],
        velocity: float = 1.0,
    ) -> bool:
        """Command end-effector pose if supported."""
        self._trace("write_cartesian_position", pose=pose, velocity=velocity)
        return self._connected

    def read_gripper_position(self) -> float | None:
        """Read gripper finger position as offset from center in meters."""
        if not self._connected:
            return None

        state = self._robot.get_current_state()
        return state.gripper.pos_m  # type: ignore[no-any-return]

    def write_gripper_position(self, position: float) -> bool:
        """Command gripper position."""
        if not self._enabled:
            return False

        self._robot.command_gripper_position(position)
        return True

    def read_force_torque(self) -> list[float] | None:
        """Read F/T sensor data if supported."""
        self._trace("read_force_torque")
        return None

    def _set_error(self, code: int, message: str) -> None:
        self._error_code = code
        self._error_message = message

    def _trace(self, method: str, **kwargs: object) -> None:
        details = ", ".join(f"{key}={value!r}" for key, value in kwargs.items())
        suffix = f"({details})" if details else "()"
        logger.info(f"A750Adapter.{method}{suffix}")


def register(registry: AdapterRegistry) -> None:
    """Register this adapter with the registry."""
    registry.register("a750", A750Adapter)


__all__ = ["A750Adapter"]

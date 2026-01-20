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

"""Mock adapter for testing - no hardware required.

Usage:
    >>> from dimos.hardware.manipulators.xarm import XArm
    >>> from dimos.hardware.manipulators.mock import MockAdapter
    >>> arm = XArm(adapter=MockAdapter())
    >>> arm.start()  # No hardware!
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from dimos.hardware.manipulators.registry import AdapterRegistry

from dimos.hardware.manipulators.spec import (
    ControlMode,
    JointLimits,
    ManipulatorInfo,
)


class MockAdapter:
    """Fake adapter for unit tests.

    Implements ManipulatorAdapter protocol with in-memory state.
    Useful for:
    - Unit testing driver logic without hardware
    - Integration testing with predictable behavior
    - Development without physical robot
    """

    def __init__(self, dof: int = 6, **_: object) -> None:
        self._dof = dof
        self._positions = [0.0] * dof
        self._velocities = [0.0] * dof
        self._efforts = [0.0] * dof
        self._enabled = False
        self._connected = False
        self._control_mode = ControlMode.POSITION
        self._cartesian_position: dict[str, float] = {
            "x": 0.3,
            "y": 0.0,
            "z": 0.3,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }
        self._gripper_position: float = 0.0
        self._error_code: int = 0
        self._error_message: str = ""

    # =========================================================================
    # Connection
    # =========================================================================

    def connect(self) -> bool:
        """Simulate connection."""
        self._connected = True
        return True

    def disconnect(self) -> None:
        """Simulate disconnection."""
        self._connected = False

    def is_connected(self) -> bool:
        """Check mock connection status."""
        return self._connected

    # =========================================================================
    # Info
    # =========================================================================

    def get_info(self) -> ManipulatorInfo:
        """Return mock info."""
        return ManipulatorInfo(
            vendor="Mock",
            model="MockArm",
            dof=self._dof,
            firmware_version="1.0.0",
            serial_number="MOCK-001",
        )

    def get_dof(self) -> int:
        """Return DOF."""
        return self._dof

    def get_limits(self) -> JointLimits:
        """Return mock joint limits."""
        return JointLimits(
            position_lower=[-math.pi] * self._dof,
            position_upper=[math.pi] * self._dof,
            velocity_max=[1.0] * self._dof,
        )

    # =========================================================================
    # Control Mode
    # =========================================================================

    def set_control_mode(self, mode: ControlMode) -> bool:
        """Set mock control mode."""
        self._control_mode = mode
        return True

    def get_control_mode(self) -> ControlMode:
        """Get mock control mode."""
        return self._control_mode

    # =========================================================================
    # State Reading
    # =========================================================================

    def read_joint_positions(self) -> list[float]:
        """Return mock joint positions."""
        return self._positions.copy()

    def read_joint_velocities(self) -> list[float]:
        """Return mock joint velocities."""
        return self._velocities.copy()

    def read_joint_efforts(self) -> list[float]:
        """Return mock joint efforts."""
        return self._efforts.copy()

    def read_state(self) -> dict[str, int]:
        """Return mock state."""
        # Use index of control mode as int (0=position, 1=velocity, etc.)
        mode_int = list(ControlMode).index(self._control_mode)
        return {
            "state": 0 if self._enabled else 1,
            "mode": mode_int,
        }

    def read_error(self) -> tuple[int, str]:
        """Return mock error."""
        return self._error_code, self._error_message

    # =========================================================================
    # Motion Control
    # =========================================================================

    def write_joint_positions(
        self,
        positions: list[float],
        velocity: float = 1.0,
    ) -> bool:
        """Set mock joint positions (instant move)."""
        if len(positions) != self._dof:
            return False
        self._positions = list(positions)
        return True

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        """Set mock joint velocities."""
        if len(velocities) != self._dof:
            return False
        self._velocities = list(velocities)
        return True

    def write_stop(self) -> bool:
        """Stop mock motion."""
        self._velocities = [0.0] * self._dof
        return True

    # =========================================================================
    # Servo Control
    # =========================================================================

    def write_enable(self, enable: bool) -> bool:
        """Enable/disable mock servos."""
        self._enabled = enable
        return True

    def read_enabled(self) -> bool:
        """Check mock servo state."""
        return self._enabled

    def write_clear_errors(self) -> bool:
        """Clear mock errors."""
        self._error_code = 0
        self._error_message = ""
        return True

    # =========================================================================
    # Cartesian Control (Optional)
    # =========================================================================

    def read_cartesian_position(self) -> dict[str, float] | None:
        """Return mock cartesian position."""
        return self._cartesian_position.copy()

    def write_cartesian_position(
        self,
        pose: dict[str, float],
        velocity: float = 1.0,
    ) -> bool:
        """Set mock cartesian position."""
        self._cartesian_position.update(pose)
        return True

    # =========================================================================
    # Gripper (Optional)
    # =========================================================================

    def read_gripper_position(self) -> float | None:
        """Return mock gripper position."""
        return self._gripper_position

    def write_gripper_position(self, position: float) -> bool:
        """Set mock gripper position."""
        self._gripper_position = position
        return True

    # =========================================================================
    # Force/Torque (Optional)
    # =========================================================================

    def read_force_torque(self) -> list[float] | None:
        """Return mock F/T sensor data (not supported in mock)."""
        return None

    # =========================================================================
    # Test Helpers (not part of Protocol)
    # =========================================================================

    def set_error(self, code: int, message: str) -> None:
        """Inject an error for testing error handling."""
        self._error_code = code
        self._error_message = message

    def set_positions(self, positions: list[float]) -> None:
        """Set positions directly for testing."""
        self._positions = list(positions)

    def set_efforts(self, efforts: list[float]) -> None:
        """Set efforts directly for testing."""
        self._efforts = list(efforts)


def register(registry: AdapterRegistry) -> None:
    """Register this adapter with the registry."""
    registry.register("mock", MockAdapter)


__all__ = ["MockAdapter"]

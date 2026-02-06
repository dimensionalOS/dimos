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

"""Manipulator specifications: Protocol and shared types.

This file defines:
1. Shared enums and dataclasses used by all arms
2. ManipulatorAdapter Protocol that adapters must implement

Note: No ABC for drivers. Each arm implements its own driver
with full control over threading and logic.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Protocol, runtime_checkable

from dimos.msgs.geometry_msgs import Quaternion, Transform, Vector3

# ============================================================================
# SHARED TYPES
# ============================================================================


class DriverStatus(Enum):
    """Status returned by driver operations."""

    DISCONNECTED = "disconnected"
    CONNECTED = "connected"
    ENABLED = "enabled"
    MOVING = "moving"
    ERROR = "error"


class ControlMode(Enum):
    """Control modes for manipulator."""

    POSITION = "position"  # Planned position control (slower, smoother)
    SERVO_POSITION = "servo_position"  # High-freq joint position streaming (100Hz+)
    VELOCITY = "velocity"
    TORQUE = "torque"
    CARTESIAN = "cartesian"
    CARTESIAN_VELOCITY = "cartesian_velocity"
    IMPEDANCE = "impedance"


@dataclass
class ManipulatorInfo:
    """Information about the manipulator."""

    vendor: str
    model: str
    dof: int
    firmware_version: str | None = None
    serial_number: str | None = None


@dataclass
class JointLimits:
    """Joint position and velocity limits."""

    position_lower: list[float]  # radians
    position_upper: list[float]  # radians
    velocity_max: list[float]  # rad/s


def default_base_transform() -> Transform:
    """Default identity transform for arm mounting."""
    return Transform(
        translation=Vector3(0.0, 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )


# ============================================================================
# ADAPTER PROTOCOL
# ============================================================================


@runtime_checkable
class ManipulatorAdapter(Protocol):
    """Protocol for hardware-specific IO.

    Implement this per vendor SDK. All methods use SI units:
    - Angles: radians
    - Angular velocity: rad/s
    - Torque: Nm
    - Position: meters
    - Force: Newtons
    """

    # --- Connection ---

    def connect(self) -> bool:
        """Connect to hardware. Returns True on success."""
        ...

    def disconnect(self) -> None:
        """Disconnect from hardware."""
        ...

    def is_connected(self) -> bool:
        """Check if connected."""
        ...

    # --- Info ---

    def get_info(self) -> ManipulatorInfo:
        """Get manipulator info (vendor, model, DOF)."""
        ...

    def get_dof(self) -> int:
        """Get degrees of freedom."""
        ...

    def get_limits(self) -> JointLimits:
        """Get joint limits."""
        ...

    # --- Control Mode ---

    def set_control_mode(self, mode: ControlMode) -> bool:
        """Set control mode (position, velocity, torque, cartesian, etc).

        Args:
            mode: Target control mode

        Returns:
            True if mode switch successful, False otherwise

        Note: Some arms (like XArm) may accept commands in any mode,
        while others (like Piper) require explicit mode switching.
        """
        ...

    def get_control_mode(self) -> ControlMode:
        """Get current control mode.

        Returns:
            Current control mode
        """
        ...

    # --- State Reading ---

    def read_joint_positions(self) -> list[float]:
        """Read current joint positions (radians)."""
        ...

    def read_joint_velocities(self) -> list[float]:
        """Read current joint velocities (rad/s)."""
        ...

    def read_joint_efforts(self) -> list[float]:
        """Read current joint efforts (Nm)."""
        ...

    def read_state(self) -> dict[str, int]:
        """Read robot state (mode, state code, etc)."""
        ...

    def read_error(self) -> tuple[int, str]:
        """Read error code and message. (0, '') means no error."""
        ...

    # --- Motion Control (Joint Space) ---

    def write_joint_positions(
        self,
        positions: list[float],
        velocity: float = 1.0,
    ) -> bool:
        """Command joint positions (radians). Returns success."""
        ...

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        """Command joint velocities (rad/s). Returns success."""
        ...

    def write_stop(self) -> bool:
        """Stop all motion immediately."""
        ...

    # --- Servo Control ---

    def write_enable(self, enable: bool) -> bool:
        """Enable or disable servos. Returns success."""
        ...

    def read_enabled(self) -> bool:
        """Check if servos are enabled."""
        ...

    def write_clear_errors(self) -> bool:
        """Clear error state. Returns success."""
        ...

    # --- Optional: Cartesian Control ---
    # Return None/False if not supported

    def read_cartesian_position(self) -> dict[str, float] | None:
        """Read end-effector pose.

        Returns:
            Dict with keys: x, y, z (meters), roll, pitch, yaw (radians)
            None if not supported
        """
        ...

    def write_cartesian_position(
        self,
        pose: dict[str, float],
        velocity: float = 1.0,
    ) -> bool:
        """Command end-effector pose.

        Args:
            pose: Dict with keys: x, y, z (meters), roll, pitch, yaw (radians)
            velocity: Speed as fraction of max (0-1)

        Returns:
            True if command accepted, False if not supported
        """
        ...

    # --- Optional: Gripper ---

    def read_gripper_position(self) -> float | None:
        """Read gripper position (meters). None if no gripper."""
        ...

    def write_gripper_position(self, position: float) -> bool:
        """Command gripper position. False if no gripper."""
        ...

    # --- Optional: Force/Torque Sensor ---

    def read_force_torque(self) -> list[float] | None:
        """Read F/T sensor [fx, fy, fz, tx, ty, tz]. None if no sensor."""
        ...


__all__ = [
    "ControlMode",
    "DriverStatus",
    "JointLimits",
    "ManipulatorAdapter",
    "ManipulatorInfo",
    "default_base_transform",
]

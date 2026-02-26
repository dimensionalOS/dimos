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

"""Hardware adapter for mobile base using Twist commands.

Maps joint commands [base_vx, base_vy, base_wz] to Twist messages
for integration with Unitree GO2 and other mobile robots.
"""

from typing import Protocol

from dimos.hardware.manipulators.spec import (
    ControlMode,
    JointLimits,
    ManipulatorInfo,
)
from dimos.msgs.geometry_msgs import Twist, Vector3
from dimos.utils.logging_config import setup_logger


logger = setup_logger()


class TwistCommandCallback(Protocol):
    """Protocol for receiving Twist commands."""

    def __call__(self, twist: Twist) -> None: ...


class BaseTwistAdapter:
    """Adapter that converts joint commands to Twist for mobile base.

    This adapter implements the ManipulatorAdapter protocol but outputs
    Twist commands instead of direct hardware commands. The Twist is
    sent to a callback (e.g., GO2Connection.move()).

    Joint mapping:
        base_vx -> Twist.linear.x (forward velocity)
        base_vy -> Twist.linear.y (lateral velocity)
        base_wz -> Twist.angular.z (yaw velocity)
    """

    def __init__(self, twist_callback: TwistCommandCallback):
        """Initialize base twist adapter.

        Args:
            twist_callback: Callback function to receive Twist commands
        """
        self._twist_callback = twist_callback
        self._current_mode: ControlMode = ControlMode.VELOCITY
        self._connected = True

    # =========================================================================
    # ManipulatorAdapter Protocol Implementation
    # =========================================================================

    # --- Connection ---

    def connect(self) -> bool:
        """Connect to hardware (always succeeds for callback-based adapter)."""
        self._connected = True
        return True

    def disconnect(self) -> None:
        """Disconnect from hardware."""
        self._connected = False

    def is_connected(self) -> bool:
        """Check if connected."""
        return self._connected

    # --- Info ---

    def get_info(self) -> ManipulatorInfo:
        """Get manipulator info."""
        return ManipulatorInfo(
            vendor="DIMOS",
            model="BaseTwistAdapter",
            dof=3,
        )

    def get_dof(self) -> int:
        """Get degrees of freedom."""
        return 3

    def get_limits(self) -> JointLimits:
        """Get joint limits for base velocities.

        - Linear: ±2.2 m/s (typical max for Unitree GO2)
        - Angular: ±2.0 rad/s
        """
        return JointLimits(
            position_lower=[-2.2, -2.2, -2.0],   # base_vx, base_vy, base_wz
            position_upper=[2.2, 2.2, 2.0],
            velocity_max=[2.2, 2.2, 2.0],
        )

    # --- Control Mode ---

    def set_control_mode(self, mode: ControlMode) -> bool:
        """Set control mode (only VELOCITY supported)."""
        if mode == ControlMode.VELOCITY:
            self._current_mode = mode
            return True
        return False

    def get_control_mode(self) -> ControlMode:
        """Get current control mode."""
        return self._current_mode

    # --- State Reading ---

    def read_joint_positions(self) -> list[float]:
        """Read joint positions (not applicable for base, returns zeros)."""
        return [0.0, 0.0, 0.0]

    def read_joint_velocities(self) -> list[float]:
        """Read joint velocities (not available, returns zeros)."""
        return [0.0, 0.0, 0.0]

    def read_joint_efforts(self) -> list[float]:
        """Read joint efforts (not applicable, returns zeros)."""
        return [0.0, 0.0, 0.0]

    def read_state(self) -> dict[str, int]:
        """Minimal state dictionary (no real hardware state)."""
        return {}

    def read_error(self) -> tuple[int, str]:
        """No error reporting for this simple adapter."""
        return 0, ""

    # --- Motion Control (Joint Space) ---

    def write_joint_positions(self, positions: list[float], velocity: float = 1.0) -> bool:
        """Write joint positions (not supported for base)."""
        return False

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        """Write joint velocities as Twist command.

        Args:
            velocities: [base_vx, base_vy, base_wz] in m/s and rad/s

        Returns:
            True if command sent successfully
        """
        if not self._connected:
            return False

        if len(velocities) != 3:
            return False

        # Convert to Twist
        twist = Twist(
            linear=Vector3(velocities[0], velocities[1], 0.0),  # vx, vy, vz
            angular=Vector3(0.0, 0.0, velocities[2]),  # wx, wy, wz
        )

        # Send via callback
        try:
            self._twist_callback(twist)
            return True
        except Exception as e:
            logger.error(f"BaseTwistAdapter error: {e}")
            return False

    def write_stop(self) -> bool:
        """Emergency stop (send zero velocities)."""
        return self.write_joint_velocities([0.0, 0.0, 0.0])

    # --- Servo Control (stubs to satisfy ManipulatorAdapter) ---

    def write_enable(self, enable: bool) -> bool:
        """Enable/disable (for base, just toggles connected flag)."""
        self._connected = enable
        return True

    def read_enabled(self) -> bool:
        """Whether the adapter is considered enabled."""
        return self._connected

    def write_clear_errors(self) -> bool:
        """No error state to clear for this adapter."""
        return True

    # --- Optional Cartesian / Gripper / Force-Torque (unsupported) ---

    def read_cartesian_position(self) -> dict[str, float] | None:
        return None

    def write_cartesian_position(
        self,
        pose: dict[str, float],
        velocity: float = 1.0,
    ) -> bool:
        return False

    def read_gripper_position(self) -> float | None:
        return None

    def write_gripper_position(self, position: float) -> bool:
        return False

    def read_force_torque(self) -> list[float] | None:
        return None
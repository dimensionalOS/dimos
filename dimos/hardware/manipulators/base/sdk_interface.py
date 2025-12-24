# Copyright 2025 Dimensional Inc.
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

"""Base SDK interface that all manipulator SDK wrappers must implement.

This interface defines the standard methods and units that all SDK wrappers
must provide, ensuring consistent behavior across different manipulator types.

Standard Units:
- Angles: radians
- Angular velocity: rad/s
- Linear position: meters
- Linear velocity: m/s
- Force: Newtons
- Torque: Nm
- Time: seconds
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any


@dataclass
class ManipulatorInfo:
    """Information about the manipulator."""

    vendor: str
    model: str
    dof: int
    firmware_version: str | None = None
    serial_number: str | None = None


class BaseManipulatorSDK(ABC):
    """Abstract base class for manipulator SDK wrappers.

    All SDK wrappers must implement this interface to ensure compatibility
    with the standard components. Methods should handle unit conversions
    internally to always work with standard units.
    """

    # ============= Connection Management =============

    @abstractmethod
    def connect(self, config: dict[str, Any]) -> bool:
        """Establish connection to the manipulator.

        Args:
            config: Configuration dict with connection parameters
                   (e.g., ip, port, can_interface, etc.)

        Returns:
            True if connection successful, False otherwise
        """
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """Disconnect from the manipulator.

        Should cleanly close all connections and free resources.
        """
        pass

    @abstractmethod
    def is_connected(self) -> bool:
        """Check if currently connected to the manipulator.

        Returns:
            True if connected, False otherwise
        """
        pass

    # ============= Joint State Query =============

    @abstractmethod
    def get_joint_positions(self) -> list[float]:
        """Get current joint positions.

        Returns:
            Joint positions in RADIANS
        """
        pass

    @abstractmethod
    def get_joint_velocities(self) -> list[float]:
        """Get current joint velocities.

        Returns:
            Joint velocities in RAD/S
        """
        pass

    @abstractmethod
    def get_joint_efforts(self) -> list[float]:
        """Get current joint efforts/torques.

        Returns:
            Joint efforts in Nm (torque) or N (force)
        """
        pass

    # ============= Joint Motion Control =============

    @abstractmethod
    def set_joint_positions(
        self,
        positions: list[float],
        velocity: float = 1.0,
        acceleration: float = 1.0,
        wait: bool = False,
    ) -> bool:
        """Move joints to target positions.

        Args:
            positions: Target positions in RADIANS
            velocity: Max velocity as fraction of maximum (0-1)
            acceleration: Max acceleration as fraction of maximum (0-1)
            wait: If True, block until motion completes

        Returns:
            True if command accepted, False otherwise
        """
        pass

    @abstractmethod
    def set_joint_velocities(self, velocities: list[float]) -> bool:
        """Set joint velocity targets.

        Args:
            velocities: Target velocities in RAD/S

        Returns:
            True if command accepted, False otherwise
        """
        pass

    @abstractmethod
    def set_joint_efforts(self, efforts: list[float]) -> bool:
        """Set joint effort/torque targets.

        Args:
            efforts: Target efforts in Nm (torque) or N (force)

        Returns:
            True if command accepted, False otherwise
        """
        pass

    @abstractmethod
    def stop_motion(self) -> bool:
        """Stop all ongoing motion immediately.

        Returns:
            True if stop successful, False otherwise
        """
        pass

    # ============= Servo Control =============

    @abstractmethod
    def enable_servos(self) -> bool:
        """Enable motor control (servos/brakes released).

        Returns:
            True if servos enabled, False otherwise
        """
        pass

    @abstractmethod
    def disable_servos(self) -> bool:
        """Disable motor control (servos/brakes engaged).

        Returns:
            True if servos disabled, False otherwise
        """
        pass

    @abstractmethod
    def are_servos_enabled(self) -> bool:
        """Check if servos are currently enabled.

        Returns:
            True if enabled, False if disabled
        """
        pass

    # ============= System State =============

    @abstractmethod
    def get_robot_state(self) -> dict[str, Any]:
        """Get current robot state information.

        Returns:
            Dict with at least these keys:
            - 'state': int (0=idle, 1=moving, 2=error, 3=e-stop)
            - 'mode': int (0=position, 1=velocity, 2=torque)
            - 'error_code': int (0 = no error)
            - 'is_moving': bool
        """
        pass

    @abstractmethod
    def get_error_code(self) -> int:
        """Get current error code.

        Returns:
            Error code (0 = no error)
        """
        pass

    @abstractmethod
    def get_error_message(self) -> str:
        """Get human-readable error message.

        Returns:
            Error message string (empty if no error)
        """
        pass

    @abstractmethod
    def clear_errors(self) -> bool:
        """Clear any error states.

        Returns:
            True if errors cleared, False otherwise
        """
        pass

    @abstractmethod
    def emergency_stop(self) -> bool:
        """Execute emergency stop.

        Returns:
            True if e-stop executed, False otherwise
        """
        pass

    # ============= Information =============

    @abstractmethod
    def get_info(self) -> ManipulatorInfo:
        """Get manipulator information.

        Returns:
            ManipulatorInfo object with vendor, model, DOF, etc.
        """
        pass

    @abstractmethod
    def get_joint_limits(self) -> tuple[list[float], list[float]]:
        """Get joint position limits.

        Returns:
            Tuple of (lower_limits, upper_limits) in RADIANS
        """
        pass

    @abstractmethod
    def get_velocity_limits(self) -> list[float]:
        """Get joint velocity limits.

        Returns:
            Maximum velocities in RAD/S
        """
        pass

    @abstractmethod
    def get_acceleration_limits(self) -> list[float]:
        """Get joint acceleration limits.

        Returns:
            Maximum accelerations in RAD/S²
        """
        pass

    # ============= Optional Methods (Override if Supported) =============
    # These have default implementations that indicate feature not available

    def get_cartesian_position(self) -> dict[str, float] | None:
        """Get current end-effector pose.

        Returns:
            Dict with keys: x, y, z (meters), roll, pitch, yaw (radians)
            None if not supported
        """
        return None

    def set_cartesian_position(
        self,
        pose: dict[str, float],
        velocity: float = 1.0,
        acceleration: float = 1.0,
        wait: bool = False,
    ) -> bool:
        """Move end-effector to target pose.

        Args:
            pose: Target pose with keys: x, y, z (meters), roll, pitch, yaw (radians)
            velocity: Max velocity as fraction (0-1)
            acceleration: Max acceleration as fraction (0-1)
            wait: If True, block until motion completes

        Returns:
            False (not supported by default)
        """
        return False

    def get_cartesian_velocity(self) -> dict[str, float] | None:
        """Get current end-effector velocity.

        Returns:
            Dict with keys: vx, vy, vz (m/s), wx, wy, wz (rad/s)
            None if not supported
        """
        return None

    def set_cartesian_velocity(self, twist: dict[str, float]) -> bool:
        """Set end-effector velocity.

        Args:
            twist: Velocity with keys: vx, vy, vz (m/s), wx, wy, wz (rad/s)

        Returns:
            False (not supported by default)
        """
        return False

    def get_force_torque(self) -> list[float] | None:
        """Get force/torque sensor reading.

        Returns:
            List of [fx, fy, fz (N), tx, ty, tz (Nm)]
            None if not supported
        """
        return None

    def zero_force_torque(self) -> bool:
        """Zero the force/torque sensor.

        Returns:
            False (not supported by default)
        """
        return False

    def set_impedance_parameters(self, stiffness: list[float], damping: list[float]) -> bool:
        """Set impedance control parameters.

        Args:
            stiffness: Stiffness values [x, y, z, rx, ry, rz]
            damping: Damping values [x, y, z, rx, ry, rz]

        Returns:
            False (not supported by default)
        """
        return False

    def get_digital_inputs(self) -> dict[str, bool] | None:
        """Get digital input states.

        Returns:
            Dict of input_id: bool
            None if not supported
        """
        return None

    def set_digital_outputs(self, outputs: dict[str, bool]) -> bool:
        """Set digital output states.

        Args:
            outputs: Dict of output_id: bool

        Returns:
            False (not supported by default)
        """
        return False

    def get_analog_inputs(self) -> dict[str, float] | None:
        """Get analog input values.

        Returns:
            Dict of input_id: float
            None if not supported
        """
        return None

    def set_analog_outputs(self, outputs: dict[str, float]) -> bool:
        """Set analog output values.

        Args:
            outputs: Dict of output_id: float

        Returns:
            False (not supported by default)
        """
        return False

    def execute_trajectory(self, trajectory: list[dict[str, Any]], wait: bool = True) -> bool:
        """Execute a joint trajectory.

        Args:
            trajectory: List of waypoints, each with:
                       - 'positions': list[float] in radians
                       - 'velocities': Optional list[float] in rad/s
                       - 'time': float seconds from start
            wait: If True, block until trajectory completes

        Returns:
            False (not supported by default)
        """
        return False

    def stop_trajectory(self) -> bool:
        """Stop any executing trajectory.

        Returns:
            False (not supported by default)
        """
        return False

    def get_gripper_position(self) -> float | None:
        """Get gripper position.

        Returns:
            Position in meters (0=closed, max=fully open)
            None if no gripper
        """
        return None

    def set_gripper_position(self, position: float, force: float = 1.0) -> bool:
        """Set gripper position.

        Args:
            position: Target position in meters
            force: Gripping force as fraction (0-1)

        Returns:
            False (not supported by default)
        """
        return False

    def set_control_mode(self, mode: str) -> bool:
        """Set control mode.

        Args:
            mode: One of 'position', 'velocity', 'torque', 'impedance'

        Returns:
            False (not supported by default)
        """
        return False

    def get_control_mode(self) -> str | None:
        """Get current control mode.

        Returns:
            Current mode string or None if not supported
        """
        return None

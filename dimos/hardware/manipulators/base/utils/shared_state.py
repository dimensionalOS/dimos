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

"""Thread-safe shared state for manipulator drivers."""

from dataclasses import dataclass, field
from threading import Lock
import time
from typing import Any


@dataclass
class SharedState:
    """Thread-safe shared state for manipulator drivers.

    This class holds the current state of the manipulator that needs to be
    shared between multiple threads (state reader, command sender, publisher).
    All access should be protected by the lock.
    """

    # Thread synchronization
    lock: Lock = field(default_factory=Lock)

    # Joint state (current values from hardware)
    joint_positions: list[float] | None = None  # radians
    joint_velocities: list[float] | None = None  # rad/s
    joint_efforts: list[float] | None = None  # Nm

    # Joint targets (commanded values)
    target_positions: list[float] | None = None  # radians
    target_velocities: list[float] | None = None  # rad/s
    target_efforts: list[float] | None = None  # Nm

    # Cartesian state (if available)
    cartesian_position: dict[str, float] | None = None  # x,y,z,roll,pitch,yaw
    cartesian_velocity: dict[str, float] | None = None  # vx,vy,vz,wx,wy,wz

    # Cartesian targets
    target_cartesian_position: dict[str, float] | None = None
    target_cartesian_velocity: dict[str, float] | None = None

    # Force/torque sensor (if available)
    force_torque: list[float] | None = None  # [fx,fy,fz,tx,ty,tz]

    # System state
    robot_state: int = 0  # 0=idle, 1=moving, 2=error, 3=e-stop
    control_mode: int = 0  # 0=position, 1=velocity, 2=torque
    error_code: int = 0  # 0 = no error
    error_message: str = ""  # Human-readable error

    # Connection and enable status
    is_connected: bool = False
    is_enabled: bool = False
    is_moving: bool = False
    is_homed: bool = False

    # Gripper state (if available)
    gripper_position: float | None = None  # meters
    gripper_force: float | None = None  # Newtons

    # Timestamps
    last_state_update: float = 0.0
    last_command_sent: float = 0.0
    last_error_time: float = 0.0

    # Statistics
    state_read_count: int = 0
    command_sent_count: int = 0
    error_count: int = 0

    def update_joint_state(
        self,
        positions: list[float] | None = None,
        velocities: list[float] | None = None,
        efforts: list[float] | None = None,
    ) -> None:
        """Thread-safe update of joint state.

        Args:
            positions: Joint positions in radians
            velocities: Joint velocities in rad/s
            efforts: Joint efforts in Nm
        """
        with self.lock:
            if positions is not None:
                self.joint_positions = positions
            if velocities is not None:
                self.joint_velocities = velocities
            if efforts is not None:
                self.joint_efforts = efforts
            self.last_state_update = time.time()
            self.state_read_count += 1

    def update_robot_state(
        self,
        state: int | None = None,
        mode: int | None = None,
        error_code: int | None = None,
        error_message: str | None = None,
    ) -> None:
        """Thread-safe update of robot state.

        Args:
            state: Robot state code
            mode: Control mode code
            error_code: Error code (0 = no error)
            error_message: Human-readable error message
        """
        with self.lock:
            if state is not None:
                self.robot_state = state
            if mode is not None:
                self.control_mode = mode
            if error_code is not None:
                self.error_code = error_code
                if error_code != 0:
                    self.error_count += 1
                    self.last_error_time = time.time()
            if error_message is not None:
                self.error_message = error_message

    def update_cartesian_state(
        self, position: dict[str, float] | None = None, velocity: dict[str, float] | None = None
    ) -> None:
        """Thread-safe update of Cartesian state.

        Args:
            position: End-effector pose (x,y,z,roll,pitch,yaw)
            velocity: End-effector twist (vx,vy,vz,wx,wy,wz)
        """
        with self.lock:
            if position is not None:
                self.cartesian_position = position
            if velocity is not None:
                self.cartesian_velocity = velocity

    def set_target_joints(
        self,
        positions: list[float] | None = None,
        velocities: list[float] | None = None,
        efforts: list[float] | None = None,
    ) -> None:
        """Thread-safe update of joint targets.

        Args:
            positions: Target positions in radians
            velocities: Target velocities in rad/s
            efforts: Target efforts in Nm
        """
        with self.lock:
            if positions is not None:
                self.target_positions = positions
            if velocities is not None:
                self.target_velocities = velocities
            if efforts is not None:
                self.target_efforts = efforts
            self.last_command_sent = time.time()
            self.command_sent_count += 1

    def get_joint_state(
        self,
    ) -> tuple[list[float] | None, list[float] | None, list[float] | None]:
        """Thread-safe read of joint state.

        Returns:
            Tuple of (positions, velocities, efforts)
        """
        with self.lock:
            return (
                self.joint_positions.copy() if self.joint_positions else None,
                self.joint_velocities.copy() if self.joint_velocities else None,
                self.joint_efforts.copy() if self.joint_efforts else None,
            )

    def get_robot_state(self) -> dict[str, Any]:
        """Thread-safe read of robot state.

        Returns:
            Dict with state information
        """
        with self.lock:
            return {
                "state": self.robot_state,
                "mode": self.control_mode,
                "error_code": self.error_code,
                "error_message": self.error_message,
                "is_connected": self.is_connected,
                "is_enabled": self.is_enabled,
                "is_moving": self.is_moving,
                "last_update": self.last_state_update,
            }

    def get_statistics(self) -> dict[str, Any]:
        """Get statistics about state updates.

        Returns:
            Dict with statistics
        """
        with self.lock:
            return {
                "state_read_count": self.state_read_count,
                "command_sent_count": self.command_sent_count,
                "error_count": self.error_count,
                "last_state_update": self.last_state_update,
                "last_command_sent": self.last_command_sent,
                "last_error_time": self.last_error_time,
            }

    def clear_errors(self) -> None:
        """Clear error state."""
        with self.lock:
            self.error_code = 0
            self.error_message = ""

    def reset(self) -> None:
        """Reset all state to initial values."""
        with self.lock:
            self.joint_positions = None
            self.joint_velocities = None
            self.joint_efforts = None
            self.target_positions = None
            self.target_velocities = None
            self.target_efforts = None
            self.cartesian_position = None
            self.cartesian_velocity = None
            self.target_cartesian_position = None
            self.target_cartesian_velocity = None
            self.force_torque = None
            self.robot_state = 0
            self.control_mode = 0
            self.error_code = 0
            self.error_message = ""
            self.is_connected = False
            self.is_enabled = False
            self.is_moving = False
            self.is_homed = False
            self.gripper_position = None
            self.gripper_force = None
            self.last_state_update = 0.0
            self.last_command_sent = 0.0
            self.last_error_time = 0.0
            self.state_read_count = 0
            self.command_sent_count = 0
            self.error_count = 0

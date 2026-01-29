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

"""XArm adapter - implements ManipulatorAdapter protocol.

SDK Units: angles=degrees, distance=mm, velocity=deg/s
DimOS Units: angles=radians, distance=meters, velocity=rad/s
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from xarm.wrapper import XArmAPI

if TYPE_CHECKING:
    from dimos.hardware.manipulators.registry import AdapterRegistry

from dimos.hardware.manipulators.spec import (
    ControlMode,
    JointLimits,
    ManipulatorAdapter,
    ManipulatorInfo,
)

# Unit conversion constants
MM_TO_M = 0.001
M_TO_MM = 1000.0
MAX_CARTESIAN_SPEED_MM = 500.0  # Max cartesian speed in mm/s

# XArm mode codes
_XARM_MODE_POSITION = 0
_XARM_MODE_SERVO_CARTESIAN = 1
_XARM_MODE_JOINT_VELOCITY = 4
_XARM_MODE_CARTESIAN_VELOCITY = 5
_XARM_MODE_JOINT_TORQUE = 6


class XArmAdapter(ManipulatorAdapter):
    """XArm-specific adapter.

    Implements ManipulatorAdapter protocol via duck typing.
    No inheritance required - just matching method signatures.
    """

    def __init__(self, address: str, dof: int = 6, **_: object) -> None:
        if not address:
            raise ValueError("address (IP) is required for XArmAdapter")
        self._ip = address
        self._dof = dof
        self._arm: XArmAPI | None = None
        self._control_mode: ControlMode = ControlMode.POSITION

    # =========================================================================
    # Connection
    # =========================================================================

    def connect(self) -> bool:
        """Connect to XArm via TCP/IP."""
        try:
            self._arm = XArmAPI(self._ip)
            self._arm.connect()

            if not self._arm.connected:
                print(f"ERROR: XArm at {self._ip} not reachable (connected=False)")
                return False

            # Initialize to servo mode for high-frequency control
            self._arm.set_mode(_XARM_MODE_SERVO_CARTESIAN)  # Mode 1 = servo mode
            self._arm.set_state(0)
            self._control_mode = ControlMode.SERVO_POSITION

            return True
        except Exception as e:
            print(f"ERROR: Failed to connect to XArm at {self._ip}: {e}")
            return False

    def disconnect(self) -> None:
        """Disconnect from XArm."""
        if self._arm:
            self._arm.disconnect()
            self._arm = None

    def is_connected(self) -> bool:
        """Check if connected to XArm."""
        return self._arm is not None and self._arm.connected

    # =========================================================================
    # Info
    # =========================================================================

    def get_info(self) -> ManipulatorInfo:
        """Get XArm information."""
        return ManipulatorInfo(
            vendor="UFACTORY",
            model=f"xArm{self._dof}",
            dof=self._dof,
        )

    def get_dof(self) -> int:
        """Get degrees of freedom."""
        return self._dof

    def get_limits(self) -> JointLimits:
        """Get joint limits (default XArm limits)."""
        # XArm typical joint limits (varies by joint, using conservative values)
        limit = 2 * math.pi
        return JointLimits(
            position_lower=[-limit] * self._dof,
            position_upper=[limit] * self._dof,
            velocity_max=[math.pi] * self._dof,  # ~180 deg/s
        )

    # =========================================================================
    # Control Mode
    # =========================================================================

    def set_control_mode(self, mode: ControlMode) -> bool:
        """Set XArm control mode.

        Note: XArm is flexible and often accepts commands without explicit
        mode switching, but some operations require the correct mode.
        """
        if not self._arm:
            return False

        mode_map = {
            ControlMode.POSITION: _XARM_MODE_POSITION,
            ControlMode.SERVO_POSITION: _XARM_MODE_SERVO_CARTESIAN,  # Mode 1 for high-freq
            ControlMode.VELOCITY: _XARM_MODE_JOINT_VELOCITY,
            ControlMode.TORQUE: _XARM_MODE_JOINT_TORQUE,
            ControlMode.CARTESIAN: _XARM_MODE_SERVO_CARTESIAN,
            ControlMode.CARTESIAN_VELOCITY: _XARM_MODE_CARTESIAN_VELOCITY,
        }

        xarm_mode = mode_map.get(mode)
        if xarm_mode is None:
            return False

        code = self._arm.set_mode(xarm_mode)
        if code == 0:
            self._arm.set_state(0)
            self._control_mode = mode
            return True
        return False

    def get_control_mode(self) -> ControlMode:
        """Get current control mode."""
        return self._control_mode

    # =========================================================================
    # State Reading
    # =========================================================================

    def read_joint_positions(self) -> list[float]:
        """Read joint positions (degrees -> radians)."""
        if not self._arm:
            raise RuntimeError("Not connected")

        _, angles = self._arm.get_servo_angle()
        if not angles:
            raise RuntimeError("Failed to read joint positions")
        return [math.radians(a) for a in angles[: self._dof]]

    def read_joint_velocities(self) -> list[float]:
        """Read joint velocities.

        Note: XArm doesn't provide real-time velocity feedback directly.
        Returns zeros. For velocity estimation, use finite differences
        on positions in the driver.
        """
        return [0.0] * self._dof

    def read_joint_efforts(self) -> list[float]:
        """Read joint torques in Nm."""
        if not self._arm:
            return [0.0] * self._dof

        code, torques = self._arm.get_joints_torque()
        if code == 0 and torques:
            return list(torques[: self._dof])
        return [0.0] * self._dof

    def read_state(self) -> dict[str, int]:
        """Read robot state."""
        if not self._arm:
            return {"state": 0, "mode": 0}

        return {
            "state": self._arm.state,
            "mode": self._arm.mode,
        }

    def read_error(self) -> tuple[int, str]:
        """Read error code and message."""
        if not self._arm:
            return 0, ""

        code = self._arm.error_code
        if code == 0:
            return 0, ""
        return code, f"XArm error {code}"

    # =========================================================================
    # Motion Control (Joint Space)
    # =========================================================================

    def write_joint_positions(
        self,
        positions: list[float],
        velocity: float = 1.0,
    ) -> bool:
        """Write joint positions for servo mode (radians -> degrees).

        Uses set_servo_angle_j() for high-frequency servo control.
        Requires mode 1 (servo mode) to be active.

        Args:
            positions: Target positions in radians
            velocity: Speed as fraction of max (0-1) - not used in servo mode
        """
        if not self._arm:
            return False

        # Convert radians to degrees
        angles = [math.degrees(p) for p in positions]

        # Use set_servo_angle_j for high-frequency servo control (100Hz+)
        # This only executes the last instruction, suitable for real-time control
        code: int = self._arm.set_servo_angle_j(angles, speed=100, mvacc=500)
        return code == 0

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        """Write joint velocities (rad/s -> deg/s).

        Note: Requires velocity mode to be active.
        """
        if not self._arm:
            return False

        # Convert rad/s to deg/s
        speeds = [math.degrees(v) for v in velocities]
        code: int = self._arm.vc_set_joint_velocity(speeds)
        return code == 0

    def write_stop(self) -> bool:
        """Emergency stop."""
        if not self._arm:
            return False
        code: int = self._arm.emergency_stop()
        return code == 0

    # =========================================================================
    # Servo Control
    # =========================================================================

    def write_enable(self, enable: bool) -> bool:
        """Enable or disable servos."""
        if not self._arm:
            return False
        code: int = self._arm.motion_enable(enable=enable)
        return code == 0

    def read_enabled(self) -> bool:
        """Check if servos are enabled."""
        if not self._arm:
            return False
        # XArm state 0 = ready/enabled
        state: int = self._arm.state
        return state == 0

    def write_clear_errors(self) -> bool:
        """Clear error state."""
        if not self._arm:
            return False
        code: int = self._arm.clean_error()
        return code == 0

    # =========================================================================
    # Cartesian Control (Optional)
    # =========================================================================

    def read_cartesian_position(self) -> dict[str, float] | None:
        """Read end-effector pose (mm -> meters, degrees -> radians)."""
        if not self._arm:
            return None

        _, pose = self._arm.get_position()
        if pose and len(pose) >= 6:
            return {
                "x": pose[0] * MM_TO_M,
                "y": pose[1] * MM_TO_M,
                "z": pose[2] * MM_TO_M,
                "roll": math.radians(pose[3]),
                "pitch": math.radians(pose[4]),
                "yaw": math.radians(pose[5]),
            }
        return None

    def write_cartesian_position(
        self,
        pose: dict[str, float],
        velocity: float = 1.0,
    ) -> bool:
        """Write end-effector pose (meters -> mm, radians -> degrees)."""
        if not self._arm:
            return False

        code: int = self._arm.set_position(
            x=pose.get("x", 0) * M_TO_MM,
            y=pose.get("y", 0) * M_TO_MM,
            z=pose.get("z", 0) * M_TO_MM,
            roll=math.degrees(pose.get("roll", 0)),
            pitch=math.degrees(pose.get("pitch", 0)),
            yaw=math.degrees(pose.get("yaw", 0)),
            speed=velocity * MAX_CARTESIAN_SPEED_MM,
            wait=False,
        )
        return code == 0

    # =========================================================================
    # Gripper (Optional)
    # =========================================================================

    def read_gripper_position(self) -> float | None:
        """Read gripper position (mm -> meters)."""
        if not self._arm:
            return None

        result = self._arm.get_gripper_position()
        code: int = result[0]
        pos: float | None = result[1]
        if code == 0 and pos is not None:
            return pos * MM_TO_M
        return None

    def write_gripper_position(self, position: float) -> bool:
        """Write gripper position (meters -> mm)."""
        if not self._arm:
            return False

        pos_mm = position * M_TO_MM
        code: int = self._arm.set_gripper_position(pos_mm)
        return code == 0

    # =========================================================================
    # Force/Torque Sensor (Optional)
    # =========================================================================

    def read_force_torque(self) -> list[float] | None:
        """Read F/T sensor data if available."""
        if not self._arm:
            return None

        code, ft = self._arm.get_ft_sensor_data()
        if code == 0 and ft:
            return list(ft)
        return None


def register(registry: AdapterRegistry) -> None:
    """Register this adapter with the registry."""
    registry.register("xarm", XArmAdapter)


__all__ = ["XArmAdapter"]

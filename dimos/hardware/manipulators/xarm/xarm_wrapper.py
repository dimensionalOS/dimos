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

"""XArm SDK wrapper implementation."""

import logging
import math
from typing import Any

from ..base.sdk_interface import BaseManipulatorSDK, ManipulatorInfo


class XArmSDKWrapper(BaseManipulatorSDK):
    """SDK wrapper for XArm manipulators.

    This wrapper translates XArm's native SDK (which uses degrees and mm)
    to our standard interface (radians and meters).
    """

    def __init__(self) -> None:
        """Initialize the XArm SDK wrapper."""
        self.logger = logging.getLogger(self.__class__.__name__)
        self.native_sdk: Any = None
        self.dof = 7  # Default, will be updated on connect
        self._connected = False

    # ============= Connection Management =============

    def connect(self, config: dict[str, Any]) -> bool:
        """Connect to XArm controller.

        Args:
            config: Configuration with 'ip' and optionally 'dof' (5, 6, or 7)

        Returns:
            True if connection successful
        """
        try:
            from xarm import XArmAPI  # type: ignore[import-untyped]

            ip = config.get("ip", "192.168.1.100")
            self.dof = config.get("dof", 7)

            self.logger.info(f"Connecting to XArm at {ip} (DOF: {self.dof})...")

            # Create XArm API instance
            # XArm SDK uses degrees by default, we'll convert to radians
            self.native_sdk = XArmAPI(ip, is_radian=False)

            # Check connection
            if self.native_sdk.connected:
                # Initialize XArm
                self.native_sdk.motion_enable(True)
                self.native_sdk.set_mode(1)  # Servo mode for high-frequency control
                self.native_sdk.set_state(0)  # Ready state

                self._connected = True
                self.logger.info(
                    f"Successfully connected to XArm (version: {self.native_sdk.version})"
                )
                return True
            else:
                self.logger.error("Failed to connect to XArm")
                return False

        except ImportError:
            self.logger.error("XArm SDK not installed. Please install: pip install xArm-Python-SDK")
            return False
        except Exception as e:
            self.logger.error(f"Connection failed: {e}")
            return False

    def disconnect(self) -> None:
        """Disconnect from XArm controller."""
        if self.native_sdk:
            try:
                self.native_sdk.disconnect()
                self._connected = False
                self.logger.info("Disconnected from XArm")
            except:
                pass
            finally:
                self.native_sdk = None

    def is_connected(self) -> bool:
        """Check if connected to XArm.

        Returns:
            True if connected
        """
        return self._connected and self.native_sdk and self.native_sdk.connected

    # ============= Joint State Query =============

    def get_joint_positions(self) -> list[float]:
        """Get current joint positions.

        Returns:
            Joint positions in RADIANS
        """
        code, angles = self.native_sdk.get_servo_angle()
        if code != 0:
            raise RuntimeError(f"XArm error getting positions: {code}")

        # Convert degrees to radians
        positions = [math.radians(angle) for angle in angles[: self.dof]]
        return positions

    def get_joint_velocities(self) -> list[float]:
        """Get current joint velocities.

        Returns:
            Joint velocities in RAD/S
        """
        # XArm doesn't directly provide velocities in older versions
        # Try to get from realtime data if available
        if hasattr(self.native_sdk, "get_joint_speeds"):
            code, speeds = self.native_sdk.get_joint_speeds()
            if code == 0:
                # Convert deg/s to rad/s
                return [math.radians(speed) for speed in speeds[: self.dof]]

        # Return zeros if not available
        return [0.0] * self.dof

    def get_joint_efforts(self) -> list[float]:
        """Get current joint efforts/torques.

        Returns:
            Joint efforts in Nm
        """
        # Try to get joint torques
        if hasattr(self.native_sdk, "get_joint_torques"):
            code, torques = self.native_sdk.get_joint_torques()
            if code == 0:
                return list(torques[: self.dof])

        # Return zeros if not available
        return [0.0] * self.dof

    # ============= Joint Motion Control =============

    def set_joint_positions(
        self,
        positions: list[float],
        _velocity: float = 1.0,
        _acceleration: float = 1.0,
        _wait: bool = False,
    ) -> bool:
        """Move joints to target positions using servo mode.

        Args:
            positions: Target positions in RADIANS
            _velocity: UNUSED in servo mode (kept for interface compatibility)
            _acceleration: UNUSED in servo mode (kept for interface compatibility)
            _wait: UNUSED in servo mode (kept for interface compatibility)

        Returns:
            True if command accepted
        """
        # Convert radians to degrees
        degrees = [math.degrees(pos) for pos in positions]

        # Use set_servo_angle_j for high-frequency servo control (100Hz+)
        # This sends immediate position commands without trajectory planning
        # Requires mode 1 (servo mode) and executes only the last instruction
        code = self.native_sdk.set_servo_angle_j(degrees, speed=100, mvacc=500, wait=False)

        return bool(code == 0)

    def set_joint_velocities(self, velocities: list[float]) -> bool:
        """Set joint velocity targets.

        Args:
            velocities: Target velocities in RAD/S

        Returns:
            True if command accepted
        """
        # Check if velocity control is supported
        if not hasattr(self.native_sdk, "vc_set_joint_velocity"):
            self.logger.warning("Velocity control not supported in this XArm version")
            return False

        # Convert rad/s to deg/s
        deg_velocities = [math.degrees(vel) for vel in velocities]

        # Set to velocity control mode if needed
        if self.native_sdk.mode != 4:
            self.native_sdk.set_mode(4)  # Joint velocity mode

        # Send velocity command
        code = self.native_sdk.vc_set_joint_velocity(deg_velocities)
        return bool(code == 0)

    def set_joint_efforts(self, efforts: list[float]) -> bool:
        """Set joint effort/torque targets.

        Args:
            efforts: Target efforts in Nm

        Returns:
            True if command accepted
        """
        # Check if torque control is supported
        if not hasattr(self.native_sdk, "set_joint_torque"):
            self.logger.warning("Torque control not supported in this XArm version")
            return False

        # Send torque command
        code = self.native_sdk.set_joint_torque(efforts)
        return bool(code == 0)

    def stop_motion(self) -> bool:
        """Stop all ongoing motion.

        Returns:
            True if stop successful
        """
        # XArm emergency stop
        code = self.native_sdk.emergency_stop()

        # Re-enable after stop
        if code == 0:
            self.native_sdk.set_state(0)  # Clear stop state
            self.native_sdk.motion_enable(True)

        return bool(code == 0)

    # ============= Servo Control =============

    def enable_servos(self) -> bool:
        """Enable motor control.

        Returns:
            True if servos enabled
        """
        code1 = self.native_sdk.motion_enable(True)
        code2 = self.native_sdk.set_state(0)  # Ready state
        code3 = self.native_sdk.set_mode(1)  # Servo mode
        return bool(code1 == 0 and code2 == 0 and code3 == 0)

    def disable_servos(self) -> bool:
        """Disable motor control.

        Returns:
            True if servos disabled
        """
        code = self.native_sdk.motion_enable(False)
        return bool(code == 0)

    def are_servos_enabled(self) -> bool:
        """Check if servos are enabled.

        Returns:
            True if enabled
        """
        # Check motor state
        return bool(self.native_sdk.mode == 1 and self.native_sdk.mode != 4)

    # ============= System State =============

    def get_robot_state(self) -> dict[str, Any]:
        """Get current robot state.

        Returns:
            State dictionary
        """
        return {
            "state": self.native_sdk.state,  # 0=ready, 1=pause, 2=stop, 3=running, 4=error
            "mode": self.native_sdk.mode,  # 0=position, 1=servo, 4=joint_vel, 5=cart_vel
            "error_code": self.native_sdk.error_code,
            "warn_code": self.native_sdk.warn_code,
            "is_moving": self.native_sdk.state == 3,
            "cmd_num": self.native_sdk.cmd_num,
        }

    def get_error_code(self) -> int:
        """Get current error code.

        Returns:
            Error code (0 = no error)
        """
        return int(self.native_sdk.error_code)

    def get_error_message(self) -> str:
        """Get human-readable error message.

        Returns:
            Error message string
        """
        if self.native_sdk.error_code == 0:
            return ""

        # XArm error codes (partial list)
        error_map = {
            1: "Emergency stop button pressed",
            2: "Joint limit exceeded",
            3: "Command reply timeout",
            4: "Power supply error",
            5: "Motor overheated",
            6: "Motor driver error",
            7: "Other error",
            10: "Servo error",
            11: "Joint collision",
            12: "Tool IO error",
            13: "Tool communication error",
            14: "Kinematic error",
            15: "Self collision",
            16: "Joint overheated",
            17: "Planning error",
            19: "Force control error",
            20: "Joint current overlimit",
            21: "TCP command overlimit",
            22: "Overspeed",
        }

        return error_map.get(
            self.native_sdk.error_code, f"Unknown error {self.native_sdk.error_code}"
        )

    def clear_errors(self) -> bool:
        """Clear error states.

        Returns:
            True if errors cleared
        """
        code = self.native_sdk.clean_error()
        if code == 0:
            # Reset to ready state
            self.native_sdk.set_state(0)
        return bool(code == 0)

    def emergency_stop(self) -> bool:
        """Execute emergency stop.

        Returns:
            True if e-stop executed
        """
        code = self.native_sdk.emergency_stop()
        return bool(code == 0)

    # ============= Information =============

    def get_info(self) -> ManipulatorInfo:
        """Get manipulator information.

        Returns:
            ManipulatorInfo object
        """
        return ManipulatorInfo(
            vendor="UFACTORY",
            model=f"xArm{self.dof}",
            dof=self.dof,
            firmware_version=self.native_sdk.version if self.native_sdk else None,
            serial_number=self.native_sdk.get_servo_version()[1][0] if self.native_sdk else None,
        )

    def get_joint_limits(self) -> tuple[list[float], list[float]]:
        """Get joint position limits.

        Returns:
            Tuple of (lower_limits, upper_limits) in RADIANS
        """
        # XArm joint limits in degrees (approximate, varies by model)
        if self.dof == 7:
            lower_deg = [-360, -118, -360, -233, -360, -97, -360]
            upper_deg = [360, 118, 360, 11, 360, 180, 360]
        elif self.dof == 6:
            lower_deg = [-360, -118, -225, -11, -360, -97]
            upper_deg = [360, 118, 11, 225, 360, 180]
        else:  # 5 DOF
            lower_deg = [-360, -118, -225, -97, -360]
            upper_deg = [360, 118, 11, 180, 360]

        # Convert to radians
        lower_rad = [math.radians(d) for d in lower_deg[: self.dof]]
        upper_rad = [math.radians(d) for d in upper_deg[: self.dof]]

        return (lower_rad, upper_rad)

    def get_velocity_limits(self) -> list[float]:
        """Get joint velocity limits.

        Returns:
            Maximum velocities in RAD/S
        """
        # XArm max velocities in deg/s (default)
        max_vel_deg = 180.0

        # Convert to rad/s
        max_vel_rad = math.radians(max_vel_deg)
        return [max_vel_rad] * self.dof

    def get_acceleration_limits(self) -> list[float]:
        """Get joint acceleration limits.

        Returns:
            Maximum accelerations in RAD/S²
        """
        # XArm max acceleration in deg/s² (default)
        max_acc_deg = 1145.0

        # Convert to rad/s²
        max_acc_rad = math.radians(max_acc_deg)
        return [max_acc_rad] * self.dof

    # ============= Optional Methods =============

    def get_cartesian_position(self) -> dict[str, float] | None:
        """Get current end-effector pose.

        Returns:
            Pose dict or None if not supported
        """
        code, pose = self.native_sdk.get_position()
        if code != 0:
            return None

        # XArm returns [x, y, z (mm), roll, pitch, yaw (degrees)]
        return {
            "x": pose[0] / 1000.0,  # mm to meters
            "y": pose[1] / 1000.0,
            "z": pose[2] / 1000.0,
            "roll": math.radians(pose[3]),
            "pitch": math.radians(pose[4]),
            "yaw": math.radians(pose[5]),
        }

    def set_cartesian_position(
        self,
        pose: dict[str, float],
        velocity: float = 1.0,
        acceleration: float = 1.0,
        wait: bool = False,
    ) -> bool:
        """Move end-effector to target pose.

        Args:
            pose: Target pose dict
            velocity: Max velocity fraction (0-1)
            acceleration: Max acceleration fraction (0-1)
            wait: Block until complete

        Returns:
            True if command accepted
        """
        # Convert to XArm format
        xarm_pose = [
            pose["x"] * 1000.0,  # meters to mm
            pose["y"] * 1000.0,
            pose["z"] * 1000.0,
            math.degrees(pose["roll"]),
            math.degrees(pose["pitch"]),
            math.degrees(pose["yaw"]),
        ]

        # XArm max Cartesian speed (default 500 mm/s)
        max_speed = 500.0
        speed = max_speed * velocity

        # XArm max Cartesian acceleration (default 2000 mm/s²)
        max_acc = 2000.0
        acc = max_acc * acceleration

        code = self.native_sdk.set_position(xarm_pose, radius=-1, speed=speed, mvacc=acc, wait=wait)

        return bool(code == 0)

    def get_force_torque(self) -> list[float] | None:
        """Get F/T sensor reading.

        Returns:
            [fx, fy, fz, tx, ty, tz] or None
        """
        if hasattr(self.native_sdk, "get_ft_sensor_data"):
            code, ft_data = self.native_sdk.get_ft_sensor_data()
            if code == 0:
                return list(ft_data)
        return None

    def zero_force_torque(self) -> bool:
        """Zero the F/T sensor.

        Returns:
            True if successful
        """
        if hasattr(self.native_sdk, "set_ft_sensor_zero"):
            code = self.native_sdk.set_ft_sensor_zero()
            return bool(code == 0)
        return False

    def get_gripper_position(self) -> float | None:
        """Get gripper position.

        Returns:
            Position in meters or None
        """
        if hasattr(self.native_sdk, "get_gripper_position"):
            code, pos = self.native_sdk.get_gripper_position()
            if code == 0:
                # Convert mm to meters
                return float(pos / 1000.0)
        return None

    def set_gripper_position(self, position: float, force: float = 1.0) -> bool:
        """Set gripper position.

        Args:
            position: Target position in meters
            force: Force fraction (0-1)

        Returns:
            True if successful
        """
        if hasattr(self.native_sdk, "set_gripper_position"):
            # Convert meters to mm
            pos_mm = position * 1000.0
            code = self.native_sdk.set_gripper_position(pos_mm, wait=False)
            return bool(code == 0)
        return False

    def set_control_mode(self, mode: str) -> bool:
        """Set control mode.

        Args:
            mode: 'position', 'velocity', 'torque', or 'impedance'

        Returns:
            True if successful
        """
        mode_map = {
            "position": 0,
            "velocity": 4,  # Joint velocity mode
            "servo": 1,  # Servo mode (for torque control)
            "impedance": 0,  # Not directly supported, use position
        }

        if mode not in mode_map:
            return False

        code = self.native_sdk.set_mode(mode_map[mode])
        return bool(code == 0)

    def get_control_mode(self) -> str | None:
        """Get current control mode.

        Returns:
            Mode string or None
        """
        mode_map = {0: "position", 1: "servo", 4: "velocity", 5: "cartesian_velocity"}

        return mode_map.get(self.native_sdk.mode, "unknown")

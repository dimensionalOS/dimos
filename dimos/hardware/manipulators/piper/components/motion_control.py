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

"""
Motion Control Component for PiperDriver.

Provides RPC methods for motion control operations including:
- Joint position control
- Joint velocity control
- End-effector pose control
- Emergency stop
- Circular motion
"""

import math
import time
from typing import Any

from dimos.core import rpc
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class MotionControlComponent:
    """
    Component providing motion control RPC methods for PiperDriver.

    This component assumes the parent class has:
    - self.piper: C_PiperInterface_V2 instance
    - self.config: PiperDriverConfig instance
    - RAD_TO_PIPER: conversion constant (radians → 0.001 degrees)
    - PIPER_TO_RAD: conversion constant (0.001 degrees → radians)
    """

    # Type hints for attributes expected from parent class
    piper: Any
    config: Any
    RAD_TO_PIPER: float
    PIPER_TO_RAD: float
    _joint_cmd_lock: Any
    _joint_cmd_: Any
    _vel_cmd_: Any
    _last_cmd_time: float

    @rpc
    def set_joint_angles(self, angles: list[float], gripper_state: int = 0x00) -> tuple[bool, str]:
        """
        Set joint angles (RPC method).

        Args:
            angles: List of joint angles in radians
            gripper_state: Gripper state (0x00 = no change, 0x01 = open, 0x02 = close)

        Returns:
            Tuple of (success, message)
        """
        try:
            if len(angles) != 6:
                return (False, f"Expected 6 joint angles, got {len(angles)}")

            # Convert radians to Piper units (0.001 degrees)
            piper_joints = [round(rad * self.RAD_TO_PIPER) for rad in angles]

            # Send joint control command
            result = self.piper.JointCtrl(
                piper_joints[0],
                piper_joints[1],
                piper_joints[2],
                piper_joints[3],
                piper_joints[4],
                piper_joints[5],
                gripper_state,
            )

            if result:
                return (True, "Joint angles set successfully")
            else:
                return (False, "Failed to set joint angles")

        except Exception as e:
            logger.error(f"set_joint_angles failed: {e}")
            return (False, str(e))

    @rpc
    def set_joint_command(self, positions: list[float]) -> tuple[bool, str]:
        """
        Manually set the joint command (for testing).
        This updates the shared joint_cmd that the control loop reads.

        Args:
            positions: List of joint positions in radians

        Returns:
            Tuple of (success, message)
        """
        try:
            if len(positions) != 6:
                return (False, f"Expected 6 joint positions, got {len(positions)}")

            with self._joint_cmd_lock:
                self._joint_cmd_ = list(positions)

            logger.info(f"✓ Joint command set: {[f'{math.degrees(p):.2f}°' for p in positions]}")
            return (True, "Joint command updated")
        except Exception as e:
            return (False, str(e))

    @rpc
    def set_end_pose(
        self, x: float, y: float, z: float, rx: float, ry: float, rz: float
    ) -> tuple[bool, str]:
        """
        Set end-effector pose.

        Args:
            x: X position in millimeters
            y: Y position in millimeters
            z: Z position in millimeters
            rx: Roll in radians
            ry: Pitch in radians
            rz: Yaw in radians

        Returns:
            Tuple of (success, message)
        """
        try:
            # Convert to Piper units
            # Position: mm → 0.001 mm
            x_piper = round(x * 1000)
            y_piper = round(y * 1000)
            z_piper = round(z * 1000)

            # Rotation: radians → 0.001 degrees
            rx_piper = round(math.degrees(rx) * 1000)
            ry_piper = round(math.degrees(ry) * 1000)
            rz_piper = round(math.degrees(rz) * 1000)

            # Send end pose control command
            result = self.piper.EndPoseCtrl(x_piper, y_piper, z_piper, rx_piper, ry_piper, rz_piper)

            if result:
                return (True, "End pose set successfully")
            else:
                return (False, "Failed to set end pose")

        except Exception as e:
            logger.error(f"set_end_pose failed: {e}")
            return (False, str(e))

    @rpc
    def emergency_stop(self) -> tuple[bool, str]:
        """Emergency stop the arm."""
        try:
            result = self.piper.EmergencyStop()

            if result:
                logger.warning("Emergency stop activated")
                return (True, "Emergency stop activated")
            else:
                return (False, "Failed to activate emergency stop")

        except Exception as e:
            logger.error(f"emergency_stop failed: {e}")
            return (False, str(e))

    @rpc
    def move_c_axis_update(self, instruction_num: int = 0x00) -> tuple[bool, str]:
        """
        Update circular motion axis.

        Args:
            instruction_num: Instruction number (0x00, 0x01, 0x02, 0x03)

        Returns:
            Tuple of (success, message)
        """
        try:
            if instruction_num not in [0x00, 0x01, 0x02, 0x03]:
                return (False, f"Invalid instruction_num: {instruction_num}")

            result = self.piper.MoveCAxisUpdateCtrl(instruction_num)

            if result:
                return (True, f"Move C axis updated with instruction {instruction_num}")
            else:
                return (False, "Failed to update Move C axis")

        except Exception as e:
            logger.error(f"move_c_axis_update failed: {e}")
            return (False, str(e))

    @rpc
    def set_joint_mit_ctrl(
        self,
        motor_num: int,
        pos_target: float,
        vel_target: float,
        torq_target: float,
        kp: int,
        kd: int,
    ) -> tuple[bool, str]:
        """
        Set joint MIT (Model-based Inverse Torque) control.

        Args:
            motor_num: Motor number (1-6)
            pos_target: Target position in radians
            vel_target: Target velocity in rad/s
            torq_target: Target torque in Nm
            kp: Proportional gain (0-100)
            kd: Derivative gain (0-100)

        Returns:
            Tuple of (success, message)
        """
        try:
            if motor_num not in range(1, 7):
                return (False, f"Invalid motor_num: {motor_num}. Must be 1-6")

            # Convert to Piper units
            pos_piper = round(pos_target * self.RAD_TO_PIPER)
            vel_piper = round(vel_target * self.RAD_TO_PIPER)
            torq_piper = round(torq_target * 1000)  # Torque in millinewton-meters

            result = self.piper.JointMitCtrl(motor_num, pos_piper, vel_piper, torq_piper, kp, kd)

            if result:
                return (True, f"Joint {motor_num} MIT control set successfully")
            else:
                return (False, f"Failed to set MIT control for joint {motor_num}")

        except Exception as e:
            logger.error(f"set_joint_mit_ctrl failed: {e}")
            return (False, str(e))

    @rpc
    def set_joint_velocities(self, velocities: list[float]) -> tuple[bool, str]:
        """
        Set joint velocities (RPC method).

        Requires velocity control mode to be enabled.

        The control loop integrates velocities to positions:
        - position_target += velocity * dt
        - Integrated positions are sent to JointCtrl

        This provides smooth velocity control while using the proven position API.

        Args:
            velocities: List of 6 joint velocities in rad/s

        Returns:
            Tuple of (success, message)
        """
        try:
            if len(velocities) != 6:
                return (False, f"Expected 6 velocities, got {len(velocities)}")

            if not self.config.velocity_control:
                return (
                    False,
                    "Velocity control mode not enabled. Call enable_velocity_control_mode() first.",
                )

            with self._joint_cmd_lock:
                self._vel_cmd_ = list(velocities)
                self._last_cmd_time = time.time()

            logger.info(f"✓ Velocity command set: {[f'{v:.3f} rad/s' for v in velocities]}")
            return (True, "Velocity command updated")

        except Exception as e:
            logger.error(f"set_joint_velocities failed: {e}")
            return (False, str(e))

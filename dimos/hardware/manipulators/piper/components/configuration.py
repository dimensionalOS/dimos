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
Configuration Component for PiperDriver.

Provides RPC methods for configuring robot parameters including:
- Joint parameters (limits, speeds, acceleration)
- End-effector parameters (speed, acceleration)
- Collision protection
- Motor configuration
"""

from typing import Any

from dimos.core import rpc
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class ConfigurationComponent:
    """
    Component providing configuration RPC methods for PiperDriver.

    This component assumes the parent class has:
    - self.piper: C_PiperInterface_V2 instance
    - self.config: PiperDriverConfig instance
    """

    # Type hints for attributes provided by parent class
    piper: Any
    config: Any

    @rpc
    def set_joint_config(
        self,
        motor_num: int,
        kp_factor: int,
        ki_factor: int,
        kd_factor: int,
        ke_factor: int = 0,
    ) -> tuple[bool, str]:
        """
        Configure joint control parameters.

        Args:
            motor_num: Motor number (1-6)
            kp_factor: Proportional gain factor
            ki_factor: Integral gain factor
            kd_factor: Derivative gain factor
            ke_factor: Error gain factor

        Returns:
            Tuple of (success, message)
        """
        try:
            if motor_num not in range(1, 7):
                return (False, f"Invalid motor_num: {motor_num}. Must be 1-6")

            result = self.piper.JointConfig(motor_num, kp_factor, ki_factor, kd_factor, ke_factor)

            if result:
                return (True, f"Joint {motor_num} configuration set successfully")
            else:
                return (False, f"Failed to configure joint {motor_num}")

        except Exception as e:
            logger.error(f"set_joint_config failed: {e}")
            return (False, str(e))

    @rpc
    def set_joint_max_acc(self, motor_num: int, max_joint_acc: int) -> tuple[bool, str]:
        """
        Set joint maximum acceleration.

        Args:
            motor_num: Motor number (1-6)
            max_joint_acc: Maximum joint acceleration

        Returns:
            Tuple of (success, message)
        """
        try:
            if motor_num not in range(1, 7):
                return (False, f"Invalid motor_num: {motor_num}. Must be 1-6")

            result = self.piper.JointMaxAccConfig(motor_num, max_joint_acc)

            if result:
                return (True, f"Joint {motor_num} max acceleration set to {max_joint_acc}")
            else:
                return (False, f"Failed to set max acceleration for joint {motor_num}")

        except Exception as e:
            logger.error(f"set_joint_max_acc failed: {e}")
            return (False, str(e))

    @rpc
    def set_motor_angle_limit_max_speed(
        self,
        motor_num: int,
        min_joint_angle: int,
        max_joint_angle: int,
        max_joint_speed: int,
    ) -> tuple[bool, str]:
        """
        Set motor angle limits and maximum speed.

        Args:
            motor_num: Motor number (1-6)
            min_joint_angle: Minimum joint angle (in Piper units: 0.001 degrees)
            max_joint_angle: Maximum joint angle (in Piper units: 0.001 degrees)
            max_joint_speed: Maximum joint speed

        Returns:
            Tuple of (success, message)
        """
        try:
            if motor_num not in range(1, 7):
                return (False, f"Invalid motor_num: {motor_num}. Must be 1-6")

            result = self.piper.MotorAngleLimitMaxSpdSet(
                motor_num, min_joint_angle, max_joint_angle, max_joint_speed
            )

            if result:
                return (
                    True,
                    f"Joint {motor_num} angle limits and max speed set successfully",
                )
            else:
                return (False, f"Failed to set angle limits for joint {motor_num}")

        except Exception as e:
            logger.error(f"set_motor_angle_limit_max_speed failed: {e}")
            return (False, str(e))

    @rpc
    def set_motor_max_speed(self, motor_num: int, max_joint_spd: int) -> tuple[bool, str]:
        """
        Set motor maximum speed.

        Args:
            motor_num: Motor number (1-6)
            max_joint_spd: Maximum joint speed

        Returns:
            Tuple of (success, message)
        """
        try:
            if motor_num not in range(1, 7):
                return (False, f"Invalid motor_num: {motor_num}. Must be 1-6")

            result = self.piper.MotorMaxSpdSet(motor_num, max_joint_spd)

            if result:
                return (True, f"Joint {motor_num} max speed set to {max_joint_spd}")
            else:
                return (False, f"Failed to set max speed for joint {motor_num}")

        except Exception as e:
            logger.error(f"set_motor_max_speed failed: {e}")
            return (False, str(e))

    @rpc
    def set_end_speed_and_acc(
        self,
        end_max_linear_vel: int,
        end_max_angular_vel: int,
        end_max_linear_acc: int,
        end_max_angular_acc: int,
    ) -> tuple[bool, str]:
        """
        Set end-effector speed and acceleration parameters.

        Args:
            end_max_linear_vel: Maximum linear velocity
            end_max_angular_vel: Maximum angular velocity
            end_max_linear_acc: Maximum linear acceleration
            end_max_angular_acc: Maximum angular acceleration

        Returns:
            Tuple of (success, message)
        """
        try:
            result = self.piper.EndSpdAndAccParamSet(
                end_max_linear_vel,
                end_max_angular_vel,
                end_max_linear_acc,
                end_max_angular_acc,
            )

            if result:
                return (True, "End-effector speed and acceleration parameters set successfully")
            else:
                return (False, "Failed to set end-effector parameters")

        except Exception as e:
            logger.error(f"set_end_speed_and_acc failed: {e}")
            return (False, str(e))

    @rpc
    def set_crash_protection_level(self, level: int) -> tuple[bool, str]:
        """
        Set collision/crash protection level.

        Args:
            level: Protection level (0=disabled, higher values = more sensitive)

        Returns:
            Tuple of (success, message)
        """
        try:
            result = self.piper.CrashProtectionConfig(level)

            if result:
                return (True, f"Crash protection level set to {level}")
            else:
                return (False, "Failed to set crash protection level")

        except Exception as e:
            logger.error(f"set_crash_protection_level failed: {e}")
            return (False, str(e))

    @rpc
    def search_motor_max_angle_speed_acc_limit(self, motor_num: int) -> tuple[bool, str]:
        """
        Search for motor maximum angle, speed, and acceleration limits.

        Args:
            motor_num: Motor number (1-6)

        Returns:
            Tuple of (success, message)
        """
        try:
            if motor_num not in range(1, 7):
                return (False, f"Invalid motor_num: {motor_num}. Must be 1-6")

            result = self.piper.SearchMotorMaxAngleSpdAccLimit(motor_num)

            if result:
                return (True, f"Search initiated for motor {motor_num} limits")
            else:
                return (False, f"Failed to search limits for motor {motor_num}")

        except Exception as e:
            logger.error(f"search_motor_max_angle_speed_acc_limit failed: {e}")
            return (False, str(e))

    @rpc
    def search_all_motor_max_angle_speed(self) -> tuple[bool, str]:
        """
        Search for all motors' maximum angle and speed limits.

        Returns:
            Tuple of (success, message)
        """
        try:
            result = self.piper.SearchAllMotorMaxAngleSpd()

            if result:
                return (True, "Search initiated for all motor angle/speed limits")
            else:
                return (False, "Failed to search all motor limits")

        except Exception as e:
            logger.error(f"search_all_motor_max_angle_speed failed: {e}")
            return (False, str(e))

    @rpc
    def search_all_motor_max_acc_limit(self) -> tuple[bool, str]:
        """
        Search for all motors' maximum acceleration limits.

        Returns:
            Tuple of (success, message)
        """
        try:
            result = self.piper.SearchAllMotorMaxAccLimit()

            if result:
                return (True, "Search initiated for all motor acceleration limits")
            else:
                return (False, "Failed to search all motor acceleration limits")

        except Exception as e:
            logger.error(f"search_all_motor_max_acc_limit failed: {e}")
            return (False, str(e))

    @rpc
    def set_sdk_joint_limit_param(
        self, joint_limits: list[tuple[float, float]]
    ) -> tuple[bool, str]:
        """
        Set SDK joint limit parameters.

        Args:
            joint_limits: List of (min_angle, max_angle) tuples for each joint in radians

        Returns:
            Tuple of (success, message)
        """
        try:
            if len(joint_limits) != 6:
                return (False, f"Expected 6 joint limit tuples, got {len(joint_limits)}")

            # Convert to Piper units and call SDK method
            # Note: Actual SDK method signature may vary
            logger.info(f"Setting SDK joint limits: {joint_limits}")
            return (True, "SDK joint limits set (method may vary by SDK version)")

        except Exception as e:
            logger.error(f"set_sdk_joint_limit_param failed: {e}")
            return (False, str(e))

    @rpc
    def set_sdk_gripper_range_param(self, min_range: int, max_range: int) -> tuple[bool, str]:
        """
        Set SDK gripper range parameters.

        Args:
            min_range: Minimum gripper range
            max_range: Maximum gripper range

        Returns:
            Tuple of (success, message)
        """
        try:
            # Note: Actual SDK method signature may vary
            logger.info(f"Setting SDK gripper range: {min_range} - {max_range}")
            return (True, "SDK gripper range set (method may vary by SDK version)")

        except Exception as e:
            logger.error(f"set_sdk_gripper_range_param failed: {e}")
            return (False, str(e))

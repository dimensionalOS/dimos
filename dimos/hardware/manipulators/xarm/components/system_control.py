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
System Control Component for XArmDriver.

Provides RPC methods for system-level control operations including:
- Mode control (servo, velocity)
- State management
- Error handling
- Emergency stop
"""

from typing import Tuple
from dimos.core import rpc
from dimos.utils.logging_config import setup_logger

logger = setup_logger(__file__)


class SystemControlComponent:
    """
    Component providing system control RPC methods for XArmDriver.

    This component assumes the parent class has:
    - self.arm: XArmAPI instance
    - self.config: XArmDriverConfig instance
    """

    @rpc
    def enable_servo_mode(self) -> Tuple[int, str]:
        """
        Enable servo mode (mode 1).
        Required for set_servo_angle_j to work.

        Returns:
            Tuple of (code, message)
        """
        try:
            code = self.arm.set_mode(1)
            if code == 0:
                logger.info("Servo mode enabled")
                return (code, "Servo mode enabled")
            else:
                logger.warning(f"Failed to enable servo mode: code={code}")
                return (code, f"Error code: {code}")
        except Exception as e:
            logger.error(f"enable_servo_mode failed: {e}")
            return (-1, str(e))

    @rpc
    def disable_servo_mode(self) -> Tuple[int, str]:
        """
        Disable servo mode (set to position mode).

        Returns:
            Tuple of (code, message)
        """
        try:
            code = self.arm.set_mode(0)
            if code == 0:
                logger.info("Servo mode disabled (position mode)")
                return (code, "Position mode enabled")
            else:
                logger.warning(f"Failed to disable servo mode: code={code}")
                return (code, f"Error code: {code}")
        except Exception as e:
            logger.error(f"disable_servo_mode failed: {e}")
            return (-1, str(e))

    @rpc
    def enable_velocity_control_mode(self) -> Tuple[int, str]:
        """
        Enable velocity control mode (mode 4).
        Required for vc_set_joint_velocity to work.

        Returns:
            Tuple of (code, message)
        """
        try:
            # Step 1: Set mode to 4 (velocity control)
            code = self.arm.set_mode(4)
            if code != 0:
                logger.warning(f"Failed to set mode to 4: code={code}")
                return (code, f"Failed to set mode: code={code}")

            # Step 2: Set state to 0 (ready/sport mode) - this activates the mode!
            code = self.arm.set_state(0)
            if code == 0:
                # Switch driver to velocity control
                self.config.velocity_control = True
                logger.info("Velocity control mode enabled (mode=4, state=0)")
                return (code, "Velocity control mode enabled")
            else:
                logger.warning(f"Failed to set state to 0: code={code}")
                return (code, f"Failed to set state: code={code}")
        except Exception as e:
            logger.error(f"enable_velocity_control_mode failed: {e}")
            return (-1, str(e))

    @rpc
    def disable_velocity_control_mode(self) -> Tuple[int, str]:
        """
        Disable velocity control mode and return to position control (mode 1).

        Returns:
            Tuple of (code, message)
        """
        try:
            # Step 1: Set state to 0 (sport mode) to allow mode change
            code = self.arm.set_state(0)
            if code != 0:
                logger.warning(f"Failed to set state to 0: code={code}")
                # Continue anyway, try to set mode

            # Step 2: Set mode to 1 (servo/position control)
            code = self.arm.set_mode(1)
            if code == 0:
                # Switch driver to position control
                self.config.velocity_control = False
                logger.info("Position control mode enabled (state=0, mode=1)")
                return (code, "Position control mode enabled")
            else:
                logger.warning(f"Failed to disable velocity control mode: code={code}")
                return (code, f"Error code: {code}")
        except Exception as e:
            logger.error(f"disable_velocity_control_mode failed: {e}")
            return (-1, str(e))

    @rpc
    def motion_enable(self, enable: bool = True) -> Tuple[int, str]:
        """Enable or disable arm motion."""
        try:
            code = self.arm.motion_enable(enable=enable)
            msg = f"Motion {'enabled' if enable else 'disabled'}"
            return (code, msg if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def set_state(self, state: int) -> Tuple[int, str]:
        """
        Set robot state.

        Args:
            state: 0=ready, 3=pause, 4=stop
        """
        try:
            code = self.arm.set_state(state=state)
            return (code, "Success" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def clean_error(self) -> Tuple[int, str]:
        """Clear error codes."""
        try:
            code = self.arm.clean_error()
            return (code, "Errors cleared" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def clean_warn(self) -> Tuple[int, str]:
        """Clear warning codes."""
        try:
            code = self.arm.clean_warn()
            return (code, "Warnings cleared" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def emergency_stop(self) -> Tuple[int, str]:
        """Emergency stop the arm."""
        try:
            code = self.arm.emergency_stop()
            return (code, "Emergency stop" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

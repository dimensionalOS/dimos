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

from typing import TYPE_CHECKING, Any, Protocol

from dimos.core import rpc
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from xarm.wrapper import XArmAPI

    class XArmConfig(Protocol):
        """Protocol for XArm configuration."""

        is_radian: bool
        velocity_control: bool


logger = setup_logger()


class SystemControlComponent:
    """
    Component providing system control RPC methods for XArmDriver.

    This component assumes the parent class has:
    - self.arm: XArmAPI instance
    - self.config: XArmDriverConfig instance
    """

    # Type hints for attributes expected from parent class
    arm: "XArmAPI"
    config: Any  # Should be XArmConfig but accessed as dict

    @rpc
    def enable_servo_mode(self) -> tuple[int, str]:
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
    def disable_servo_mode(self) -> tuple[int, str]:
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
    def enable_velocity_control_mode(self) -> tuple[int, str]:
        """
        Enable velocity control mode (mode 4).
        Required for vc_set_joint_velocity to work.

        Returns:
            Tuple of (code, message)
        """
        try:
            # IMPORTANT: Set config flag BEFORE changing robot mode
            # This prevents control loop from sending wrong command type during transition
            self.config.velocity_control = True

            # Step 1: Set mode to 4 (velocity control)
            code = self.arm.set_mode(4)
            if code != 0:
                logger.warning(f"Failed to set mode to 4: code={code}")
                self.config.velocity_control = False  # Revert on failure
                return (code, f"Failed to set mode: code={code}")

            # Step 2: Set state to 0 (ready/sport mode) - this activates the mode!
            code = self.arm.set_state(0)
            if code == 0:
                logger.info("Velocity control mode enabled (mode=4, state=0)")
                return (code, "Velocity control mode enabled")
            else:
                logger.warning(f"Failed to set state to 0: code={code}")
                self.config.velocity_control = False  # Revert on failure
                return (code, f"Failed to set state: code={code}")
        except Exception as e:
            logger.error(f"enable_velocity_control_mode failed: {e}")
            self.config.velocity_control = False  # Revert on exception
            return (-1, str(e))

    @rpc
    def disable_velocity_control_mode(self) -> tuple[int, str]:
        """
        Disable velocity control mode and return to position control (mode 1).

        Returns:
            Tuple of (code, message)
        """
        try:
            # IMPORTANT: Set config flag BEFORE changing robot mode
            # This prevents control loop from sending velocity commands after mode change
            self.config.velocity_control = False

            # Step 1: Clear any errors that may have occurred
            self.arm.clean_error()
            self.arm.clean_warn()

            # Step 2: Set mode to 1 (servo/position control)
            code = self.arm.set_mode(1)
            if code != 0:
                logger.warning(f"Failed to set mode to 1: code={code}")
                self.config.velocity_control = True  # Revert on failure
                return (code, f"Failed to set mode: code={code}")

            # Step 3: Set state to 0 (ready) - CRITICAL for accepting new commands
            code = self.arm.set_state(0)
            if code == 0:
                logger.info("Position control mode enabled (state=0, mode=1)")
                return (code, "Position control mode enabled")
            else:
                logger.warning(f"Failed to set state to 0: code={code}")
                self.config.velocity_control = True  # Revert on failure
                return (code, f"Failed to set state: code={code}")
        except Exception as e:
            logger.error(f"disable_velocity_control_mode failed: {e}")
            self.config.velocity_control = True  # Revert on exception
            return (-1, str(e))

    @rpc
    def motion_enable(self, enable: bool = True) -> tuple[int, str]:
        """Enable or disable arm motion."""
        try:
            code = self.arm.motion_enable(enable=enable)
            msg = f"Motion {'enabled' if enable else 'disabled'}"
            return (code, msg if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def set_state(self, state: int) -> tuple[int, str]:
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
    def clean_error(self) -> tuple[int, str]:
        """Clear error codes."""
        try:
            code = self.arm.clean_error()
            return (code, "Errors cleared" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def clean_warn(self) -> tuple[int, str]:
        """Clear warning codes."""
        try:
            code = self.arm.clean_warn()
            return (code, "Warnings cleared" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def emergency_stop(self) -> tuple[int, str]:
        """Emergency stop the arm."""
        try:
            code = self.arm.emergency_stop()
            return (code, "Emergency stop" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    # =========================================================================
    # Configuration & Persistence
    # =========================================================================

    @rpc
    def clean_conf(self) -> tuple[int, str]:
        """Clean configuration."""
        try:
            code = self.arm.clean_conf()
            return (code, "Configuration cleaned" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def save_conf(self) -> tuple[int, str]:
        """Save current configuration to robot."""
        try:
            code = self.arm.save_conf()
            return (code, "Configuration saved" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def reload_dynamics(self) -> tuple[int, str]:
        """Reload dynamics parameters."""
        try:
            code = self.arm.reload_dynamics()
            return (code, "Dynamics reloaded" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    # =========================================================================
    # Mode & State Control
    # =========================================================================

    @rpc
    def set_mode(self, mode: int) -> tuple[int, str]:
        """
        Set control mode.

        Args:
            mode: 0=position, 1=servo, 4=velocity, etc.
        """
        try:
            code = self.arm.set_mode(mode)
            return (code, f"Mode set to {mode}" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    # =========================================================================
    # Collision & Safety
    # =========================================================================

    @rpc
    def set_collision_sensitivity(self, sensitivity: int) -> tuple[int, str]:
        """Set collision sensitivity (0-5, 0=least sensitive)."""
        try:
            code = self.arm.set_collision_sensitivity(sensitivity)
            return (
                code,
                f"Collision sensitivity set to {sensitivity}"
                if code == 0
                else f"Error code: {code}",
            )
        except Exception as e:
            return (-1, str(e))

    @rpc
    def set_teach_sensitivity(self, sensitivity: int) -> tuple[int, str]:
        """Set teach sensitivity (1-5)."""
        try:
            code = self.arm.set_teach_sensitivity(sensitivity)
            return (
                code,
                f"Teach sensitivity set to {sensitivity}" if code == 0 else f"Error code: {code}",
            )
        except Exception as e:
            return (-1, str(e))

    @rpc
    def set_collision_rebound(self, enable: int) -> tuple[int, str]:
        """Enable/disable collision rebound (0=disable, 1=enable)."""
        try:
            code = self.arm.set_collision_rebound(enable)
            return (
                code,
                f"Collision rebound {'enabled' if enable else 'disabled'}"
                if code == 0
                else f"Error code: {code}",
            )
        except Exception as e:
            return (-1, str(e))

    @rpc
    def set_self_collision_detection(self, enable: int) -> tuple[int, str]:
        """Enable/disable self collision detection."""
        try:
            code = self.arm.set_self_collision_detection(enable)
            return (
                code,
                f"Self collision detection {'enabled' if enable else 'disabled'}"
                if code == 0
                else f"Error code: {code}",
            )
        except Exception as e:
            return (-1, str(e))

    # =========================================================================
    # Reduced Mode & Boundaries
    # =========================================================================

    @rpc
    def set_reduced_mode(self, enable: int) -> tuple[int, str]:
        """Enable/disable reduced mode."""
        try:
            code = self.arm.set_reduced_mode(enable)
            return (
                code,
                f"Reduced mode {'enabled' if enable else 'disabled'}"
                if code == 0
                else f"Error code: {code}",
            )
        except Exception as e:
            return (-1, str(e))

    @rpc
    def set_reduced_max_tcp_speed(self, speed: float) -> tuple[int, str]:
        """Set maximum TCP speed in reduced mode."""
        try:
            code = self.arm.set_reduced_max_tcp_speed(speed)
            return (
                code,
                f"Reduced max TCP speed set to {speed}" if code == 0 else f"Error code: {code}",
            )
        except Exception as e:
            return (-1, str(e))

    @rpc
    def set_reduced_max_joint_speed(self, speed: float) -> tuple[int, str]:
        """Set maximum joint speed in reduced mode."""
        try:
            code = self.arm.set_reduced_max_joint_speed(speed)
            return (
                code,
                f"Reduced max joint speed set to {speed}" if code == 0 else f"Error code: {code}",
            )
        except Exception as e:
            return (-1, str(e))

    @rpc
    def set_fence_mode(self, enable: int) -> tuple[int, str]:
        """Enable/disable fence mode."""
        try:
            code = self.arm.set_fence_mode(enable)
            return (
                code,
                f"Fence mode {'enabled' if enable else 'disabled'}"
                if code == 0
                else f"Error code: {code}",
            )
        except Exception as e:
            return (-1, str(e))

    # =========================================================================
    # TCP & Dynamics Configuration
    # =========================================================================

    @rpc
    def set_tcp_offset(self, offset: list[float]) -> tuple[int, str]:
        """Set TCP offset [x, y, z, roll, pitch, yaw]."""
        try:
            code = self.arm.set_tcp_offset(offset)
            return (code, "TCP offset set" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def set_tcp_load(self, weight: float, center_of_gravity: list[float]) -> tuple[int, str]:
        """Set TCP load (payload)."""
        try:
            code = self.arm.set_tcp_load(weight, center_of_gravity)
            return (code, f"TCP load set: {weight}kg" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def set_gravity_direction(self, direction: list[float]) -> tuple[int, str]:
        """Set gravity direction vector."""
        try:
            code = self.arm.set_gravity_direction(direction)
            return (code, "Gravity direction set" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def set_world_offset(self, offset: list[float]) -> tuple[int, str]:
        """Set world coordinate offset."""
        try:
            code = self.arm.set_world_offset(offset)
            return (code, "World offset set" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    # =========================================================================
    # Motion Parameters
    # =========================================================================

    @rpc
    def set_tcp_jerk(self, jerk: float) -> tuple[int, str]:
        """Set TCP jerk (mm/s³)."""
        try:
            code = self.arm.set_tcp_jerk(jerk)
            return (code, f"TCP jerk set to {jerk}" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def set_tcp_maxacc(self, acc: float) -> tuple[int, str]:
        """Set TCP maximum acceleration (mm/s²)."""
        try:
            code = self.arm.set_tcp_maxacc(acc)
            return (
                code,
                f"TCP max acceleration set to {acc}" if code == 0 else f"Error code: {code}",
            )
        except Exception as e:
            return (-1, str(e))

    @rpc
    def set_joint_jerk(self, jerk: float) -> tuple[int, str]:
        """Set joint jerk (rad/s³ or °/s³)."""
        try:
            code = self.arm.set_joint_jerk(jerk, is_radian=self.config.is_radian)
            return (code, f"Joint jerk set to {jerk}" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def set_joint_maxacc(self, acc: float) -> tuple[int, str]:
        """Set joint maximum acceleration (rad/s² or °/s²)."""
        try:
            code = self.arm.set_joint_maxacc(acc, is_radian=self.config.is_radian)
            return (
                code,
                f"Joint max acceleration set to {acc}" if code == 0 else f"Error code: {code}",
            )
        except Exception as e:
            return (-1, str(e))

    @rpc
    def set_pause_time(self, seconds: float) -> tuple[int, str]:
        """Set pause time for motion commands."""
        try:
            code = self.arm.set_pause_time(seconds)
            return (code, f"Pause time set to {seconds}s" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    # =========================================================================
    # Digital I/O (Tool GPIO)
    # =========================================================================

    @rpc
    def get_tgpio_digital(self, io_num: int) -> tuple[int, int | None]:
        """Get tool GPIO digital input value."""
        try:
            code, value = self.arm.get_tgpio_digital(io_num)
            return (code, value if code == 0 else None)
        except Exception:
            return (-1, None)

    @rpc
    def set_tgpio_digital(self, io_num: int, value: int) -> tuple[int, str]:
        """Set tool GPIO digital output value (0 or 1)."""
        try:
            code = self.arm.set_tgpio_digital(io_num, value)
            return (code, f"TGPIO {io_num} set to {value}" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    # =========================================================================
    # Digital I/O (Controller GPIO)
    # =========================================================================

    @rpc
    def get_cgpio_digital(self, io_num: int) -> tuple[int, int | None]:
        """Get controller GPIO digital input value."""
        try:
            code, value = self.arm.get_cgpio_digital(io_num)
            return (code, value if code == 0 else None)
        except Exception:
            return (-1, None)

    @rpc
    def set_cgpio_digital(self, io_num: int, value: int) -> tuple[int, str]:
        """Set controller GPIO digital output value (0 or 1)."""
        try:
            code = self.arm.set_cgpio_digital(io_num, value)
            return (code, f"CGPIO {io_num} set to {value}" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    # =========================================================================
    # Analog I/O
    # =========================================================================

    @rpc
    def get_tgpio_analog(self, io_num: int) -> tuple[int, float | None]:
        """Get tool GPIO analog input value."""
        try:
            code, value = self.arm.get_tgpio_analog(io_num)
            return (code, value if code == 0 else None)
        except Exception:
            return (-1, None)

    @rpc
    def get_cgpio_analog(self, io_num: int) -> tuple[int, float | None]:
        """Get controller GPIO analog input value."""
        try:
            code, value = self.arm.get_cgpio_analog(io_num)
            return (code, value if code == 0 else None)
        except Exception:
            return (-1, None)

    @rpc
    def set_cgpio_analog(self, io_num: int, value: float) -> tuple[int, str]:
        """Set controller GPIO analog output value."""
        try:
            code = self.arm.set_cgpio_analog(io_num, value)
            return (
                code,
                f"CGPIO analog {io_num} set to {value}" if code == 0 else f"Error code: {code}",
            )
        except Exception as e:
            return (-1, str(e))

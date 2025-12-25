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
System Control Component for PiperDriver.

Provides RPC methods for system-level control operations including:
- Enable/disable arm
- Mode control (drag teach, MIT control, etc.)
- Motion control
- Master/slave configuration
"""

from typing import Any

from dimos.core import rpc
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class SystemControlComponent:
    """
    Component providing system control RPC methods for PiperDriver.

    This component assumes the parent class has:
    - self.piper: C_PiperInterface_V2 instance
    - self.config: PiperDriverConfig instance
    """

    # Type hints for attributes expected from parent class
    piper: Any  # C_PiperInterface_V2 instance
    config: Any  # Config dict accessed as object

    @rpc
    def enable_servo_mode(self) -> tuple[bool, str]:
        """
        Enable servo mode.
        This enables the arm to receive motion commands.

        Returns:
            Tuple of (success, message)
        """
        try:
            result = self.piper.EnableArm()

            if result:
                logger.info("Servo mode enabled")
                return (True, "Servo mode enabled")
            else:
                logger.warning("Failed to enable servo mode")
                return (False, "Failed to enable servo mode")

        except Exception as e:
            logger.error(f"enable_servo_mode failed: {e}")
            return (False, str(e))

    @rpc
    def disable_servo_mode(self) -> tuple[bool, str]:
        """
        Disable servo mode.

        Returns:
            Tuple of (success, message)
        """
        try:
            result = self.piper.DisableArm()

            if result:
                logger.info("Servo mode disabled")
                return (True, "Servo mode disabled")
            else:
                logger.warning("Failed to disable servo mode")
                return (False, "Failed to disable servo mode")

        except Exception as e:
            logger.error(f"disable_servo_mode failed: {e}")
            return (False, str(e))

    @rpc
    def motion_enable(self, enable: bool = True) -> tuple[bool, str]:
        """Enable or disable arm motion."""
        try:
            if enable:
                result = self.piper.EnableArm()
                msg = "Motion enabled"
            else:
                result = self.piper.DisableArm()
                msg = "Motion disabled"

            if result:
                return (True, msg)
            else:
                return (False, f"Failed to {msg.lower()}")

        except Exception as e:
            return (False, str(e))

    @rpc
    def set_motion_ctrl_1(
        self,
        ctrl_mode: int = 0x00,
        move_mode: int = 0x00,
        move_spd_rate: int = 50,
        coor_mode: int = 0x00,
        reference_joint: int = 0x00,
    ) -> tuple[bool, str]:
        """
        Set motion control parameters (MotionCtrl_1).

        Args:
            ctrl_mode: Control mode
            move_mode: Movement mode
            move_spd_rate: Movement speed rate (0-100)
            coor_mode: Coordinate mode
            reference_joint: Reference joint

        Returns:
            Tuple of (success, message)
        """
        try:
            result = self.piper.MotionCtrl_1(
                ctrl_mode, move_mode, move_spd_rate, coor_mode, reference_joint
            )

            if result:
                return (True, "Motion control 1 parameters set successfully")
            else:
                return (False, "Failed to set motion control 1 parameters")

        except Exception as e:
            logger.error(f"set_motion_ctrl_1 failed: {e}")
            return (False, str(e))

    @rpc
    def set_motion_ctrl_2(
        self,
        limit_fun_en: int = 0x00,
        collis_detect_en: int = 0x00,
        friction_feed_en: int = 0x00,
        gravity_feed_en: int = 0x00,
        is_mit_mode: int = 0x00,
    ) -> tuple[bool, str]:
        """
        Set motion control parameters (MotionCtrl_2).

        Args:
            limit_fun_en: Limit function enable (0x00 = disabled, 0x01 = enabled)
            collis_detect_en: Collision detection enable
            friction_feed_en: Friction compensation enable
            gravity_feed_en: Gravity compensation enable
            is_mit_mode: MIT mode enable (0x00 = disabled, 0x01 = enabled)

        Returns:
            Tuple of (success, message)
        """
        try:
            result = self.piper.MotionCtrl_2(
                limit_fun_en,
                collis_detect_en,
                friction_feed_en,
                gravity_feed_en,
                is_mit_mode,
            )

            if result:
                return (True, "Motion control 2 parameters set successfully")
            else:
                return (False, "Failed to set motion control 2 parameters")

        except Exception as e:
            logger.error(f"set_motion_ctrl_2 failed: {e}")
            return (False, str(e))

    @rpc
    def set_mode_ctrl(
        self,
        drag_teach_en: int = 0x00,
        teach_record_en: int = 0x00,
    ) -> tuple[bool, str]:
        """
        Set mode control (drag teaching, recording, etc.).

        Args:
            drag_teach_en: Drag teaching enable (0x00 = disabled, 0x01 = enabled)
            teach_record_en: Teaching record enable

        Returns:
            Tuple of (success, message)
        """
        try:
            result = self.piper.ModeCtrl(drag_teach_en, teach_record_en)

            if result:
                mode_str = []
                if drag_teach_en == 0x01:
                    mode_str.append("drag teaching")
                if teach_record_en == 0x01:
                    mode_str.append("recording")

                if mode_str:
                    return (True, f"Mode control set: {', '.join(mode_str)} enabled")
                else:
                    return (True, "Mode control set: all modes disabled")
            else:
                return (False, "Failed to set mode control")

        except Exception as e:
            logger.error(f"set_mode_ctrl failed: {e}")
            return (False, str(e))

    @rpc
    def configure_master_slave(
        self,
        linkage_config: int,
        feedback_offset: int,
        ctrl_offset: int,
        linkage_offset: int,
    ) -> tuple[bool, str]:
        """
        Configure master/slave linkage.

        Args:
            linkage_config: Linkage configuration
            feedback_offset: Feedback offset
            ctrl_offset: Control offset
            linkage_offset: Linkage offset

        Returns:
            Tuple of (success, message)
        """
        try:
            result = self.piper.MasterSlaveConfig(
                linkage_config, feedback_offset, ctrl_offset, linkage_offset
            )

            if result:
                return (True, "Master/slave configuration set successfully")
            else:
                return (False, "Failed to set master/slave configuration")

        except Exception as e:
            logger.error(f"configure_master_slave failed: {e}")
            return (False, str(e))

    @rpc
    def search_firmware_version(self) -> tuple[bool, str]:
        """
        Search for firmware version.

        Returns:
            Tuple of (success, message)
        """
        try:
            result = self.piper.SearchPiperFirmwareVersion()

            if result:
                return (True, "Firmware version search initiated")
            else:
                return (False, "Failed to search firmware version")

        except Exception as e:
            logger.error(f"search_firmware_version failed: {e}")
            return (False, str(e))

    @rpc
    def piper_init(self) -> tuple[bool, str]:
        """
        Initialize Piper arm.

        Returns:
            Tuple of (success, message)
        """
        try:
            result = self.piper.PiperInit()

            if result:
                logger.info("Piper initialized")
                return (True, "Piper initialized successfully")
            else:
                logger.warning("Failed to initialize Piper")
                return (False, "Failed to initialize Piper")

        except Exception as e:
            logger.error(f"piper_init failed: {e}")
            return (False, str(e))

    @rpc
    def enable_piper(self) -> tuple[bool, str]:
        """
        Enable Piper (convenience method).

        Returns:
            Tuple of (success, message)
        """
        try:
            result = self.piper.EnablePiper()

            if result:
                logger.info("Piper enabled")
                return (True, "Piper enabled")
            else:
                logger.warning("Failed to enable Piper")
                return (False, "Failed to enable Piper")

        except Exception as e:
            logger.error(f"enable_piper failed: {e}")
            return (False, str(e))

    @rpc
    def disable_piper(self) -> tuple[bool, str]:
        """
        Disable Piper (convenience method).

        Returns:
            Tuple of (success, message)
        """
        try:
            result = self.piper.DisablePiper()

            if result:
                logger.info("Piper disabled")
                return (True, "Piper disabled")
            else:
                logger.warning("Failed to disable Piper")
                return (False, "Failed to disable Piper")

        except Exception as e:
            logger.error(f"disable_piper failed: {e}")
            return (False, str(e))

    # =========================================================================
    # Velocity Control Mode
    # =========================================================================

    @rpc
    def enable_velocity_control_mode(self) -> tuple[bool, str]:
        """
        Enable velocity control mode (integration-based).

        This switches the control loop to use velocity integration:
        - Velocity commands are integrated: position_target += velocity * dt
        - Integrated positions are sent to JointCtrl (standard position control)
        - Provides smooth velocity control interface while using proven position API

        Returns:
            Tuple of (success, message)
        """
        try:
            # Set config flag to enable velocity control
            # The control loop will integrate velocities to positions
            self.config.velocity_control = True

            logger.info("Velocity control mode enabled (integration-based)")
            return (True, "Velocity control mode enabled")

        except Exception as e:
            logger.error(f"enable_velocity_control_mode failed: {e}")
            self.config.velocity_control = False  # Revert on exception
            return (False, str(e))

    @rpc
    def disable_velocity_control_mode(self) -> tuple[bool, str]:
        """
        Disable velocity control mode and return to position control.

        Returns:
            Tuple of (success, message)
        """
        try:
            # Set config flag to disable velocity control
            # The control loop will switch back to standard position control mode
            self.config.velocity_control = False

            # Reset position target to allow re-initialization when re-enabled
            self._position_target_ = None

            logger.info("Position control mode enabled (velocity mode disabled)")
            return (True, "Position control mode enabled")

        except Exception as e:
            logger.error(f"disable_velocity_control_mode failed: {e}")
            self.config.velocity_control = True  # Revert on exception
            return (False, str(e))

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

"""Standard servo control component for manipulator drivers."""

import logging
import time
from typing import Any

from ..sdk_interface import BaseManipulatorSDK
from ..spec import ManipulatorCapabilities
from ..utils import SharedState
from . import component_api


class StandardServoComponent:
    """Servo control component that works with any SDK wrapper.

    This component provides standard servo/motor control methods that work
    consistently across all manipulator types. Methods decorated with @component_api
    are automatically exposed as RPC methods on the driver. It handles:
    - Servo enable/disable
    - Control mode switching
    - Emergency stop
    - Error recovery
    - Homing operations
    """

    def __init__(
        self,
        sdk: BaseManipulatorSDK | None = None,
        shared_state: SharedState | None = None,
        capabilities: ManipulatorCapabilities | None = None,
    ):
        """Initialize the servo component.

        Args:
            sdk: SDK wrapper instance (can be set later)
            shared_state: Shared state instance (can be set later)
            capabilities: Manipulator capabilities (can be set later)
        """
        self.sdk = sdk
        self.shared_state = shared_state
        self.capabilities = capabilities
        self.logger = logging.getLogger(self.__class__.__name__)

        # State tracking
        self.last_enable_time = 0.0
        self.last_disable_time = 0.0

    # ============= Initialization Methods (called by BaseDriver) =============

    def set_sdk(self, sdk: BaseManipulatorSDK) -> None:
        """Set the SDK wrapper instance."""
        self.sdk = sdk

    def set_shared_state(self, shared_state: SharedState) -> None:
        """Set the shared state instance."""
        self.shared_state = shared_state

    def set_capabilities(self, capabilities: ManipulatorCapabilities) -> None:
        """Set the capabilities instance."""
        self.capabilities = capabilities

    def initialize(self) -> None:
        """Initialize the component after all resources are set."""
        self.logger.debug("Servo component initialized")

    # ============= Component API Methods =============

    @component_api
    def enable_servo(self, check_errors: bool = True) -> dict[str, Any]:
        """Enable servo/motor control.

        Args:
            check_errors: If True, check for errors before enabling

        Returns:
            Dict with 'success' and optional 'error' keys
        """
        try:
            if self.sdk is None:
                return {"success": False, "error": "SDK not configured"}

            # Check if already enabled
            if self.sdk.are_servos_enabled():
                return {"success": True, "message": "Servos already enabled"}

            # Check for errors if requested
            if check_errors:
                error_code = self.sdk.get_error_code()
                if error_code != 0:
                    error_msg = self.sdk.get_error_message()
                    return {
                        "success": False,
                        "error": f"Cannot enable servos with active error: {error_msg} (code: {error_code})",
                    }

            # Enable servos
            success = self.sdk.enable_servos()

            if success:
                self.last_enable_time = time.time()
                if self.shared_state:
                    self.shared_state.is_enabled = True
                self.logger.info("Servos enabled successfully")
            else:
                self.logger.error("Failed to enable servos")

            return {"success": success}

        except Exception as e:
            self.logger.error(f"Error in enable_servo: {e}")
            return {"success": False, "error": str(e)}

    @component_api
    def disable_servo(self, stop_motion: bool = True) -> dict[str, Any]:
        """Disable servo/motor control.

        Args:
            stop_motion: If True, stop any ongoing motion first

        Returns:
            Dict with 'success' and optional 'error' keys
        """
        try:
            if self.sdk is None:
                return {"success": False, "error": "SDK not configured"}

            # Check if already disabled
            if not self.sdk.are_servos_enabled():
                return {"success": True, "message": "Servos already disabled"}

            # Stop motion if requested
            if stop_motion:
                self.sdk.stop_motion()
                time.sleep(0.1)  # Brief delay to ensure motion stopped

            # Disable servos
            success = self.sdk.disable_servos()

            if success:
                self.last_disable_time = time.time()
                if self.shared_state:
                    self.shared_state.is_enabled = False
                self.logger.info("Servos disabled successfully")
            else:
                self.logger.error("Failed to disable servos")

            return {"success": success}

        except Exception as e:
            self.logger.error(f"Error in disable_servo: {e}")
            return {"success": False, "error": str(e)}

    @component_api
    def toggle_servo(self) -> dict[str, Any]:
        """Toggle servo enable/disable state.

        Returns:
            Dict with 'success', 'enabled' state, and optional 'error' keys
        """
        try:
            if self.sdk is None:
                return {"success": False, "error": "SDK not configured"}

            current_state = self.sdk.are_servos_enabled()

            if current_state:
                result = self.disable_servo()
            else:
                result = self.enable_servo()

            if result["success"]:
                result["enabled"] = not current_state

            return result

        except Exception as e:
            self.logger.error(f"Error in toggle_servo: {e}")
            return {"success": False, "error": str(e)}

    @component_api
    def get_servo_state(self) -> dict[str, Any]:
        """Get current servo state.

        Returns:
            Dict with servo state information
        """
        try:
            if self.sdk is None:
                return {"success": False, "error": "SDK not configured"}

            enabled = self.sdk.are_servos_enabled()
            robot_state = self.sdk.get_robot_state()

            return {
                "enabled": enabled,
                "mode": robot_state.get("mode", 0),
                "state": robot_state.get("state", 0),
                "is_moving": robot_state.get("is_moving", False),
                "last_enable_time": self.last_enable_time,
                "last_disable_time": self.last_disable_time,
                "success": True,
            }

        except Exception as e:
            self.logger.error(f"Error in get_servo_state: {e}")
            return {"success": False, "error": str(e)}

    @component_api
    def emergency_stop(self) -> dict[str, Any]:
        """Execute emergency stop.

        Returns:
            Dict with 'success' and optional 'error' keys
        """
        try:
            if self.sdk is None:
                return {"success": False, "error": "SDK not configured"}

            # Execute e-stop
            success = self.sdk.emergency_stop()

            if success:
                # Update shared state
                if self.shared_state:
                    self.shared_state.update_robot_state(state=3)  # 3 = e-stop state
                    self.shared_state.is_enabled = False
                    self.shared_state.is_moving = False

                self.logger.warning("Emergency stop executed")
            else:
                self.logger.error("Failed to execute emergency stop")

            return {"success": success}

        except Exception as e:
            self.logger.error(f"Error in emergency_stop: {e}")
            # Try to stop motion as fallback
            try:
                if self.sdk is not None:
                    self.sdk.stop_motion()
            except:
                pass
            return {"success": False, "error": str(e)}

    @component_api
    def reset_emergency_stop(self) -> dict[str, Any]:
        """Reset from emergency stop state.

        Returns:
            Dict with 'success' and optional 'error' keys
        """
        try:
            if self.sdk is None:
                return {"success": False, "error": "SDK not configured"}

            # Clear errors first
            self.sdk.clear_errors()

            # Re-enable servos
            success = self.sdk.enable_servos()

            if success:
                if self.shared_state:
                    self.shared_state.update_robot_state(state=0)  # 0 = idle
                    self.shared_state.is_enabled = True

                self.logger.info("Emergency stop reset successfully")
            else:
                self.logger.error("Failed to reset emergency stop")

            return {"success": success}

        except Exception as e:
            self.logger.error(f"Error in reset_emergency_stop: {e}")
            return {"success": False, "error": str(e)}

    @component_api
    def set_control_mode(self, mode: str) -> dict[str, Any]:
        """Set control mode.

        Args:
            mode: Control mode ('position', 'velocity', 'torque', 'impedance')

        Returns:
            Dict with 'success' and optional 'error' keys
        """
        try:
            if self.sdk is None:
                return {"success": False, "error": "SDK not configured"}

            # Validate mode
            valid_modes = ["position", "velocity", "torque", "impedance"]
            if mode not in valid_modes:
                return {
                    "success": False,
                    "error": f"Invalid mode '{mode}'. Valid modes: {valid_modes}",
                }

            # Check if mode is supported
            if mode == "impedance" and self.capabilities:
                if not self.capabilities.has_impedance_control:
                    return {"success": False, "error": "Impedance control not supported"}

            # Set control mode
            success = self.sdk.set_control_mode(mode)

            if success:
                # Map mode string to integer
                mode_map = {"position": 0, "velocity": 1, "torque": 2, "impedance": 3}
                if self.shared_state:
                    self.shared_state.update_robot_state(mode=mode_map.get(mode, 0))

                self.logger.info(f"Control mode set to '{mode}'")

            return {"success": success}

        except Exception as e:
            self.logger.error(f"Error in set_control_mode: {e}")
            return {"success": False, "error": str(e)}

    @component_api
    def get_control_mode(self) -> dict[str, Any]:
        """Get current control mode.

        Returns:
            Dict with current mode and success status
        """
        try:
            if self.sdk is None:
                return {"success": False, "error": "SDK not configured"}

            mode = self.sdk.get_control_mode()

            if mode:
                return {"mode": mode, "success": True}
            else:
                # Try to get from robot state
                robot_state = self.sdk.get_robot_state()
                mode_int = robot_state.get("mode", 0)

                # Map integer to string
                mode_map = {0: "position", 1: "velocity", 2: "torque", 3: "impedance"}
                mode_str = mode_map.get(mode_int, "unknown")

                return {"mode": mode_str, "success": True}

        except Exception as e:
            self.logger.error(f"Error in get_control_mode: {e}")
            return {"success": False, "error": str(e)}

    @component_api
    def clear_errors(self) -> dict[str, Any]:
        """Clear any error states.

        Returns:
            Dict with 'success' and optional 'error' keys
        """
        try:
            if self.sdk is None:
                return {"success": False, "error": "SDK not configured"}

            # Clear errors via SDK
            success = self.sdk.clear_errors()

            if success:
                # Update shared state
                if self.shared_state:
                    self.shared_state.clear_errors()

                self.logger.info("Errors cleared successfully")
            else:
                self.logger.error("Failed to clear errors")

            return {"success": success}

        except Exception as e:
            self.logger.error(f"Error in clear_errors: {e}")
            return {"success": False, "error": str(e)}

    @component_api
    def reset_fault(self) -> dict[str, Any]:
        """Reset from fault state.

        This typically involves:
        1. Clearing errors
        2. Disabling servos
        3. Brief delay
        4. Re-enabling servos

        Returns:
            Dict with 'success' and optional 'error' keys
        """
        try:
            if self.sdk is None:
                return {"success": False, "error": "SDK not configured"}

            self.logger.info("Resetting fault state...")

            # Step 1: Clear errors
            if not self.sdk.clear_errors():
                return {"success": False, "error": "Failed to clear errors"}

            # Step 2: Disable servos if enabled
            if self.sdk.are_servos_enabled():
                if not self.sdk.disable_servos():
                    return {"success": False, "error": "Failed to disable servos"}

            # Step 3: Brief delay
            time.sleep(0.5)

            # Step 4: Re-enable servos
            if not self.sdk.enable_servos():
                return {"success": False, "error": "Failed to re-enable servos"}

            # Update shared state
            if self.shared_state:
                self.shared_state.update_robot_state(
                    state=0,  # idle
                    error_code=0,
                    error_message="",
                )
                self.shared_state.is_enabled = True

            self.logger.info("Fault reset successfully")
            return {"success": True}

        except Exception as e:
            self.logger.error(f"Error in reset_fault: {e}")
            return {"success": False, "error": str(e)}

    @component_api
    def home_robot(self, position: list[float] | None = None) -> dict[str, Any]:
        """Move robot to home position.

        Args:
            position: Optional home position in radians.
                     If None, uses zero position or configured home.

        Returns:
            Dict with 'success' and optional 'error' keys
        """
        try:
            if self.sdk is None:
                return {"success": False, "error": "SDK not configured"}

            # Determine home position
            if position is None:
                # Use configured home or zero position
                if self.capabilities:
                    position = [0.0] * self.capabilities.dof
                else:
                    # Get current DOF from joint state
                    current = self.sdk.get_joint_positions()
                    position = [0.0] * len(current)

            # Enable servos if needed
            if not self.sdk.are_servos_enabled():
                if not self.sdk.enable_servos():
                    return {"success": False, "error": "Failed to enable servos"}

            # Move to home position
            success = self.sdk.set_joint_positions(
                position,
                velocity=0.3,  # Slower speed for homing
                acceleration=0.3,
                wait=True,  # Wait for completion
            )

            if success:
                if self.shared_state:
                    self.shared_state.is_homed = True
                self.logger.info("Robot homed successfully")

            return {"success": success}

        except Exception as e:
            self.logger.error(f"Error in home_robot: {e}")
            return {"success": False, "error": str(e)}

    @component_api
    def brake_release(self) -> dict[str, Any]:
        """Release motor brakes (if applicable).

        Returns:
            Dict with 'success' and optional 'error' keys
        """
        try:
            # This is typically the same as enabling servos
            return self.enable_servo()

        except Exception as e:
            self.logger.error(f"Error in brake_release: {e}")
            return {"success": False, "error": str(e)}

    @component_api
    def brake_engage(self) -> dict[str, Any]:
        """Engage motor brakes (if applicable).

        Returns:
            Dict with 'success' and optional 'error' keys
        """
        try:
            # This is typically the same as disabling servos
            return self.disable_servo()

        except Exception as e:
            self.logger.error(f"Error in brake_engage: {e}")
            return {"success": False, "error": str(e)}

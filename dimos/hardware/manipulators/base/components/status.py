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

"""Standard status monitoring component for manipulator drivers."""

from collections import deque
from dataclasses import dataclass
import logging
import time
from typing import Any

from ..sdk_interface import BaseManipulatorSDK
from ..spec import ManipulatorCapabilities
from ..utils import SharedState
from . import component_api


@dataclass
class HealthMetrics:
    """Health metrics for monitoring."""

    update_rate: float = 0.0  # Hz
    command_rate: float = 0.0  # Hz
    error_rate: float = 0.0  # errors/minute
    uptime: float = 0.0  # seconds
    total_errors: int = 0
    total_commands: int = 0
    total_updates: int = 0


class StandardStatusComponent:
    """Status monitoring component that works with any SDK wrapper.

    This component provides standard status monitoring methods that work
    consistently across all manipulator types. Methods decorated with @component_api
    are automatically exposed as RPC methods on the driver. It handles:
    - Robot state queries
    - Error monitoring
    - Health metrics
    - System information
    - Force/torque monitoring (if supported)
    - Temperature monitoring (if supported)
    """

    def __init__(
        self,
        sdk: BaseManipulatorSDK | None = None,
        shared_state: SharedState | None = None,
        capabilities: ManipulatorCapabilities | None = None,
    ):
        """Initialize the status component.

        Args:
            sdk: SDK wrapper instance (can be set later)
            shared_state: Shared state instance (can be set later)
            capabilities: Manipulator capabilities (can be set later)
        """
        self.sdk = sdk
        self.shared_state = shared_state
        self.capabilities = capabilities
        self.logger = logging.getLogger(self.__class__.__name__)

        # Health monitoring
        self.start_time = time.time()
        self.health_metrics = HealthMetrics()

        # Rate calculation
        self.update_timestamps: deque[float] = deque(maxlen=100)
        self.command_timestamps: deque[float] = deque(maxlen=100)
        self.error_timestamps: deque[float] = deque(maxlen=100)

        # Error history
        self.error_history: deque[dict[str, Any]] = deque(maxlen=50)

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
        self.start_time = time.time()
        self.logger.debug("Status component initialized")

    def publish_state(self) -> None:
        """Called periodically to update metrics (by publisher thread)."""
        current_time = time.time()
        self.update_timestamps.append(current_time)
        self._update_health_metrics()

    # ============= Component API Methods =============

    @component_api
    def get_robot_state(self) -> dict[str, Any]:
        """Get comprehensive robot state.

        Returns:
            Dict with complete state information
        """
        try:
            if self.sdk is None:
                return {"success": False, "error": "SDK not configured"}

            current_time = time.time()

            # Get state from SDK
            robot_state = self.sdk.get_robot_state()

            # Get additional info
            error_msg = (
                self.sdk.get_error_message() if robot_state.get("error_code", 0) != 0 else ""
            )

            # Map state integer to string
            state_map = {0: "idle", 1: "moving", 2: "error", 3: "emergency_stop"}
            state_str = state_map.get(robot_state.get("state", 0), "unknown")

            # Map mode integer to string
            mode_map = {0: "position", 1: "velocity", 2: "torque", 3: "impedance"}
            mode_str = mode_map.get(robot_state.get("mode", 0), "unknown")

            result = {
                "state": state_str,
                "state_code": robot_state.get("state", 0),
                "mode": mode_str,
                "mode_code": robot_state.get("mode", 0),
                "error_code": robot_state.get("error_code", 0),
                "error_message": error_msg,
                "is_moving": robot_state.get("is_moving", False),
                "is_connected": self.sdk.is_connected(),
                "is_enabled": self.sdk.are_servos_enabled(),
                "timestamp": current_time,
                "success": True,
            }

            # Add shared state info if available
            if self.shared_state:
                result["is_homed"] = self.shared_state.is_homed
                result["last_update"] = self.shared_state.last_state_update
                result["last_command"] = self.shared_state.last_command_sent

            return result

        except Exception as e:
            self.logger.error(f"Error in get_robot_state: {e}")
            return {"success": False, "error": str(e)}

    @component_api
    def get_system_info(self) -> dict[str, Any]:
        """Get system information.

        Returns:
            Dict with system information
        """
        try:
            if self.sdk is None:
                return {"success": False, "error": "SDK not configured"}

            # Get manipulator info
            info = self.sdk.get_info()

            result = {
                "vendor": info.vendor,
                "model": info.model,
                "dof": info.dof,
                "firmware_version": info.firmware_version,
                "serial_number": info.serial_number,
                "success": True,
            }

            # Add capabilities if available
            if self.capabilities:
                result["capabilities"] = {
                    "dof": self.capabilities.dof,
                    "has_gripper": self.capabilities.has_gripper,
                    "has_force_torque": self.capabilities.has_force_torque,
                    "has_impedance_control": self.capabilities.has_impedance_control,
                    "has_cartesian_control": self.capabilities.has_cartesian_control,
                    "payload_mass": self.capabilities.payload_mass,
                    "reach": self.capabilities.reach,
                }

            return result

        except Exception as e:
            self.logger.error(f"Error in get_system_info: {e}")
            return {"success": False, "error": str(e)}

    @component_api
    def get_capabilities(self) -> dict[str, Any]:
        """Get manipulator capabilities.

        Returns:
            Dict with capability information
        """
        try:
            if not self.capabilities:
                return {"success": False, "error": "Capabilities not available"}

            return {
                "dof": self.capabilities.dof,
                "has_gripper": self.capabilities.has_gripper,
                "has_force_torque": self.capabilities.has_force_torque,
                "has_impedance_control": self.capabilities.has_impedance_control,
                "has_cartesian_control": self.capabilities.has_cartesian_control,
                "joint_limits_lower": self.capabilities.joint_limits_lower,
                "joint_limits_upper": self.capabilities.joint_limits_upper,
                "max_joint_velocity": self.capabilities.max_joint_velocity,
                "max_joint_acceleration": self.capabilities.max_joint_acceleration,
                "payload_mass": self.capabilities.payload_mass,
                "reach": self.capabilities.reach,
                "success": True,
            }

        except Exception as e:
            self.logger.error(f"Error in get_capabilities: {e}")
            return {"success": False, "error": str(e)}

    @component_api
    def get_error_state(self) -> dict[str, Any]:
        """Get detailed error state.

        Returns:
            Dict with error information
        """
        try:
            if self.sdk is None:
                return {"success": False, "error": "SDK not configured"}

            error_code = self.sdk.get_error_code()
            error_msg = self.sdk.get_error_message()

            result = {
                "has_error": error_code != 0,
                "error_code": error_code,
                "error_message": error_msg,
                "error_history": list(self.error_history),
                "total_errors": self.health_metrics.total_errors,
                "success": True,
            }

            # Add last error time from shared state
            if self.shared_state:
                result["last_error_time"] = self.shared_state.last_error_time

            return result

        except Exception as e:
            self.logger.error(f"Error in get_error_state: {e}")
            return {"success": False, "error": str(e)}

    @component_api
    def get_health_metrics(self) -> dict[str, Any]:
        """Get health metrics.

        Returns:
            Dict with health metrics
        """
        try:
            self._update_health_metrics()

            return {
                "uptime": self.health_metrics.uptime,
                "update_rate": self.health_metrics.update_rate,
                "command_rate": self.health_metrics.command_rate,
                "error_rate": self.health_metrics.error_rate,
                "total_updates": self.health_metrics.total_updates,
                "total_commands": self.health_metrics.total_commands,
                "total_errors": self.health_metrics.total_errors,
                "is_healthy": self._is_healthy(),
                "timestamp": time.time(),
                "success": True,
            }

        except Exception as e:
            self.logger.error(f"Error in get_health_metrics: {e}")
            return {"success": False, "error": str(e)}

    @component_api
    def get_statistics(self) -> dict[str, Any]:
        """Get operation statistics.

        Returns:
            Dict with statistics
        """
        try:
            stats = {}

            # Get stats from shared state
            if self.shared_state:
                stats.update(self.shared_state.get_statistics())

            # Add component stats
            stats["uptime"] = time.time() - self.start_time
            stats["health_metrics"] = {
                "update_rate": self.health_metrics.update_rate,
                "command_rate": self.health_metrics.command_rate,
                "error_rate": self.health_metrics.error_rate,
            }

            stats["success"] = True
            return stats

        except Exception as e:
            self.logger.error(f"Error in get_statistics: {e}")
            return {"success": False, "error": str(e)}

    @component_api
    def check_connection(self) -> dict[str, Any]:
        """Check connection status.

        Returns:
            Dict with connection status
        """
        try:
            if self.sdk is None:
                return {"success": False, "error": "SDK not configured"}

            connected = self.sdk.is_connected()

            result: dict[str, Any] = {
                "connected": connected,
                "timestamp": time.time(),
                "success": True,
            }

            # Try to get more info if connected
            if connected:
                try:
                    # Try a simple query to verify connection
                    self.sdk.get_error_code()
                    result["verified"] = True
                except:
                    result["verified"] = False
                    result["message"] = "Connected but cannot communicate"

            return result

        except Exception as e:
            self.logger.error(f"Error in check_connection: {e}")
            return {"success": False, "error": str(e)}

    # ============= Force/Torque Monitoring (Optional) =============

    @component_api
    def get_force_torque(self) -> dict[str, Any]:
        """Get force/torque sensor data.

        Returns:
            Dict with F/T data if available
        """
        try:
            # Check if F/T is supported
            if not self.capabilities or not self.capabilities.has_force_torque:
                return {"success": False, "error": "Force/torque sensor not available"}

            if self.sdk is None:
                return {"success": False, "error": "SDK not configured"}

            ft_data = self.sdk.get_force_torque()

            if ft_data:
                return {
                    "force": ft_data[:3] if len(ft_data) >= 3 else None,  # [fx, fy, fz]
                    "torque": ft_data[3:6] if len(ft_data) >= 6 else None,  # [tx, ty, tz]
                    "data": ft_data,
                    "timestamp": time.time(),
                    "success": True,
                }
            else:
                return {"success": False, "error": "Failed to read F/T sensor"}

        except Exception as e:
            self.logger.error(f"Error in get_force_torque: {e}")
            return {"success": False, "error": str(e)}

    @component_api
    def zero_force_torque(self) -> dict[str, Any]:
        """Zero the force/torque sensor.

        Returns:
            Dict with success status
        """
        try:
            # Check if F/T is supported
            if not self.capabilities or not self.capabilities.has_force_torque:
                return {"success": False, "error": "Force/torque sensor not available"}

            if self.sdk is None:
                return {"success": False, "error": "SDK not configured"}

            success = self.sdk.zero_force_torque()
            return {"success": success}

        except Exception as e:
            self.logger.error(f"Error in zero_force_torque: {e}")
            return {"success": False, "error": str(e)}

    # ============= I/O Monitoring (Optional) =============

    @component_api
    def get_digital_inputs(self) -> dict[str, Any]:
        """Get digital input states.

        Returns:
            Dict with digital input states
        """
        try:
            if self.sdk is None:
                return {"success": False, "error": "SDK not configured"}

            inputs = self.sdk.get_digital_inputs()

            if inputs is not None:
                return {"inputs": inputs, "timestamp": time.time(), "success": True}
            else:
                return {"success": False, "error": "Digital inputs not available"}

        except Exception as e:
            self.logger.error(f"Error in get_digital_inputs: {e}")
            return {"success": False, "error": str(e)}

    @component_api
    def set_digital_outputs(self, outputs: dict[str, bool]) -> dict[str, Any]:
        """Set digital output states.

        Args:
            outputs: Dict of output_id: bool

        Returns:
            Dict with success status
        """
        try:
            if self.sdk is None:
                return {"success": False, "error": "SDK not configured"}

            success = self.sdk.set_digital_outputs(outputs)
            return {"success": success}

        except Exception as e:
            self.logger.error(f"Error in set_digital_outputs: {e}")
            return {"success": False, "error": str(e)}

    @component_api
    def get_analog_inputs(self) -> dict[str, Any]:
        """Get analog input values.

        Returns:
            Dict with analog input values
        """
        try:
            if self.sdk is None:
                return {"success": False, "error": "SDK not configured"}

            inputs = self.sdk.get_analog_inputs()

            if inputs is not None:
                return {"inputs": inputs, "timestamp": time.time(), "success": True}
            else:
                return {"success": False, "error": "Analog inputs not available"}

        except Exception as e:
            self.logger.error(f"Error in get_analog_inputs: {e}")
            return {"success": False, "error": str(e)}

    # ============= Gripper Status (Optional) =============

    @component_api
    def get_gripper_state(self) -> dict[str, Any]:
        """Get gripper state.

        Returns:
            Dict with gripper state
        """
        try:
            # Check if gripper is supported
            if not self.capabilities or not self.capabilities.has_gripper:
                return {"success": False, "error": "Gripper not available"}

            if self.sdk is None:
                return {"success": False, "error": "SDK not configured"}

            position = self.sdk.get_gripper_position()

            if position is not None:
                result: dict[str, Any] = {
                    "position": position,  # meters
                    "timestamp": time.time(),
                    "success": True,
                }

                # Add from shared state if available
                if self.shared_state and self.shared_state.gripper_force is not None:
                    result["force"] = self.shared_state.gripper_force

                return result
            else:
                return {"success": False, "error": "Failed to get gripper state"}

        except Exception as e:
            self.logger.error(f"Error in get_gripper_state: {e}")
            return {"success": False, "error": str(e)}

    # ============= Helper Methods =============

    def _update_health_metrics(self) -> None:
        """Update health metrics based on recent data."""
        current_time = time.time()

        # Update uptime
        self.health_metrics.uptime = current_time - self.start_time

        # Calculate update rate
        if len(self.update_timestamps) > 1:
            time_span = self.update_timestamps[-1] - self.update_timestamps[0]
            if time_span > 0:
                self.health_metrics.update_rate = len(self.update_timestamps) / time_span

        # Calculate command rate
        if len(self.command_timestamps) > 1:
            time_span = self.command_timestamps[-1] - self.command_timestamps[0]
            if time_span > 0:
                self.health_metrics.command_rate = len(self.command_timestamps) / time_span

        # Calculate error rate (errors per minute)
        recent_errors = [t for t in self.error_timestamps if current_time - t < 60]
        self.health_metrics.error_rate = len(recent_errors)

        # Update totals from shared state
        if self.shared_state:
            stats = self.shared_state.get_statistics()
            self.health_metrics.total_updates = stats.get("state_read_count", 0)
            self.health_metrics.total_commands = stats.get("command_sent_count", 0)
            self.health_metrics.total_errors = stats.get("error_count", 0)

    def _is_healthy(self) -> bool:
        """Check if system is healthy based on metrics."""
        # Check update rate (should be > 10 Hz)
        if self.health_metrics.update_rate < 10:
            return False

        # Check error rate (should be < 10 per minute)
        if self.health_metrics.error_rate > 10:
            return False

        # Check SDK is configured
        if self.sdk is None:
            return False

        # Check connection
        if not self.sdk.is_connected():
            return False

        # Check for persistent errors
        if self.sdk.get_error_code() != 0:
            return False

        return True

    def record_error(self, error_code: int, error_msg: str) -> None:
        """Record an error occurrence.

        Args:
            error_code: Error code
            error_msg: Error message
        """
        current_time = time.time()
        self.error_timestamps.append(current_time)
        self.error_history.append(
            {"code": error_code, "message": error_msg, "timestamp": current_time}
        )

    def record_command(self) -> None:
        """Record a command occurrence."""
        self.command_timestamps.append(time.time())

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

"""Piper driver using the generalized component-based architecture."""

import logging
import time
from typing import Any

from dimos.hardware.manipulators.base import (
    BaseManipulatorDriver,
    StandardMotionComponent,
    StandardServoComponent,
    StandardStatusComponent,
)

from .piper_wrapper import PiperSDKWrapper

logger = logging.getLogger(__name__)


class PiperDriver(BaseManipulatorDriver):
    """Piper driver using component-based architecture.

    This driver supports the Piper 6-DOF manipulator via CAN bus.
    All the complex logic is handled by the base class and standard components.
    This file just assembles the pieces.
    """

    def __init__(self, **kwargs: Any) -> None:
        """Initialize the Piper driver.

        Args:
            **kwargs: Arguments for Module initialization.
                Driver configuration can be passed via 'config' keyword arg:
                - can_port: CAN interface name (e.g., 'can0')
                - has_gripper: Whether gripper is attached
                - enable_on_start: Whether to enable servos on start
        """
        # Extract driver-specific config from kwargs
        config: dict[str, Any] = kwargs.pop("config", {})

        # Extract driver-specific params that might be passed directly
        driver_params = [
            "can_port",
            "has_gripper",
            "enable_on_start",
            "control_rate",
            "monitor_rate",
        ]
        for param in driver_params:
            if param in kwargs:
                config[param] = kwargs.pop(param)

        logger.info(f"Initializing PiperDriver with config: {config}")

        # Create SDK wrapper
        sdk = PiperSDKWrapper()

        # Create standard components
        components = [
            StandardMotionComponent(sdk),
            StandardServoComponent(sdk),
            StandardStatusComponent(sdk),
        ]

        # Optional: Add gripper component if configured
        # if config.get('has_gripper', False):
        #     from dimos.hardware.manipulators.base.components import StandardGripperComponent
        #     components.append(StandardGripperComponent(sdk))

        # Remove any kwargs that would conflict with explicit arguments
        kwargs.pop("sdk", None)
        kwargs.pop("components", None)
        kwargs.pop("name", None)

        # Initialize base driver with SDK and components
        super().__init__(
            sdk=sdk, components=components, config=config, name="PiperDriver", **kwargs
        )

        # Initialize position target for velocity integration
        self._position_target: list[float] | None = None
        self._last_velocity_time: float = 0.0

        # Enable on start if configured
        if config.get("enable_on_start", False):
            logger.info("Enabling Piper servos on start...")
            servo_component = self.get_component(StandardServoComponent)
            if servo_component:
                result = servo_component.enable_servo()
                if result["success"]:
                    logger.info("Piper servos enabled successfully")
                else:
                    logger.warning(f"Failed to enable servos: {result.get('error')}")

        logger.info("PiperDriver initialized successfully")

    def _process_command(self, command: Any) -> None:
        """Override to implement velocity control via position integration.

        Args:
            command: Command to process
        """
        # Handle velocity commands specially for Piper
        if command.type == "velocity":
            # Piper doesn't have native velocity control - integrate to position
            current_time = time.time()

            # Initialize position target from current state on first velocity command
            if self._position_target is None:
                positions = self.shared_state.joint_positions
                if positions:
                    self._position_target = list(positions)
                    logger.info(
                        f"Velocity control: Initialized position target from current state: {self._position_target}"
                    )
                else:
                    logger.warning("Cannot start velocity control - no current position available")
                    return

            # Calculate dt since last velocity command
            if self._last_velocity_time > 0:
                dt = current_time - self._last_velocity_time
            else:
                dt = 1.0 / self.control_rate  # Use nominal period for first command

            self._last_velocity_time = current_time

            # Integrate velocity to position: pos += vel * dt
            velocities = command.data["velocities"]
            for i in range(min(len(velocities), len(self._position_target))):
                self._position_target[i] += velocities[i] * dt

            # Send integrated position command
            success = self.sdk.set_joint_positions(
                self._position_target,
                velocity=1.0,  # Use max velocity for responsiveness
                acceleration=1.0,
                wait=False,
            )

            if success:
                self.shared_state.target_positions = self._position_target
                self.shared_state.target_velocities = velocities

        else:
            # Reset velocity integration when switching to position mode
            if command.type == "position":
                self._position_target = None
                self._last_velocity_time = 0.0

            # Use base implementation for other command types
            super()._process_command(command)


# Blueprint configuration for the driver
def get_blueprint() -> dict[str, Any]:
    """Get the blueprint configuration for the Piper driver.

    Returns:
        Dictionary with blueprint configuration
    """
    return {
        "name": "PiperDriver",
        "class": PiperDriver,
        "config": {
            "can_port": "can0",  # Default CAN interface
            "has_gripper": True,  # Piper usually has gripper
            "enable_on_start": True,  # Enable servos on startup
            "control_rate": 100,  # Hz - control loop + joint feedback
            "monitor_rate": 10,  # Hz - robot state monitoring
        },
        "inputs": {
            "joint_position_command": "JointCommand",
            "joint_velocity_command": "JointCommand",
        },
        "outputs": {
            "joint_state": "JointState",
            "robot_state": "RobotState",
        },
        "rpc_methods": [
            # Motion control
            "move_joint",
            "move_joint_velocity",
            "move_joint_effort",
            "stop_motion",
            "get_joint_state",
            "get_joint_limits",
            "get_velocity_limits",
            "set_velocity_scale",
            "set_acceleration_scale",
            "move_cartesian",
            "get_cartesian_state",
            "execute_trajectory",
            "stop_trajectory",
            # Servo control
            "enable_servo",
            "disable_servo",
            "toggle_servo",
            "get_servo_state",
            "emergency_stop",
            "reset_emergency_stop",
            "set_control_mode",
            "get_control_mode",
            "clear_errors",
            "reset_fault",
            "home_robot",
            "brake_release",
            "brake_engage",
            # Status monitoring
            "get_robot_state",
            "get_system_info",
            "get_capabilities",
            "get_error_state",
            "get_health_metrics",
            "get_statistics",
            "check_connection",
            "get_force_torque",
            "zero_force_torque",
            "get_digital_inputs",
            "set_digital_outputs",
            "get_analog_inputs",
            "get_gripper_state",
        ],
    }


# Expose blueprint for declarative composition (compatible with dimos framework)
piper_driver = PiperDriver.blueprint

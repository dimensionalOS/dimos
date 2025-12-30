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

"""XArm driver using the generalized component-based architecture."""

import logging
from typing import Any

from dimos.hardware.manipulators.base import (
    BaseManipulatorDriver,
    StandardMotionComponent,
    StandardServoComponent,
    StandardStatusComponent,
)

from .xarm_wrapper import XArmSDKWrapper

logger = logging.getLogger(__name__)


class XArmDriver(BaseManipulatorDriver):
    """XArm driver using component-based architecture.

    This driver supports XArm5, XArm6, and XArm7 models.
    All the complex logic is handled by the base class and standard components.
    This file just assembles the pieces.
    """

    def __init__(self, **kwargs: Any) -> None:
        """Initialize the XArm driver.

        Args:
            **kwargs: Arguments for Module initialization.
                Driver configuration can be passed via 'config' keyword arg:
                - ip: IP address of the XArm controller
                - dof: Degrees of freedom (5, 6, or 7)
                - has_gripper: Whether gripper is attached
                - has_force_torque: Whether F/T sensor is attached
        """
        # Extract driver-specific config from kwargs
        config: dict[str, Any] = kwargs.pop("config", {})

        # Extract driver-specific params that might be passed directly
        driver_params = [
            "ip",
            "dof",
            "has_gripper",
            "has_force_torque",
            "control_rate",
            "monitor_rate",
        ]
        for param in driver_params:
            if param in kwargs:
                config[param] = kwargs.pop(param)

        logger.info(f"Initializing XArmDriver with config: {config}")

        # Create SDK wrapper
        sdk = XArmSDKWrapper()

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

        # Optional: Add force/torque component if configured
        # if config.get('has_force_torque', False):
        #     from dimos.hardware.manipulators.base.components import StandardForceTorqueComponent
        #     components.append(StandardForceTorqueComponent(sdk))

        # Remove any kwargs that would conflict with explicit arguments
        kwargs.pop("sdk", None)
        kwargs.pop("components", None)
        kwargs.pop("name", None)

        # Initialize base driver with SDK and components
        super().__init__(sdk=sdk, components=components, config=config, name="XArmDriver", **kwargs)

        logger.info("XArmDriver initialized successfully")


# Blueprint configuration for the driver
def get_blueprint() -> dict[str, Any]:
    """Get the blueprint configuration for the XArm driver.

    Returns:
        Dictionary with blueprint configuration
    """
    return {
        "name": "XArmDriver",
        "class": XArmDriver,
        "config": {
            "ip": "192.168.1.210",  # Default IP
            "dof": 7,  # Default to 7-DOF
            "has_gripper": False,
            "has_force_torque": False,
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
xarm_driver = XArmDriver.blueprint

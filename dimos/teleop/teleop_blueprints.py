# Copyright 2025-2026 Dimensional Inc.
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
Blueprints for Quest3 teleoperation with XArm.

This module provides declarative blueprints for combining Quest3TeleopModule,
TeleopArmController, and XArmDriver for VR teleoperation.

Architecture:
    Quest3TeleopModule (VR calibration, delta computation)
        ↓ X button → calibrate VR
        ↓ Computes: delta = current_controller - initial_controller
        ↓ Publishes: delta poses (Pose)
        ↓
    TeleopArmController (Robot calibration, delta application)
        ↓ First delta → auto-calibrate robot
        ↓ Computes: target = initial_robot + delta
        ↓ Publishes: cartesian commands (Pose)
        ↓
    XArmDriver (Robot control)

Usage:
    # Programmatically:
    from dimos.teleop.teleop_blueprints import quest3_xarm6_teleop
    coordinator = quest3_xarm6_teleop.build()
    coordinator.loop()

    # Or build your own composition:
    from dimos.teleop.quest3.teleop_module import quest3_teleop_module
    from dimos.teleop.teleop_arm_controller import teleop_arm_controller
    from dimos.hardware.manipulators.xarm.xarm_driver import xarm_driver
    from dimos.core.blueprints import autoconnect

    my_system = autoconnect(
        quest3_teleop_module(
            driver_module_name="XArmDriver",
            signaling_port=8443,
        ),
        teleop_arm_controller(
            driver_module_name="XArmDriver",
            enable_left_arm=True,
        ),
        xarm_driver(
            ip="192.168.1.210",
            dof=6,
            connection_type="hardware",
        ),
    )
"""

from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.hardware.manipulators.xarm.xarm_driver import XArmDriver, xarm_driver
from dimos.msgs.geometry_msgs import Pose, PoseStamped
from dimos.msgs.std_msgs import Bool
from dimos.teleop.quest3.teleop_module import Quest3TeleopModule, quest3_teleop_module
from dimos.teleop.teleop_arm_controller import TeleopArmController, teleop_arm_controller

# =============================================================================
# Quest3 + XArm6 Teleoperation Blueprint
# =============================================================================
# Combines:
#   - Quest3TeleopModule: VR calibration + delta computation
#   - TeleopArmController: Robot calibration + delta application
#   - XArmDriver: Hardware/sim interface
#
# Data flow:
#   Quest3TeleopModule.left_controller_delta ──► TeleopArmController.left_controller_delta
#   Quest3TeleopModule.right_controller_delta ──► TeleopArmController.right_controller_delta
#   Quest3TeleopModule.left_trigger ──► TeleopArmController.left_trigger
#   Quest3TeleopModule.right_trigger ──► TeleopArmController.right_trigger
#   TeleopArmController.left_cartesian_command ──► XArmDriver.cartesian_command
#   TeleopArmController (RPC) ──► XArmDriver.get_cartesian_state (auto-calibration)
# =============================================================================

quest3_xarm6_teleop = (
    autoconnect(
        quest3_teleop_module(
            driver_module_name="XArmDriver",
            signaling_host="0.0.0.0",
            signaling_port=8443,  # HTTPS port (required for WebXR)
            use_https=True,  # Enable HTTPS for Quest 3 WebXR
            enable_left_arm=True,
            enable_right_arm=False,
        ),
        teleop_arm_controller(
            driver_module_name="XArmDriver",
            control_frequency=20.0,  # 20 Hz for consistent control
            enable_left_arm=True,
            enable_right_arm=False,
        ),
        xarm_driver(
            ip="192.168.1.210",
            dof=6,
            has_gripper=False,
            has_force_torque=False,
            has_cartesian_control=True,  # Enable Cartesian control for teleop
            control_rate=20,  # 20 Hz to match teleop frequency
            monitor_rate=10,
            connection_type="hardware",  # Change to "hardware" for real robot
        ),
    )
    .remappings(
        [
            # Map TeleopArmController's left_cartesian_command to XArmDriver's cartesian_command
            (TeleopArmController, "left_cartesian_command", "cartesian_command"),
        ]
    )
    .transports(
        {
            # Delta poses from Quest3TeleopModule to TeleopArmController
            ("left_controller_delta", PoseStamped): LCMTransport(
                "/quest3/left_controller_delta", PoseStamped
            ),
            ("right_controller_delta", PoseStamped): LCMTransport(
                "/quest3/right_controller_delta", PoseStamped
            ),
            # Trigger states
            ("left_trigger", Bool): LCMTransport("/quest3/left_trigger", Bool),
            ("right_trigger", Bool): LCMTransport("/quest3/right_trigger", Bool),
            # Cartesian commands to robot
            ("left_cartesian_command", Pose): LCMTransport("/xarm/cartesian_command", Pose),
            ("right_cartesian_command", Pose): LCMTransport("/xarm/right_cartesian_command", Pose),
            ("cartesian_command", Pose): LCMTransport("/xarm/cartesian_command", Pose),
        }
    )
)


__all__ = [
    "quest3_xarm6_teleop",
]

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
Blueprints for Quest3 teleoperation.

This module provides declarative blueprints for combining Quest3TeleopModule
and TeleopRobotController for VR teleoperation with any robot arm.

Architecture:
    Quest3TeleopModule (VR calibration, delta computation)
        ↓ X button → calibrate VR
        ↓ Computes: delta = current_controller - initial_controller
        ↓ Publishes: delta poses (PoseStamped)
        ↓
    TeleopRobotController (Robot calibration, delta application)
        ↓ First delta → auto-calibrate robot
        ↓ Computes: target = initial_robot + delta
        ↓ Publishes: cartesian commands (PoseStamped)
        ↓
    Robot Driver (any manipulator driver)

Usage:
    # Programmatically:
    from dimos.teleop.teleop_blueprints import quest3_teleop
    coordinator = quest3_teleop.build()
    coordinator.loop()

    # Or build your own composition with a specific driver:
    from dimos.teleop.quest3.quest3_teleop_module import quest3_teleop_module
    from dimos.teleop.teleop_robot_controller import teleop_robot_controller
    from dimos.core.blueprints import autoconnect

    my_system = autoconnect(
        quest3_teleop_module(
            signaling_port=8443,
        ),
        teleop_robot_controller(
            driver_module_name="MyRobotDriver",
            enable_left_arm=True,
        ),
        my_robot_driver(...),
    )
"""

from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.std_msgs import Bool, Float32
from dimos.teleop.quest3.quest3_teleop_module import quest3_teleop_module
from dimos.teleop.teleop_robot_controller import teleop_robot_controller

# =============================================================================
# Quest3 Teleoperation Blueprint
# =============================================================================
# Combines:
#   - Quest3TeleopModule: VR calibration + delta computation
#   - TeleopRobotController: Robot calibration + delta application
#
# Data flow:
#   Quest3TeleopModule.left_controller_delta ──► TeleopRobotController.left_controller_delta
#   Quest3TeleopModule.right_controller_delta ──► TeleopRobotController.right_controller_delta
#   Quest3TeleopModule.left_trigger ──► TeleopRobotController.left_trigger
#   Quest3TeleopModule.right_trigger ──► TeleopRobotController.right_trigger
#   TeleopRobotController.left_cartesian_command ──► Robot Driver (via LCM)
#   TeleopRobotController.right_cartesian_command ──► Robot Driver (via LCM)
# =============================================================================

quest3_teleop = autoconnect(
    quest3_teleop_module(
        signaling_host="0.0.0.0",
        signaling_port=8443,  # HTTPS port (required for WebXR)
        use_https=True,  # Enable HTTPS for Quest 3 WebXR
        enable_left_arm=True,
        enable_right_arm=True,
        visualize_in_rerun=True,
        safety_limits=True,
    ),
    teleop_robot_controller(
        driver_module_name="DummyDriver",
        dummy_driver=True,  # Skip RPC calls, use zeros for initial pose
        control_frequency=50.0,  # Hz - control loop frequency
        enable_left_arm=True,
        enable_right_arm=True,
    ),
).transports(
    {
        # Delta poses from Quest3TeleopModule to TeleopRobotController
        ("left_controller_delta", PoseStamped): LCMTransport(
            "/quest3/left_controller_delta", PoseStamped
        ),
        ("right_controller_delta", PoseStamped): LCMTransport(
            "/quest3/right_controller_delta", PoseStamped
        ),
            # Gripper values (0.0-1.0)
            ("left_trigger", Float32): LCMTransport("/quest3/left_trigger", Float32),
            ("right_trigger", Float32): LCMTransport("/quest3/right_trigger", Float32),
    }
)


__all__ = [
    "quest3_teleop",
]

#!/usr/bin/env python3
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
Example: Topic-Based Cartesian Motion Control with xArm

Demonstrates topic-based Cartesian space motion control. The controller
subscribes to /target_pose and automatically moves to received targets.

This example shows:
1. Deploy xArm driver with LCM transports
2. Deploy CartesianMotionController with LCM transports
3. Configure controller to subscribe to /target_pose topic
4. Keep system running to process incoming targets

Use target_setter.py to publish target poses to /target_pose topic.

Pattern matches: interactive_control.py + sample_trajectory_generator.py
"""

import signal
import time

from dimos import core
from dimos.hardware.manipulators.xarm import XArmDriver
from dimos.manipulation.control import CartesianMotionController
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.sensor_msgs import JointCommand, JointState, RobotState

# Global flag for graceful shutdown
shutdown_requested = False


def signal_handler(sig, frame):
    """Handle Ctrl+C for graceful shutdown."""
    global shutdown_requested
    print("\n\nShutdown requested...")
    shutdown_requested = True


def main():
    """
    Deploy and run topic-based Cartesian motion control system.

    The system subscribes to /target_pose and automatically moves
    the robot to received target poses.
    """

    # Register signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # =========================================================================
    # Step 1: Start dimos cluster
    # =========================================================================
    print("=" * 80)
    print("Topic-Based Cartesian Motion Control")
    print("=" * 80)
    print("\nStarting dimos cluster...")
    dimos = core.start(1)  # Start with 1 worker

    try:
        # =========================================================================
        # Step 2: Deploy xArm driver
        # =========================================================================
        print("\nDeploying xArm driver...")
        arm_driver = dimos.deploy(
            XArmDriver,
            ip_address="192.168.1.210",
            xarm_type="xarm6",
            report_type="dev",
            enable_on_start=True,
        )

        # Set up driver transports
        arm_driver.joint_state.transport = core.LCMTransport("/xarm/joint_states", JointState)
        arm_driver.robot_state.transport = core.LCMTransport("/xarm/robot_state", RobotState)
        arm_driver.joint_position_command.transport = core.LCMTransport(
            "/xarm/joint_position_command", JointCommand
        )
        arm_driver.joint_velocity_command.transport = core.LCMTransport(
            "/xarm/joint_velocity_command", JointCommand
        )

        print("Starting xArm driver...")
        arm_driver.start()

        # =========================================================================
        # Step 3: Deploy Cartesian motion controller
        # =========================================================================
        print("\nDeploying Cartesian motion controller...")
        controller = dimos.deploy(
            CartesianMotionController,
            arm_driver=arm_driver,
            control_frequency=20.0,
            position_kp=1.0,
            position_kd=0.1,
            orientation_kp=2.0,
            orientation_kd=0.2,
            max_linear_velocity=0.15,
            max_angular_velocity=0.8,
            position_tolerance=0.002,
            orientation_tolerance=0.02,
            velocity_control_mode=True,
        )

        # Set up controller transports
        controller.joint_state.transport = core.LCMTransport("/xarm/joint_states", JointState)
        controller.robot_state.transport = core.LCMTransport("/xarm/robot_state", RobotState)
        controller.joint_position_command.transport = core.LCMTransport(
            "/xarm/joint_position_command", JointCommand
        )

        # IMPORTANT: Configure controller to subscribe to /target_pose topic
        controller.target_pose.transport = core.LCMTransport("/target_pose", PoseStamped)

        # Publish current pose for target setters to use
        controller.current_pose.transport = core.LCMTransport("/xarm/current_pose", PoseStamped)

        print("Starting controller...")
        controller.start()

        # =========================================================================
        # Step 4: Keep system running
        # =========================================================================
        print("\n" + "=" * 80)
        print("✓ System ready!")
        print("=" * 80)
        print("\nController is now listening to /target_pose topic")
        print("Use target_setter.py to publish target poses")
        print("\nPress Ctrl+C to shutdown")
        print("=" * 80 + "\n")

        # Keep running until shutdown requested
        while not shutdown_requested:
            time.sleep(0.5)

        # =========================================================================
        # Step 5: Clean shutdown
        # =========================================================================
        print("\nShutting down...")
        print("Stopping controller...")
        controller.stop()
        print("Stopping driver...")
        arm_driver.stop()
        print("✓ Shutdown complete")

    finally:
        # Always stop dimos cluster
        print("Stopping dimos cluster...")
        dimos.stop()


if __name__ == "__main__":
    """
    Topic-Based Cartesian Control for xArm.

    Usage:
        # Terminal 1: Start the controller (this script)
        python3 example_cartesian_control.py

        # Terminal 2: Publish target poses
        python3 target_setter.py --world 0.4 0.0 0.5  # Absolute world coordinates
        python3 target_setter.py --relative 0.05 0 0  # Relative movement (50mm in X)

    The controller subscribes to /target_pose topic and automatically moves
    the robot to received target poses.

    Requirements:
        - xArm robot connected at 192.168.2.235
        - Robot will be automatically enabled in servo mode
        - Proper network configuration
    """
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback

        traceback.print_exc()

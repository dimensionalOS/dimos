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
Example: Joint Trajectory Control with xArm

Demonstrates joint-space trajectory execution. The controller
executes trajectories by sampling at 100Hz and sending joint commands.

This example shows:
1. Deploy xArm driver with LCM transports
2. Deploy JointTrajectoryController with LCM transports
3. Execute trajectories via RPC or topic
4. Monitor execution status

Use trajectory_setter.py to interactively create and execute trajectories.
"""

import signal
import time

from dimos import core
from dimos.hardware.manipulators.xarm import XArmDriver
from dimos.manipulation.control import JointTrajectoryController
from dimos.msgs.sensor_msgs import JointCommand, JointState, RobotState
from dimos.msgs.trajectory_msgs import JointTrajectory, TrajectoryState

# Global flag for graceful shutdown
shutdown_requested = False


def signal_handler(sig, frame):
    """Handle Ctrl+C for graceful shutdown."""
    global shutdown_requested
    print("\n\nShutdown requested...")
    shutdown_requested = True


def main():
    """
    Deploy and run joint trajectory control system.

    The system executes joint trajectories at 100Hz by sampling
    and forwarding joint positions to the arm driver.
    """

    # Register signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # =========================================================================
    # Step 1: Start dimos cluster
    # =========================================================================
    print("=" * 80)
    print("Joint Trajectory Control")
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

        print("Starting xArm driver...")
        arm_driver.start()

        # =========================================================================
        # Step 3: Deploy Joint Trajectory Controller
        # =========================================================================
        print("\nDeploying Joint Trajectory Controller...")
        controller = dimos.deploy(
            JointTrajectoryController,
            control_frequency=100.0,  # 100Hz execution
        )

        # Set up controller transports
        controller.joint_state.transport = core.LCMTransport("/xarm/joint_states", JointState)
        controller.robot_state.transport = core.LCMTransport("/xarm/robot_state", RobotState)
        controller.joint_position_command.transport = core.LCMTransport(
            "/xarm/joint_position_command", JointCommand
        )

        # Subscribe to trajectory topic (from trajectory_setter.py)
        controller.trajectory.transport = core.LCMTransport("/trajectory", JointTrajectory)

        print("Starting controller...")
        controller.start()

        # Wait for joint state
        print("\nWaiting for joint state...")
        time.sleep(1.0)

        # =========================================================================
        # Step 4: Keep system running
        # =========================================================================
        print("\n" + "=" * 80)
        print("System ready!")
        print("=" * 80)
        print("\nJoint Trajectory Controller is running at 100Hz")
        print("Listening on /trajectory topic")
        print("\nUse trajectory_setter.py in another terminal to publish trajectories")
        print("\nPress Ctrl+C to shutdown")
        print("=" * 80 + "\n")

        # Keep running until shutdown requested
        while not shutdown_requested:
            # Print status periodically
            status = controller.get_status()
            if status.state == TrajectoryState.EXECUTING:
                print(
                    f"\rExecuting: {status.progress:.1%} | "
                    f"elapsed={status.time_elapsed:.2f}s | "
                    f"remaining={status.time_remaining:.2f}s",
                    end="",
                )
            time.sleep(0.5)

        # =========================================================================
        # Step 5: Clean shutdown
        # =========================================================================
        print("\n\nShutting down...")
        print("Stopping controller...")
        controller.stop()
        print("Stopping driver...")
        arm_driver.stop()
        print("Shutdown complete")

    finally:
        # Always stop dimos cluster
        print("Stopping dimos cluster...")
        dimos.stop()


if __name__ == "__main__":
    """
    Joint Trajectory Control for xArm.

    Usage:
        # Terminal 1: Start the controller (this script)
        python3 example_trajectory_control.py

        # Terminal 2: Create and execute trajectories
        python3 trajectory_setter.py

    The controller executes joint trajectories at 100Hz by sampling
    and forwarding joint positions to the arm driver.

    Requirements:
        - xArm robot connected at 192.168.1.210
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

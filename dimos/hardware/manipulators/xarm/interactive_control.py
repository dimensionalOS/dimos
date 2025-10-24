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
Interactive Terminal UI for xArm Control.

Provides a menu-driven interface to:
- Select which joint to move
- Specify movement delta in degrees
- Execute smooth trajectories
- View current joint positions

Usage:
    export XARM_IP=192.168.1.235
    venv/bin/python dimos/hardware/manipulators/xarm/interactive_control.py
"""

import os
import time
import math

from dimos import core
from dimos.hardware.manipulators.xarm.xarm_driver import XArmDriver
from dimos.hardware.manipulators.xarm.sample_trajectory_generator import (
    SampleTrajectoryGenerator,
)
from dimos.msgs.sensor_msgs import JointState, RobotState, JointCommand
from dimos.utils.logging_config import setup_logger

logger = setup_logger(__file__)


def print_banner():
    """Print welcome banner."""
    print("\n" + "=" * 80)
    print("  xArm Interactive Control")
    print("  Real-time joint control via terminal UI")
    print("=" * 80)


def print_current_state(traj_gen):
    """Display current joint positions."""
    state = traj_gen.get_current_state()

    print("\n" + "-" * 80)
    print("CURRENT JOINT POSITIONS:")
    print("-" * 80)

    if state["joint_state"]:
        js = state["joint_state"]
        for i in range(len(js.position)):
            pos_deg = math.degrees(js.position[i])
            vel_deg = math.degrees(js.velocity[i])
            print(f"  Joint {i + 1}: {pos_deg:8.2f}° (velocity: {vel_deg:6.2f}°/s)")
    else:
        print("  ⚠ No joint state available yet")

    if state["robot_state"]:
        rs = state["robot_state"]
        print(
            f"\n  Robot Status: state={rs.state} (0=ready), mode={rs.mode} (1=servo), error={rs.error_code}"
        )

    print("-" * 80)


def get_control_mode():
    """Get control mode selection from user."""
    while True:
        try:
            print("\nSelect control mode:")
            print("  1. Position control (move by angle)")
            print("  2. Velocity control (move with velocity)")
            choice = input("Mode (1 or 2): ").strip()

            if choice == "1":
                print(choice)
                return "position"
            elif choice == "2":
                print(choice)
                return "velocity"
            else:
                print("⚠ Invalid choice. Please enter 1 or 2.")

        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
            return None


def get_joint_selection(num_joints):
    """Get joint selection from user."""
    while True:
        try:
            print(f"\nSelect joint to move (1-{num_joints}), or 0 to quit:")
            choice = input("Joint number: ").strip()

            if not choice:
                continue

            joint_num = int(choice)

            if joint_num == 0:
                return None

            if 1 <= joint_num <= num_joints:
                return joint_num - 1  # Convert to 0-indexed
            else:
                print(f"⚠ Invalid joint number. Please enter 1-{num_joints} (or 0 to quit)")

        except ValueError:
            print("⚠ Invalid input. Please enter a number.")
        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
            return None


def get_delta_degrees():
    """Get movement delta from user."""
    while True:
        try:
            print("\nEnter movement delta in degrees:")
            print("  Positive = counterclockwise")
            print("  Negative = clockwise")
            delta_str = input("Delta (degrees): ").strip()

            if not delta_str:
                continue

            delta = float(delta_str)

            # Sanity check
            if abs(delta) > 180:
                confirm = input(f"⚠ Large movement ({delta}°). Continue? (y/n): ").strip().lower()
                if confirm != "y":
                    continue

            return delta

        except ValueError:
            print("⚠ Invalid input. Please enter a number.")
        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
            return None


def get_velocity_deg_s():
    """Get velocity from user."""
    while True:
        try:
            print("\nEnter target velocity in degrees/second:")
            print("  Positive = counterclockwise")
            print("  Negative = clockwise")
            vel_str = input("Velocity (°/s): ").strip()

            if not vel_str:
                continue

            velocity = float(vel_str)

            # Sanity check
            if abs(velocity) > 100:
                confirm = (
                    input(f"⚠ High velocity ({velocity}°/s). Continue? (y/n): ").strip().lower()
                )
                if confirm != "y":
                    continue

            return velocity

        except ValueError:
            print("⚠ Invalid input. Please enter a number.")
        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
            return None


def get_duration():
    """Get movement duration from user."""
    while True:
        try:
            print("\nEnter movement duration in seconds (default: 1.0):")
            duration_str = input("Duration (s): ").strip()

            if not duration_str:
                return 1.0  # Default

            duration = float(duration_str)

            if duration <= 0:
                print("⚠ Duration must be positive")
                continue

            if duration > 10:
                confirm = input(f"⚠ Long duration ({duration}s). Continue? (y/n): ").strip().lower()
                if confirm != "y":
                    continue

            return duration

        except ValueError:
            print("⚠ Invalid input. Please enter a number.")
        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
            return None


def confirm_motion_position(joint_index, delta_degrees, duration):
    """Confirm position motion with user."""
    print("\n" + "=" * 80)
    print("POSITION MOTION SUMMARY:")
    print(f"  Joint: {joint_index + 1}")
    print(
        f"  Delta: {delta_degrees:+.2f}° ({'clockwise' if delta_degrees < 0 else 'counterclockwise'})"
    )
    print(f"  Duration: {duration:.2f}s")
    print("=" * 80)

    confirm = input("\nExecute this motion? (y/n): ").strip().lower()
    return confirm == "y"


def confirm_motion_velocity(joint_index, velocity_deg_s, duration):
    """Confirm velocity motion with user."""
    print("\n" + "=" * 80)
    print("VELOCITY MOTION SUMMARY:")
    print(f"  Joint: {joint_index + 1}")
    print(
        f"  Velocity: {velocity_deg_s:+.2f}°/s ({'clockwise' if velocity_deg_s < 0 else 'counterclockwise'})"
    )
    print(f"  Duration: {duration:.2f}s (with ramp up/down)")
    print(f"  Profile: 20% ramp up, 60% constant, 20% ramp down")
    print("=" * 80)

    confirm = input("\nExecute this motion? (y/n): ").strip().lower()
    return confirm == "y"


def wait_for_trajectory_completion(traj_gen, duration):
    """Wait for trajectory to complete and show progress."""
    print("\n⚙ Executing motion...")

    # Wait with progress updates
    steps = 10
    step_duration = duration / steps

    for i in range(steps):
        time.sleep(step_duration)
        progress = ((i + 1) / steps) * 100
        print(f"  Progress: {progress:.0f}%")

    # Extra time for settling
    time.sleep(0.5)

    # Check if completed
    state = traj_gen.get_current_state()
    if state["trajectory_active"]:
        print("⚠ Trajectory still active, waiting...")
        time.sleep(duration * 0.5)

    print("✓ Motion complete!")


def interactive_control_loop(xarm, traj_gen, num_joints):
    """Main interactive control loop."""
    print_banner()

    # Wait for initial state
    print("\nInitializing... waiting for robot state...")
    time.sleep(2.0)

    # Enable servo mode and set state to ready
    state = traj_gen.get_current_state()
    if state["robot_state"]:
        if state["robot_state"].mode != 1:
            print("\n⚙ Enabling servo mode...")
            code, msg = xarm.enable_servo_mode()
            if code == 0:
                print(f"✓ {msg}")
                time.sleep(0.5)

        if state["robot_state"].state != 0:
            print("⚙ Setting robot to ready state...")
            code, msg = xarm.set_state(0)
            if code == 0:
                print(f"✓ {msg}")
                time.sleep(0.5)

    # Enable command publishing
    print("\n⚙ Enabling command publishing...")
    traj_gen.enable_publishing()
    time.sleep(1.0)
    print("✓ System ready for motion control")

    # Track current control mode (starts in position mode)
    current_mode = "position"

    # Main control loop
    while True:
        try:
            # Display current state
            print_current_state(traj_gen)

            # Show current mode
            mode_indicator = "POSITION" if current_mode == "position" else "VELOCITY"
            print(f"\n[Current Mode: {mode_indicator}]")

            # Get control mode selection
            control_mode = get_control_mode()
            if control_mode is None:
                break

            # Switch modes if user selected a different mode
            if control_mode != current_mode:
                print(f"\n⚙ Switching from {current_mode} mode to {control_mode} mode...")

                if control_mode == "velocity":
                    # Switch to velocity control mode (mode 4)
                    code, msg = xarm.enable_velocity_control_mode()
                    print(f"  {msg} (code: {code})")
                    if code == 0:
                        current_mode = "velocity"
                        # Notify trajectory generator about mode change
                        traj_gen.set_velocity_mode(True)
                        time.sleep(0.3)
                    else:
                        print(f"⚠ Failed to enable velocity mode, staying in {current_mode} mode")
                        continue
                else:
                    # Switch to position control mode (mode 1)
                    code, msg = xarm.disable_velocity_control_mode()
                    print(f"  {msg} (code: {code})")
                    if code == 0:
                        current_mode = "position"
                        # Notify trajectory generator about mode change
                        traj_gen.set_velocity_mode(False)
                        time.sleep(0.3)
                    else:
                        print(f"⚠ Failed to enable position mode, staying in {current_mode} mode")
                        continue

            # Get joint selection
            joint_index = get_joint_selection(num_joints)
            if joint_index is None:
                break

            # Get parameters based on mode
            if control_mode == "position":
                # Position control: get delta
                delta_degrees = get_delta_degrees()
                if delta_degrees is None:
                    break

                duration = get_duration()
                if duration is None:
                    break

                # Confirm motion
                if not confirm_motion_position(joint_index, delta_degrees, duration):
                    print("⚠ Motion cancelled")
                    continue

                # Execute position motion
                result = traj_gen.move_joint(
                    joint_index=joint_index, delta_degrees=delta_degrees, duration=duration
                )
                print(f"\n✓ {result}")

                # Wait for completion
                wait_for_trajectory_completion(traj_gen, duration)

            else:  # velocity mode
                # Velocity control: get velocity
                velocity_deg_s = get_velocity_deg_s()
                if velocity_deg_s is None:
                    break

                duration = get_duration()
                if duration is None:
                    break

                # Confirm motion
                if not confirm_motion_velocity(joint_index, velocity_deg_s, duration):
                    print("⚠ Motion cancelled")
                    continue

                # Execute velocity motion (mode already set above if needed)
                result = traj_gen.move_joint_velocity(
                    joint_index=joint_index, velocity_deg_s=velocity_deg_s, duration=duration
                )
                print(f"\n✓ {result}")

                # Wait for completion
                wait_for_trajectory_completion(traj_gen, duration)

            # Ask to continue
            print("\n" + "=" * 80)
            continue_choice = input("\nContinue with another motion? (y/n): ").strip().lower()
            if continue_choice != "y":
                break

        except KeyboardInterrupt:
            print("\n\n⚠ Interrupted by user")
            break
        except Exception as e:
            print(f"\n⚠ Error: {e}")
            continue_choice = input("\nContinue despite error? (y/n): ").strip().lower()
            if continue_choice != "y":
                break

    print("\n" + "=" * 80)
    print("Shutting down...")
    print("=" * 80)


def main():
    """Run interactive xArm control."""
    # Get IP address
    ip_address = os.getenv("XARM_IP", "192.168.1.235")
    num_joints = 6

    # Start dimos cluster
    logger.info("Starting dimos cluster...")
    cluster = core.start(1)

    # Deploy xArm driver
    logger.info("Deploying XArmDriver...")
    xarm = cluster.deploy(
        XArmDriver,
        ip_address=ip_address,
        report_type="dev",
        enable_on_start=True,
        num_joints=num_joints,
    )

    # Set up driver transports
    xarm.joint_state.transport = core.LCMTransport("/xarm/joint_states", JointState)
    xarm.robot_state.transport = core.LCMTransport("/xarm/robot_state", RobotState)
    xarm.joint_position_command.transport = core.LCMTransport(
        "/xarm/joint_position_command", JointCommand
    )
    xarm.joint_velocity_command.transport = core.LCMTransport(
        "/xarm/joint_velocity_command", JointCommand
    )

    # Start driver
    logger.info("Starting xArm driver...")
    xarm.start()

    # Deploy trajectory generator
    logger.info("Deploying SampleTrajectoryGenerator...")
    traj_gen = cluster.deploy(
        SampleTrajectoryGenerator,
        num_joints=num_joints,
        control_mode="position",
        publish_rate=100.0,  # 100 Hz
        enable_on_start=False,
    )

    # Set up trajectory generator transports
    traj_gen.joint_state_input.transport = core.LCMTransport("/xarm/joint_states", JointState)
    traj_gen.robot_state_input.transport = core.LCMTransport("/xarm/robot_state", RobotState)
    traj_gen.joint_position_command.transport = core.LCMTransport(
        "/xarm/joint_position_command", JointCommand
    )
    traj_gen.joint_velocity_command.transport = core.LCMTransport(
        "/xarm/joint_velocity_command", JointCommand
    )

    # Start trajectory generator
    logger.info("Starting trajectory generator...")
    traj_gen.start()

    try:
        # Run interactive control loop
        interactive_control_loop(xarm, traj_gen, num_joints)

    except KeyboardInterrupt:
        print("\n\n⚠ Interrupted by user")

    finally:
        # Cleanup
        print("\nStopping trajectory generator...")
        traj_gen.stop()
        print("Stopping xArm driver...")
        xarm.stop()
        print("Stopping cluster...")
        cluster.stop()
        print("✓ Shutdown complete\n")


if __name__ == "__main__":
    main()

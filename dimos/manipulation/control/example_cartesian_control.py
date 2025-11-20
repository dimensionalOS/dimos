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
Example: Using CartesianMotionController with xArm

Demonstrates Cartesian space motion control using the exact same pattern as
interactive_control.py and sample_trajectory_generator.py.

This example shows:
1. Deploy xArm driver with LCM transports
2. Deploy CartesianMotionController with LCM transports
3. Send Cartesian motion commands via RPC
4. Monitor convergence

Pattern matches: interactive_control.py + sample_trajectory_generator.py
"""

import math
import time

from dimos import core
from dimos.hardware.manipulators.xarm import XArmDriver
from dimos.manipulation.control import CartesianMotionController
from dimos.msgs.sensor_msgs import JointState, JointCommand, RobotState
from dimos.msgs.geometry_msgs import PoseStamped


def main():
    """
    Example: Move xArm to target Cartesian poses using the controller.
    """

    # =========================================================================
    # Step 1: Start dimos cluster
    # =========================================================================
    print("Starting dimos cluster...")
    dimos = core.start(1)  # Start with 1 worker

    try:
        # =========================================================================
        # Step 2: Deploy xArm driver (EXACTLY like interactive_control.py)
        # =========================================================================
        print("\nDeploying xArm driver...")
        arm_driver = dimos.deploy(
            XArmDriver,
            ip_address="192.168.2.235",
            xarm_type="xarm6",
            report_type="dev",
            enable_on_start=True,  # Auto-enable like interactive_control.py
        )

        # Set up driver transports (EXACTLY like interactive_control.py lines 478-485)
        arm_driver.joint_state.transport = core.LCMTransport("/xarm/joint_states", JointState)
        arm_driver.robot_state.transport = core.LCMTransport("/xarm/robot_state", RobotState)
        arm_driver.joint_position_command.transport = core.LCMTransport(
            "/xarm/joint_position_command", JointCommand
        )
        arm_driver.joint_velocity_command.transport = core.LCMTransport(
            "/xarm/joint_velocity_command", JointCommand
        )

        # Start driver (EXACTLY like interactive_control.py line 489)
        print("Starting xArm driver...")
        arm_driver.start()

        # =========================================================================
        # Step 3: Deploy controller (EXACTLY like interactive_control.py lines 492-513)
        # =========================================================================
        print("Deploying Cartesian motion controller...")
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

        # Set up controller transports (EXACTLY like interactive_control.py lines 502-509)
        controller.joint_state.transport = core.LCMTransport("/xarm/joint_states", JointState)
        controller.robot_state.transport = core.LCMTransport("/xarm/robot_state", RobotState)
        controller.joint_position_command.transport = core.LCMTransport(
            "/xarm/joint_position_command", JointCommand
        )

        # Start controller (EXACTLY like interactive_control.py line 513)
        print("Starting controller...")
        controller.start()

        # =========================================================================
        # Step 4: Run interactive control loop
        # =========================================================================
        interactive_control_loop(controller)

        # =========================================================================
        # Step 5: Clean shutdown
        # =========================================================================
        print("\nStopping controller...")
        controller.stop()
        print("Stopping driver...")
        arm_driver.stop()
        print("✓ Shutdown complete")

    finally:
        # Always stop dimos cluster
        print("Stopping dimos cluster...")
        dimos.stop()


def print_banner() -> None:
    """Print welcome banner."""
    print("\n" + "=" * 80)
    print("  xArm Cartesian Control - Interactive Mode")
    print("  Control the TCP position with relative X/Y/Z movements")
    print("=" * 80)


def print_current_pose(controller: CartesianMotionController) -> None:
    """Display current TCP pose."""
    current_pose = controller.get_current_pose()

    print("\n" + "-" * 80)
    print("CURRENT TCP POSE:")
    print("-" * 80)

    if current_pose:
        print(f"  Position (m):")
        print(f"    X: {current_pose.x:8.4f} m  ({current_pose.x * 1000:7.1f} mm)")
        print(f"    Y: {current_pose.y:8.4f} m  ({current_pose.y * 1000:7.1f} mm)")
        print(f"    Z: {current_pose.z:8.4f} m  ({current_pose.z * 1000:7.1f} mm)")
        print(f"  Orientation (rad):")
        print(f"    Roll:  {current_pose.roll:7.4f} rad  ({math.degrees(current_pose.roll):7.2f}°)")
        print(f"    Pitch: {current_pose.pitch:7.4f} rad  ({math.degrees(current_pose.pitch):7.2f}°)")
        print(f"    Yaw:   {current_pose.yaw:7.4f} rad  ({math.degrees(current_pose.yaw):7.2f}°)")
    else:
        print("  ⚠ No pose available yet")

    print("-" * 80)


def get_relative_movement():
    """Get relative movement in x, y, z from user (in mm)."""
    print("\nEnter relative movement (in millimeters):")
    print("  Positive X = forward, Y = left, Z = up")
    print("  Leave blank to skip axis")

    try:
        dx_str = input("  ΔX (mm): ").strip()
        dy_str = input("  ΔY (mm): ").strip()
        dz_str = input("  ΔZ (mm): ").strip()

        dx = float(dx_str) / 1000.0 if dx_str else 0.0  # Convert mm to m
        dy = float(dy_str) / 1000.0 if dy_str else 0.0
        dz = float(dz_str) / 1000.0 if dz_str else 0.0

        # Sanity check
        total_dist = math.sqrt(dx**2 + dy**2 + dz**2) * 1000  # in mm
        if total_dist > 200:
            confirm = input(f"⚠ Large movement ({total_dist:.1f}mm). Continue? (y/n): ").strip().lower()
            if confirm != "y":
                return None

        return dx, dy, dz

    except ValueError:
        print("⚠ Invalid input. Please enter numbers.")
        return None
    except KeyboardInterrupt:
        print("\n\n⚠ Interrupted by user")
        return None


def get_rotation_change():
    """Get rotation change in roll, pitch, yaw from user (in degrees)."""
    print("\nEnter rotation change (in degrees, leave blank to skip):")

    try:
        droll_str = input("  ΔRoll (°): ").strip()
        dpitch_str = input("  ΔPitch (°): ").strip()
        dyaw_str = input("  ΔYaw (°): ").strip()

        droll = math.radians(float(droll_str)) if droll_str else 0.0
        dpitch = math.radians(float(dpitch_str)) if dpitch_str else 0.0
        dyaw = math.radians(float(dyaw_str)) if dyaw_str else 0.0

        return droll, dpitch, dyaw

    except ValueError:
        print("⚠ Invalid input. Please enter numbers.")
        return None
    except KeyboardInterrupt:
        print("\n\n⚠ Interrupted by user")
        return None


def confirm_motion(dx, dy, dz, droll, dpitch, dyaw):
    """Confirm motion with user."""
    print("\n" + "=" * 80)
    print("MOTION SUMMARY:")
    print(f"  Position delta: X={dx*1000:+.1f}mm, Y={dy*1000:+.1f}mm, Z={dz*1000:+.1f}mm")
    print(f"  Rotation delta: Roll={math.degrees(droll):+.1f}°, Pitch={math.degrees(dpitch):+.1f}°, Yaw={math.degrees(dyaw):+.1f}°")
    total_dist = math.sqrt(dx**2 + dy**2 + dz**2) * 1000
    print(f"  Total linear distance: {total_dist:.1f}mm")
    print("=" * 80)

    confirm = input("\nExecute this motion? (y/n): ").strip().lower()
    return confirm == "y"


def interactive_control_loop(controller: CartesianMotionController) -> None:
    """Main interactive control loop."""
    print_banner()

    # Wait for initial state
    print("\nInitializing... waiting for robot state...")
    time.sleep(2.0)

    print("✓ System ready for Cartesian control")

    # Main control loop
    while True:
        try:
            # Display current pose
            print_current_pose(controller)

            # Get current pose for reference
            current_pose = controller.get_current_pose()
            if not current_pose:
                print("⚠ Cannot get current pose. Waiting...")
                time.sleep(1.0)
                continue

            # Ask user for movement type
            print("\nWhat would you like to do?")
            print("  1. Move position (X/Y/Z)")
            print("  2. Move position and rotation")
            print("  0. Quit")

            choice = input("Choice: ").strip()

            if choice == "0":
                break
            elif choice not in ["1", "2"]:
                print("⚠ Invalid choice. Please enter 1, 2, or 0.")
                continue

            # Get relative movement
            movement = get_relative_movement()
            if movement is None:
                continue
            dx, dy, dz = movement

            # Get rotation change if requested
            if choice == "2":
                rotation = get_rotation_change()
                if rotation is None:
                    continue
                droll, dpitch, dyaw = rotation
            else:
                droll, dpitch, dyaw = 0.0, 0.0, 0.0

            # Skip if no movement
            if dx == 0 and dy == 0 and dz == 0 and droll == 0 and dpitch == 0 and dyaw == 0:
                print("⚠ No movement specified")
                continue

            # Confirm motion
            if not confirm_motion(dx, dy, dz, droll, dpitch, dyaw):
                print("⚠ Motion cancelled")
                continue

            # Calculate target pose
            target_position = [
                current_pose.x + dx,
                current_pose.y + dy,
                current_pose.z + dz,
            ]
            target_orientation = [
                current_pose.roll + droll,
                current_pose.pitch + dpitch,
                current_pose.yaw + dyaw,
            ]

            # Execute motion
            print("\n⚙ Sending motion command...")
            controller.set_target_pose(
                position=target_position,
                orientation=target_orientation,
                frame_id="world",
            )

            # Wait for convergence
            print("⚙ Waiting for convergence...")
            converged = wait_for_convergence(controller, timeout=15.0)

            if not converged:
                print("⚠ Motion did not converge within timeout")

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
            import traceback
            traceback.print_exc()
            continue_choice = input("\nContinue despite error? (y/n): ").strip().lower()
            if continue_choice != "y":
                break

    print("\n" + "=" * 80)
    print("Shutting down...")
    print("=" * 80)


def wait_for_convergence(controller: CartesianMotionController, timeout: float = 10.0) -> bool:
    """
    Wait for the controller to converge to the target.

    Args:
        controller: CartesianMotionController instance
        timeout: Maximum time to wait (seconds)

    Returns:
        True if converged, False if timeout
    """
    start_time = time.time()

    while time.time() - start_time < timeout:
        if controller.is_converged():
            elapsed = time.time() - start_time
            print(f"    ✓ Converged in {elapsed:.2f}s")
            return True
        time.sleep(0.1)

    print(f"    ✗ Timeout after {timeout:.1f}s (did not converge)")
    return False


def example_using_posestamped_messages(controller):
    """
    Alternative example: Using PoseStamped messages directly.

    This shows how higher-level planners can publish PoseStamped messages
    that the controller subscribes to automatically.

    Args:
        controller: CartesianMotionController instance
    """
    # Create a PoseStamped message
    target = PoseStamped(
        ts=time.time(),
        frame_id="world",
        position=[0.3, 0.2, 0.5],  # xyz position
        orientation=[0, 0, 0, 1],  # quaternion (identity = no rotation)
    )

    # Option 1: Set via RPC
    controller.set_target_pose(position=list(target.position), orientation=list(target.orientation))

    # Option 2: Publish directly to target_pose topic (if connected)
    # controller.target_pose.publish(target)


if __name__ == "__main__":
    """
    Interactive Cartesian Control for xArm.

    Usage:
        python3 example_cartesian_control.py

    The script will start an interactive terminal UI where you can:
    - View current TCP position and orientation
    - Specify relative movements in X/Y/Z (in millimeters)
    - Optionally specify rotations in roll/pitch/yaw (in degrees)
    - Execute motions and monitor convergence

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

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
Interactive Terminal UI for Piper Control.

Provides a menu-driven interface to:
- Select which joint to move
- Specify target position in degrees
- Execute smooth movements
- View current joint positions
- Control robot via position or velocity commands
- Specify velocity duration (how long to send velocity commands)

Usage:
    # Position control mode (default):
    python3 dimos/hardware/manipulators/piper/interactive_control.py

    # Velocity control mode (with duration support):
    python3 dimos/hardware/manipulators/piper/interactive_control.py --mode velocity

    # Or specify CAN interface:
    python3 dimos/hardware/manipulators/piper/interactive_control.py --can can0 --mode position

Velocity Mode Features:
    - Enter velocities for all 6 joints (deg/s)
    - Specify duration (seconds) to continuously send velocity commands
    - Commands are sent at 20 Hz to prevent timeout (0.1s safety timeout)
    - Robot automatically stops after duration expires
    - Press Ctrl+C to stop early
"""

import math
import time

from dimos import core
from dimos.hardware.manipulators.piper.piper_driver import PiperDriver
from dimos.hardware.manipulators.piper.sample_trajectory_generator import SampleTrajectoryGenerator
from dimos.msgs.sensor_msgs import JointCommand, JointState, RobotState
from dimos.utils.logging_config import setup_logger

logger = setup_logger(__file__)


def print_banner(velocity_mode: bool = False) -> None:
    """Print welcome banner."""
    print("\n" + "=" * 80)
    print("  Piper Interactive Control")
    mode_str = "VELOCITY CONTROL" if velocity_mode else "POSITION CONTROL"
    print(f"  Mode: {mode_str}")
    print("  Real-time joint control via terminal UI")
    print("=" * 80)


def print_current_state(traj_gen) -> None:
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
        print(f"\n  Robot Status: state={rs.state}, mode={rs.mode}, error={rs.error_code}")

    print("-" * 80)


def get_joint_selection(num_joints: int):
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


def get_velocity_input() -> tuple[list[float], float] | None:
    """Get velocity input and duration for all joints.

    Returns:
        Tuple of (velocities, duration) or None if quit
    """
    while True:
        try:
            print("\n" + "=" * 80)
            print("VELOCITY CONTROL INPUT")
            print("=" * 80)
            print("Enter joint velocities in deg/s (separated by spaces)")
            print("Format: j1 j2 j3 j4 j5 j6")
            print("Example: 10 0 0 0 0 0  (rotate joint 1 at 10 deg/s)")
            print("Press Enter for all zeros (stop), or 'q' to quit")
            print("=" * 80)

            vel_input = input("\nVelocities (deg/s): ").strip()

            if not vel_input:
                # Default: all zeros, instant stop
                return ([0.0] * 6, 0.0)

            if vel_input.lower() == "q":
                return None

            # Parse velocities
            vel_str_list = vel_input.split()

            if len(vel_str_list) != 6:
                print(f"⚠ Expected 6 velocities, got {len(vel_str_list)}. Please try again.")
                continue

            # Convert to radians/s
            velocities_deg = [float(v) for v in vel_str_list]
            velocities_rad = [math.radians(v) for v in velocities_deg]

            # Display velocities
            print("\n→ Velocities (deg/s):", [f"{v:+.2f}" for v in velocities_deg])
            print("→ Velocities (rad/s):", [f"{v:+.4f}" for v in velocities_rad])

            # Check if all zeros (instant stop)
            if all(v == 0.0 for v in velocities_rad):
                confirm = input("\nSend zero velocities (stop)? (y/n): ").strip().lower()
                if confirm == "y":
                    return (velocities_rad, 0.0)
                else:
                    print("⚠ Cancelled")
                    continue

            # Get duration
            while True:
                try:
                    duration_input = input("\nDuration (seconds, default=1.0): ").strip()

                    if not duration_input:
                        duration = 1.0
                        break

                    duration = float(duration_input)

                    if duration <= 0:
                        print("⚠ Duration must be positive")
                        continue

                    if duration > 30:
                        confirm = (
                            input(f"⚠ Long duration ({duration}s). Continue? (y/n): ")
                            .strip()
                            .lower()
                        )
                        if confirm != "y":
                            continue

                    break

                except ValueError:
                    print("⚠ Invalid duration. Please enter a number.")

            # Final confirmation
            print(f"\n→ Will send velocities for {duration} seconds")
            confirm = input("Confirm? (y/n): ").strip().lower()
            if confirm == "y":
                return (velocities_rad, duration)
            else:
                print("⚠ Cancelled")
                continue

        except ValueError as e:
            print(f"⚠ Invalid input: {e}. Please enter numbers only.")
        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
            return None


def confirm_motion(joint_index, delta_degrees, duration):
    """Confirm motion with user."""
    print("\n" + "=" * 80)
    print("MOTION SUMMARY:")
    print(f"  Joint: {joint_index + 1}")
    print(
        f"  Delta: {delta_degrees:+.2f}° ({'clockwise' if delta_degrees < 0 else 'counterclockwise'})"
    )
    print(f"  Duration: {duration:.2f}s")
    print("=" * 80)

    confirm = input("\nExecute this motion? (y/n): ").strip().lower()
    return confirm == "y"


def wait_for_trajectory_completion(traj_gen, duration) -> None:
    """Wait for trajectory to complete and show progress."""
    print("\n→ Executing motion...")

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


def velocity_control_loop(piper, traj_gen, num_joints: int) -> None:
    """Velocity control loop - publishes to joint_velocity_command topic."""
    print_banner(velocity_mode=True)

    # Wait for initial state
    print("\nInitializing... waiting for robot state...")
    time.sleep(2.0)

    # Enable servo mode (RPC call is OK for configuration)
    print("\n→ Enabling servo mode...")
    result = piper.enable_servo_mode()
    if result:
        print("  ✓ Servo mode enabled")
    else:
        print("  ⚠ Servo mode may already be enabled")
    time.sleep(0.5)

    # Enable velocity control mode (RPC call is OK for configuration)
    print("\n→ Enabling velocity control mode (integration-based)...")
    success, msg = piper.enable_velocity_control_mode()
    if not success:
        print(f"  ✗ Failed: {msg}")
        return
    print(f"  ✓ {msg}")
    time.sleep(0.5)

    print("\n✓ System ready for velocity control")
    print("  (Velocities published to topic → updates _vel_cmd_ → control loop integrates)")
    print("\n⚠ SAFETY:")
    print("  - Velocities will be published continuously at 20 Hz for the specified duration")
    print("  - Robot will automatically stop after duration expires")
    print("  - Press Ctrl+C to stop early, or send zero velocities to stop immediately")

    # Velocity control loop
    while True:
        try:
            # Display current state
            print_current_state(traj_gen)

            # Get velocity input and duration
            result = get_velocity_input()
            if result is None:
                break

            velocities, duration = result

            # Check if stopping (all zeros)
            if all(v == 0.0 for v in velocities):
                # Publish zero velocity to topic
                vel_cmd = JointCommand(positions=velocities)
                piper.joint_velocity_command.publish(vel_cmd)
                print("\n✓ Robot stopped (zero velocity published)")
                continue

            # Send velocity commands continuously for the specified duration
            print(f"\n→ Publishing velocities for {duration} seconds...")
            print("  (Press Ctrl+C to stop early)")

            start_time = time.time()
            send_rate = 20  # Hz (publish every 0.05s, well below 0.1s timeout)
            next_send_time = start_time

            try:
                while (time.time() - start_time) < duration:
                    # Publish velocity command to topic (updates _vel_cmd_ in real-time)
                    vel_cmd = JointCommand(positions=velocities)
                    piper.joint_velocity_command.publish(vel_cmd)

                    # Calculate elapsed and remaining time
                    elapsed = time.time() - start_time
                    remaining = duration - elapsed

                    # Progress indicator
                    print(
                        f"\r  ⏱ Elapsed: {elapsed:.1f}s / {duration:.1f}s (remaining: {remaining:.1f}s)",
                        end="",
                        flush=True,
                    )

                    # Sleep until next send time
                    next_send_time += 1.0 / send_rate
                    sleep_time = next_send_time - time.time()
                    if sleep_time > 0:
                        time.sleep(sleep_time)

            except KeyboardInterrupt:
                print("\n\n⚠ Stopped early by user")

            # After duration expires or interruption, send zero velocities
            print("\n\n→ Duration complete - stopping robot...")
            zero_vel_cmd = JointCommand(positions=[0.0] * 6)
            piper.joint_velocity_command.publish(zero_vel_cmd)
            print("✓ Robot stopped (zero velocity published)")

        except KeyboardInterrupt:
            print("\n\n⚠ Interrupted by user - publishing zero velocities...")
            zero_vel_cmd = JointCommand(positions=[0.0] * 6)
            piper.joint_velocity_command.publish(zero_vel_cmd)
            break
        except Exception as e:
            print(f"\n⚠ Error: {e}")
            import traceback

            traceback.print_exc()
            # Safety: publish zero velocities on error
            zero_vel_cmd = JointCommand(positions=[0.0] * 6)
            piper.joint_velocity_command.publish(zero_vel_cmd)
            continue_choice = input("\nContinue despite error? (y/n): ").strip().lower()
            if continue_choice != "y":
                break

    # Disable velocity control mode
    print("\n→ Disabling velocity control mode...")
    success, msg = piper.disable_velocity_control_mode()
    print(f"  {msg}")

    print("\n" + "=" * 80)
    print("Velocity control loop ended")
    print("=" * 80)


def interactive_control_loop(piper, traj_gen, num_joints: int) -> None:
    """Main interactive control loop (position mode)."""
    print_banner(velocity_mode=False)

    # Wait for initial state
    print("\nInitializing... waiting for robot state...")
    time.sleep(2.0)

    # Enable servo mode if needed
    state = traj_gen.get_current_state()
    if state["robot_state"]:
        # Piper doesn't have explicit mode/state management like xArm
        # Just make sure servo is enabled
        print("\n→ Ensuring servo mode is enabled...")
        result = piper.enable_servo_mode()
        if result:
            print("✓ Servo mode enabled")
        else:
            print("⚠ Servo mode may already be enabled")
        time.sleep(0.5)

    # Enable command publishing
    print("\n→ Enabling command publishing...")
    traj_gen.enable_publishing()
    time.sleep(1.0)
    print("✓ System ready for motion control")

    # Main control loop
    while True:
        try:
            # Display current state
            print_current_state(traj_gen)

            # Get joint selection
            joint_index = get_joint_selection(num_joints)
            if joint_index is None:
                break

            # Get delta in degrees
            delta_degrees = get_delta_degrees()
            if delta_degrees is None:
                break

            # Get duration
            duration = get_duration()
            if duration is None:
                break

            # Confirm motion
            if not confirm_motion(joint_index, delta_degrees, duration):
                print("⚠ Motion cancelled")
                continue

            # Execute motion
            result = traj_gen.move_joint(
                joint_index=joint_index, delta_degrees=delta_degrees, duration=duration
            )
            print(f"\n→ {result}")

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
            import traceback

            traceback.print_exc()
            continue_choice = input("\nContinue despite error? (y/n): ").strip().lower()
            if continue_choice != "y":
                break

    print("\n" + "=" * 80)
    print("Shutting down...")
    print("=" * 80)


def main() -> None:
    """Run interactive Piper control."""
    import argparse

    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Interactive Piper Control")
    parser.add_argument(
        "--can",
        type=str,
        default="can0",
        help="CAN interface name (default: can0)",
    )
    parser.add_argument(
        "--mode",
        type=str,
        default="position",
        choices=["position", "velocity"],
        help="Control mode: position or velocity (default: position)",
    )
    args = parser.parse_args()

    can_name = args.can
    control_mode = args.mode
    num_joints = 6  # Piper is always 6 joints

    logger.info(f"Using Piper on CAN interface: {can_name}")
    logger.info(f"Control mode: {control_mode}")

    # Start dimos
    logger.info("Starting dimos...")
    dimos = core.start(1)

    # Deploy Piper driver
    logger.info("Deploying PiperDriver...")
    piper = dimos.deploy(
        PiperDriver,
        can_name=can_name,
        enable_on_start=True,
        control_frequency=100.0,
        joint_state_rate=100.0,
        robot_state_rate=10.0,
    )

    # Set up driver transports
    piper.joint_state.transport = core.LCMTransport("/piper/joint_states", JointState)
    piper.robot_state.transport = core.LCMTransport("/piper/robot_state", RobotState)
    piper.joint_position_command.transport = core.LCMTransport(
        "/piper/joint_position_command", JointCommand
    )
    piper.joint_velocity_command.transport = core.LCMTransport(
        "/piper/joint_velocity_command", JointCommand
    )

    # Start driver
    logger.info("Starting Piper driver...")
    piper.start()

    # Deploy trajectory generator
    logger.info("Deploying SampleTrajectoryGenerator...")
    traj_gen = dimos.deploy(
        SampleTrajectoryGenerator,
        num_joints=num_joints,
        control_mode="position",
        publish_rate=100.0,  # 100 Hz
        enable_on_start=False,
    )

    # Set up trajectory generator transports
    traj_gen.joint_state_input.transport = core.LCMTransport("/piper/joint_states", JointState)
    traj_gen.robot_state_input.transport = core.LCMTransport("/piper/robot_state", RobotState)
    traj_gen.joint_position_command.transport = core.LCMTransport(
        "/piper/joint_position_command", JointCommand
    )
    traj_gen.joint_velocity_command.transport = core.LCMTransport(
        "/piper/joint_velocity_command", JointCommand
    )

    # Start trajectory generator
    logger.info("Starting trajectory generator...")
    traj_gen.start()

    try:
        # Run interactive control loop based on mode
        if control_mode == "velocity":
            velocity_control_loop(piper, traj_gen, num_joints)
        else:
            interactive_control_loop(piper, traj_gen, num_joints)

    except KeyboardInterrupt:
        print("\n\n⚠ Interrupted by user")

    finally:
        # Cleanup
        print("\nStopping trajectory generator...")
        traj_gen.stop()
        print("Stopping Piper driver...")
        piper.stop()
        print("Stopping dimos...")
        dimos.stop()
        print("✓ Shutdown complete\n")


if __name__ == "__main__":
    main()

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
Interactive Trajectory Publisher for Joint Trajectory Control.

Interactive terminal UI for creating joint trajectories using the
JointTrajectoryGenerator with trapezoidal velocity profiles.

Workflow:
1. Add waypoints (joint positions only, no timing)
2. Generator applies trapezoidal velocity profile
3. Preview the generated trajectory
4. Publish to /trajectory topic

Use with example_trajectory_control.py running in another terminal.
"""

import math
import sys
import time

from dimos import core
from dimos.manipulation.planning import JointTrajectoryGenerator
from dimos.msgs.sensor_msgs import JointState
from dimos.msgs.trajectory_msgs import JointTrajectory


class TrajectorySetter:
    """
    Creates and publishes JointTrajectory using trapezoidal velocity profiles.

    Uses JointTrajectoryGenerator to compute proper timing and velocities
    from a list of waypoints. Subscribes to arm-specific joint_states to get
    current joint positions.

    Supports multiple arm types:
    - xarm (xarm5/6/7)
    - piper
    - Any future arm that publishes joint_states
    """

    def __init__(self, arm_type: str = "xarm"):
        """
        Initialize the trajectory setter.

        Args:
            arm_type: Type of arm ("xarm", "piper", etc.)
        """
        self.arm_type = arm_type.lower()

        # Publisher for trajectories
        self.trajectory_pub: core.LCMTransport[JointTrajectory] = core.LCMTransport(
            "/trajectory", JointTrajectory
        )

        # Subscribe to arm-specific joint state topic
        joint_state_topic = f"/{self.arm_type}/joint_states"
        self.joint_state_sub: core.LCMTransport[JointState] = core.LCMTransport(
            joint_state_topic, JointState
        )
        self.latest_joint_state: JointState | None = None

        # Will be set dynamically from joint_state
        self.num_joints: int | None = None
        self.generator: JointTrajectoryGenerator | None = None

        print(f"TrajectorySetter initialized for {self.arm_type.upper()}")
        print("  Publishing to: /trajectory")
        print(f"  Subscribing to: {joint_state_topic}")

    def start(self) -> bool:
        """Start subscribing to joint state."""
        self.joint_state_sub.subscribe(self._on_joint_state)
        print("  Waiting for joint state...")

        for _ in range(50):  # 5 second timeout
            if self.latest_joint_state is not None:
                # Dynamically determine joint count from actual joint_state
                self.num_joints = len(self.latest_joint_state.position)
                print(f"  ✓ Joint state received ({self.num_joints} joints)")

                # Now create generator with correct joint count
                self.generator = JointTrajectoryGenerator(
                    num_joints=self.num_joints,
                    max_velocity=1.0,  # rad/s
                    max_acceleration=2.0,  # rad/s^2
                    points_per_segment=50,
                )
                print(f"  Max velocity: {self.generator.max_velocity[0]:.2f} rad/s")
                print(f"  Max acceleration: {self.generator.max_acceleration[0]:.2f} rad/s^2")
                return True
            time.sleep(0.1)

        print("  ⚠ Warning: No joint state received (timeout)")
        return False

    def _on_joint_state(self, msg: JointState) -> None:
        """Callback for joint state updates."""
        self.latest_joint_state = msg

    def get_current_joints(self) -> list[float] | None:
        """Get current joint positions in radians (first num_joints only)."""
        if self.latest_joint_state is None:
            return None
        # Only take first num_joints (exclude gripper if present)
        return list(self.latest_joint_state.position[: self.num_joints])

    def generate_trajectory(self, waypoints: list[list[float]]) -> JointTrajectory:
        """
        Generate a trajectory from waypoints using trapezoidal velocity profile.

        Args:
            waypoints: List of joint positions [j1, j2, ..., j6] in radians

        Returns:
            JointTrajectory with proper timing and velocities
        """
        if self.generator is None:
            raise RuntimeError("Generator not initialized - joint state not received yet")
        return self.generator.generate(waypoints)

    def publish_trajectory(self, trajectory: JointTrajectory) -> None:
        """
        Publish a JointTrajectory to the /trajectory topic.

        Args:
            trajectory: Generated trajectory to publish
        """
        self.trajectory_pub.broadcast(None, trajectory)
        print(
            f"\nPublished trajectory: {len(trajectory.points)} points, "
            f"duration={trajectory.duration:.2f}s"
        )


def parse_joint_input(line: str, num_joints: int) -> list[float] | None:
    """
    Parse joint positions from user input.

    Accepts degrees by default, or radians with 'r' suffix.
    """
    parts = line.strip().split()
    if len(parts) != num_joints:
        return None

    positions = []
    for part in parts:
        try:
            if part.endswith("r"):
                positions.append(float(part[:-1]))
            else:
                positions.append(math.radians(float(part)))
        except ValueError:
            return None

    return positions


def preview_waypoints(waypoints: list[list[float]], num_joints: int) -> None:
    """Show waypoints list."""
    if not waypoints:
        print("No waypoints")
        return

    # Dynamically generate header based on joint count
    joint_headers = " ".join([f"{'J' + str(i + 1):>7}" for i in range(num_joints)])
    line_width = 6 + 3 + num_joints * 8 + 10

    print(f"\nWaypoints ({len(waypoints)}):")
    print("-" * line_width)
    print(f"  # | {joint_headers} (degrees)")
    print("-" * line_width)
    for i, joints in enumerate(waypoints):
        deg = [f"{math.degrees(j):7.1f}" for j in joints]
        print(f" {i + 1:2} | {' '.join(deg)}")
    print("-" * line_width)


def preview_trajectory(trajectory: JointTrajectory, num_joints: int) -> None:
    """Show generated trajectory preview."""
    # Dynamically generate header based on joint count
    joint_headers = " ".join([f"{'J' + str(i + 1):>7}" for i in range(num_joints)])
    line_width = 9 + 3 + num_joints * 8 + 10

    print("\n" + "=" * line_width)
    print("GENERATED TRAJECTORY")
    print("=" * line_width)
    print(f"Duration: {trajectory.duration:.3f}s")
    print(f"Points: {len(trajectory.points)}")
    print("-" * line_width)
    print(f"{'Time':>6} | {joint_headers} (degrees)")
    print("-" * line_width)

    # Sample at regular intervals
    num_samples = min(15, max(len(trajectory.points) // 10, 5))
    for i in range(num_samples + 1):
        t = (i / num_samples) * trajectory.duration
        q_ref, _ = trajectory.sample(t)
        q_deg = [f"{math.degrees(q):7.1f}" for q in q_ref]
        print(f"{t:6.2f} | {' '.join(q_deg)}")

    print("-" * line_width)

    # Show velocity profile info
    if trajectory.points:
        max_vels = [0.0] * len(trajectory.points[0].velocities)
        for pt in trajectory.points:
            for j, v in enumerate(pt.velocities):
                max_vels[j] = max(max_vels[j], abs(v))
        vel_deg = [f"{math.degrees(v):5.1f}" for v in max_vels]
        print(f"Peak velocities (deg/s): [{', '.join(vel_deg)}]")
    print("=" * line_width)


def interactive_mode(setter: TrajectorySetter) -> None:
    """Interactive mode for creating trajectories."""
    if setter.num_joints is None:
        print("Error: No joint state received. Cannot start interactive mode.")
        return

    # Generate dynamic joint list for help text
    joint_args = " ".join([f"<j{i + 1}>" for i in range(setter.num_joints)])

    print("\n" + "=" * 80)
    print("Interactive Trajectory Setter")
    print("=" * 80)
    print(f"\nArm: {setter.num_joints} joints")
    print("\nCommands:")
    print(f"  add {joint_args}   - Add waypoint (degrees)")
    print("  here                                 - Add current position as waypoint")
    print("  current                              - Show current joints")
    print("  list                                 - List waypoints")
    print("  delete <n>                           - Delete waypoint n")
    print("  preview                              - Generate and preview trajectory")
    print("  run                                  - Generate and publish trajectory")
    print("  clear                                - Clear waypoints")
    print("  vel <value>                          - Set max velocity (rad/s)")
    print("  accel <value>                        - Set max acceleration (rad/s^2)")
    print("  limits                               - Show current limits")
    print("  quit                                 - Exit")
    print("=" * 80)

    waypoints: list[list[float]] = []
    generated_trajectory: JointTrajectory | None = None

    try:
        while True:
            prompt = f"[{len(waypoints)} wp] > "
            line = input(prompt).strip()

            if not line:
                continue

            parts = line.split()
            cmd = parts[0].lower()

            # ADD waypoint
            if cmd == "add" and len(parts) >= setter.num_joints + 1:
                joints = parse_joint_input(
                    " ".join(parts[1 : setter.num_joints + 1]), setter.num_joints
                )
                if joints:
                    waypoints.append(joints)
                    generated_trajectory = None  # Invalidate cached trajectory
                    deg = [f"{math.degrees(j):.1f}" for j in joints]
                    print(f"Added waypoint {len(waypoints)}: [{', '.join(deg)}] deg")
                else:
                    print(f"Invalid joint values (need {setter.num_joints} values in degrees)")

            # HERE - add current position
            elif cmd == "here":
                joints = setter.get_current_joints()
                if joints:
                    waypoints.append(joints)
                    generated_trajectory = None
                    deg = [f"{math.degrees(j):.1f}" for j in joints]
                    print(f"Added waypoint {len(waypoints)}: [{', '.join(deg)}] deg")
                else:
                    print("No joint state available")

            # CURRENT
            elif cmd == "current":
                joints = setter.get_current_joints()
                if joints:
                    deg = [f"{math.degrees(j):.1f}" for j in joints]
                    print(f"Current: [{', '.join(deg)}] deg")
                else:
                    print("No joint state available")

            # LIST
            elif cmd == "list":
                preview_waypoints(waypoints, setter.num_joints)

            # DELETE
            elif cmd == "delete" and len(parts) >= 2:
                try:
                    idx = int(parts[1]) - 1
                    if 0 <= idx < len(waypoints):
                        waypoints.pop(idx)
                        generated_trajectory = None
                        print(f"Deleted waypoint {idx + 1}")
                    else:
                        print(f"Invalid index (1-{len(waypoints)})")
                except ValueError:
                    print("Invalid index")

            # PREVIEW
            elif cmd == "preview":
                if len(waypoints) < 2:
                    print("Need at least 2 waypoints")
                else:
                    print("\nGenerating trajectory...")
                    try:
                        generated_trajectory = setter.generate_trajectory(waypoints)
                        preview_trajectory(generated_trajectory, setter.num_joints)
                    except Exception as e:
                        print(f"Error generating trajectory: {e}")

            # RUN
            elif cmd == "run":
                if len(waypoints) < 2:
                    print("Need at least 2 waypoints")
                    continue

                # Generate if not already generated
                if generated_trajectory is None:
                    print("\nGenerating trajectory...")
                    try:
                        generated_trajectory = setter.generate_trajectory(waypoints)
                    except Exception as e:
                        print(f"Error generating trajectory: {e}")
                        continue

                preview_trajectory(generated_trajectory, setter.num_joints)
                confirm = input("\nPublish to robot? [y/N]: ").strip().lower()
                if confirm == "y":
                    setter.publish_trajectory(generated_trajectory)

            # CLEAR
            elif cmd == "clear":
                waypoints.clear()
                generated_trajectory = None
                print("Cleared")

            # VEL - set max velocity
            elif cmd == "vel" and len(parts) >= 2:
                if setter.generator is None:
                    print("Generator not initialized")
                    continue
                try:
                    vel = float(parts[1])
                    if vel <= 0:
                        print("Velocity must be positive")
                    else:
                        setter.generator.set_limits(vel, setter.generator.max_acceleration)
                        generated_trajectory = None
                        print(
                            f"Max velocity set to {vel:.2f} rad/s ({math.degrees(vel):.1f} deg/s)"
                        )
                except ValueError:
                    print("Invalid velocity")

            # ACCEL - set max acceleration
            elif cmd == "accel" and len(parts) >= 2:
                if setter.generator is None:
                    print("Generator not initialized")
                    continue
                try:
                    accel = float(parts[1])
                    if accel <= 0:
                        print("Acceleration must be positive")
                    else:
                        setter.generator.set_limits(setter.generator.max_velocity, accel)
                        generated_trajectory = None
                        print(f"Max acceleration set to {accel:.2f} rad/s^2")
                except ValueError:
                    print("Invalid acceleration")

            # LIMITS - show current limits
            elif cmd == "limits":
                if setter.generator is None:
                    print("Generator not initialized")
                    continue
                v = setter.generator.max_velocity[0]
                a = setter.generator.max_acceleration[0]
                print(f"Max velocity: {v:.2f} rad/s ({math.degrees(v):.1f} deg/s)")
                print(f"Max acceleration: {a:.2f} rad/s^2 ({math.degrees(a):.1f} deg/s^2)")

            # QUIT
            elif cmd in ("quit", "exit", "q"):
                break

            else:
                print(f"Unknown command: {cmd}")

    except KeyboardInterrupt:
        print("\n\nExiting...")


def main() -> int:
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(description="Interactive Trajectory Setter for robot arms")
    parser.add_argument(
        "--arm",
        type=str,
        default="xarm",
        choices=["xarm", "piper"],
        help="Type of arm to control (default: xarm)",
    )
    parser.add_argument(
        "--custom-arm",
        type=str,
        help="Custom arm type (will subscribe to /<custom-arm>/joint_states)",
    )
    args = parser.parse_args()

    arm_type = args.custom_arm if args.custom_arm else args.arm

    print("\n" + "=" * 80)
    print("Trajectory Setter")
    print("=" * 80)
    print(f"\nArm Type: {arm_type.upper()}")
    print("Generates joint trajectories using trapezoidal velocity profiles.")
    print("Run example_trajectory_control.py in another terminal first!")
    print("=" * 80)

    setter = TrajectorySetter(arm_type=arm_type)
    if not setter.start():
        print(f"\nWarning: Could not get joint state from /{arm_type}/joint_states")
        print("Controller may not be running or arm type may be incorrect.")
        response = input("Continue anyway? [y/N]: ").strip().lower()
        if response != "y":
            return 0

    interactive_mode(setter)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        sys.exit(0)
    except Exception as e:
        print(f"\nError: {e}")
        import traceback

        traceback.print_exc()
        sys.exit(1)

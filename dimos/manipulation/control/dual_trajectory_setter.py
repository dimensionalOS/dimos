#!/usr/bin/env python3
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
Dual-Arm Interactive Trajectory Publisher.

Interactive terminal UI for creating joint trajectories for two arms independently.
Supports running trajectories on left arm, right arm, or both simultaneously.

Workflow:
1. Add waypoints to left or right arm (or both)
2. Generator applies trapezoidal velocity profiles
3. Preview the generated trajectories
4. Run on left, right, or both arms

Use with xarm-dual-trajectory blueprint running in another terminal.
"""

from dataclasses import dataclass
import math
import sys
import time

from dimos import core
from dimos.manipulation.planning import JointTrajectoryGenerator
from dimos.msgs.sensor_msgs import JointState
from dimos.msgs.trajectory_msgs import JointTrajectory


@dataclass
class ArmState:
    """State for a single arm."""

    name: str
    num_joints: int | None = None
    latest_joint_state: JointState | None = None
    generator: JointTrajectoryGenerator | None = None
    waypoints: list[list[float]] | None = None
    generated_trajectory: JointTrajectory | None = None

    def __post_init__(self) -> None:
        self.waypoints = []


class DualTrajectorySetter:
    """
    Creates and publishes JointTrajectory for dual-arm setups.

    Manages two arms independently with separate waypoints and trajectories.
    Supports running trajectories on one or both arms.
    """

    def __init__(
        self,
        left_joint_topic: str = "/xarm/left/joint_states",
        right_joint_topic: str = "/xarm/right/joint_states",
        left_trajectory_topic: str = "/xarm/left/trajectory",
        right_trajectory_topic: str = "/xarm/right/trajectory",
    ):
        """
        Initialize the dual trajectory setter.

        Args:
            left_joint_topic: Topic for left arm joint states
            right_joint_topic: Topic for right arm joint states
            left_trajectory_topic: Topic to publish left arm trajectories
            right_trajectory_topic: Topic to publish right arm trajectories
        """
        # Arm states
        self.left = ArmState(name="left")
        self.right = ArmState(name="right")

        # Publishers for trajectories
        self.left_trajectory_pub: core.LCMTransport[JointTrajectory] = core.LCMTransport(
            left_trajectory_topic, JointTrajectory
        )
        self.right_trajectory_pub: core.LCMTransport[JointTrajectory] = core.LCMTransport(
            right_trajectory_topic, JointTrajectory
        )

        # Subscribers for joint states
        self.left_joint_sub: core.LCMTransport[JointState] = core.LCMTransport(
            left_joint_topic, JointState
        )
        self.right_joint_sub: core.LCMTransport[JointState] = core.LCMTransport(
            right_joint_topic, JointState
        )

        print("DualTrajectorySetter initialized")
        print(f"  Left arm:  {left_joint_topic} -> {left_trajectory_topic}")
        print(f"  Right arm: {right_joint_topic} -> {right_trajectory_topic}")

    def start(self) -> bool:
        """Start subscribing to joint states."""
        self.left_joint_sub.subscribe(self._on_left_joint_state)
        self.right_joint_sub.subscribe(self._on_right_joint_state)
        print("  Waiting for joint states...")

        # Wait for both arms
        left_ready = False
        right_ready = False

        for _ in range(50):  # 5 second timeout
            if not left_ready and self.left.latest_joint_state is not None:
                self.left.num_joints = len(self.left.latest_joint_state.position)
                self.left.generator = JointTrajectoryGenerator(
                    num_joints=self.left.num_joints,
                    max_velocity=1.0,
                    max_acceleration=2.0,
                    points_per_segment=50,
                )
                print(f"  Left arm ready ({self.left.num_joints} joints)")
                left_ready = True

            if not right_ready and self.right.latest_joint_state is not None:
                self.right.num_joints = len(self.right.latest_joint_state.position)
                self.right.generator = JointTrajectoryGenerator(
                    num_joints=self.right.num_joints,
                    max_velocity=1.0,
                    max_acceleration=2.0,
                    points_per_segment=50,
                )
                print(f"  Right arm ready ({self.right.num_joints} joints)")
                right_ready = True

            if left_ready and right_ready:
                return True

            time.sleep(0.1)

        if not left_ready:
            print("  Warning: Left arm not responding")
        if not right_ready:
            print("  Warning: Right arm not responding")

        return left_ready or right_ready

    def _on_left_joint_state(self, msg: JointState) -> None:
        """Callback for left arm joint state."""
        self.left.latest_joint_state = msg

    def _on_right_joint_state(self, msg: JointState) -> None:
        """Callback for right arm joint state."""
        self.right.latest_joint_state = msg

    def get_current_joints(self, arm: ArmState) -> list[float] | None:
        """Get current joint positions for an arm."""
        if arm.latest_joint_state is None or arm.num_joints is None:
            return None
        return list(arm.latest_joint_state.position[: arm.num_joints])

    def generate_trajectory(self, arm: ArmState) -> JointTrajectory | None:
        """Generate trajectory for an arm from its waypoints."""
        if arm.generator is None or not arm.waypoints or len(arm.waypoints) < 2:
            return None
        return arm.generator.generate(arm.waypoints)

    def publish_trajectory(self, arm: ArmState, trajectory: JointTrajectory) -> None:
        """Publish trajectory to an arm."""
        if arm.name == "left":
            self.left_trajectory_pub.broadcast(None, trajectory)
        else:
            self.right_trajectory_pub.broadcast(None, trajectory)
        print(
            f"  Published to {arm.name}: {len(trajectory.points)} points, "
            f"duration={trajectory.duration:.2f}s"
        )


def parse_joint_input(line: str, num_joints: int) -> list[float] | None:
    """Parse joint positions from user input (degrees by default, 'r' suffix for radians)."""
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


def preview_waypoints(arm: ArmState) -> None:
    """Show waypoints for an arm."""
    if not arm.waypoints or arm.num_joints is None:
        print(f"  {arm.name.upper()}: No waypoints")
        return

    joint_headers = " ".join([f"{'J' + str(i + 1):>7}" for i in range(arm.num_joints)])
    line_width = 6 + 3 + arm.num_joints * 8 + 10

    print(f"\n{arm.name.upper()} Waypoints ({len(arm.waypoints)}):")
    print("-" * line_width)
    print(f"  # | {joint_headers} (degrees)")
    print("-" * line_width)
    for i, joints in enumerate(arm.waypoints):
        deg = [f"{math.degrees(j):7.1f}" for j in joints]
        print(f" {i + 1:2} | {' '.join(deg)}")
    print("-" * line_width)


def preview_trajectory(arm: ArmState) -> None:
    """Show generated trajectory for an arm."""
    if arm.generated_trajectory is None or arm.num_joints is None:
        print(f"  {arm.name.upper()}: No trajectory")
        return

    traj = arm.generated_trajectory
    joint_headers = " ".join([f"{'J' + str(i + 1):>7}" for i in range(arm.num_joints)])
    line_width = 9 + 3 + arm.num_joints * 8 + 10

    print(f"\n{'=' * line_width}")
    print(f"{arm.name.upper()} TRAJECTORY")
    print(f"{'=' * line_width}")
    print(f"Duration: {traj.duration:.3f}s | Points: {len(traj.points)}")
    print("-" * line_width)
    print(f"{'Time':>6} | {joint_headers} (degrees)")
    print("-" * line_width)

    num_samples = min(10, max(len(traj.points) // 10, 5))
    for i in range(num_samples + 1):
        t = (i / num_samples) * traj.duration
        q_ref, _ = traj.sample(t)
        q_deg = [f"{math.degrees(q):7.1f}" for q in q_ref]
        print(f"{t:6.2f} | {' '.join(q_deg)}")

    print("-" * line_width)


def interactive_mode(setter: DualTrajectorySetter) -> None:
    """Interactive mode for creating dual-arm trajectories."""
    left = setter.left
    right = setter.right

    print("\n" + "=" * 80)
    print("Dual-Arm Interactive Trajectory Setter")
    print("=" * 80)

    if left.num_joints:
        print(f"  Left arm:  {left.num_joints} joints")
    else:
        print("  Left arm:  NOT CONNECTED")

    if right.num_joints:
        print(f"  Right arm: {right.num_joints} joints")
    else:
        print("  Right arm: NOT CONNECTED")

    print("\nCommands:")
    print("  left add <j1> <j2> ...   - Add waypoint to left arm (degrees)")
    print("  right add <j1> <j2> ...  - Add waypoint to right arm (degrees)")
    print("  left here                - Add current position as waypoint (left)")
    print("  right here               - Add current position as waypoint (right)")
    print("  left current             - Show current left arm joints")
    print("  right current            - Show current right arm joints")
    print("  left list                - List left arm waypoints")
    print("  right list               - List right arm waypoints")
    print("  left delete <n>          - Delete waypoint n from left")
    print("  right delete <n>         - Delete waypoint n from right")
    print("  left clear               - Clear left arm waypoints")
    print("  right clear              - Clear right arm waypoints")
    print("  preview                  - Preview both trajectories")
    print("  run left                 - Run trajectory on left arm only")
    print("  run right                - Run trajectory on right arm only")
    print("  run both                 - Run trajectories on both arms")
    print("  vel <arm> <value>        - Set max velocity (rad/s)")
    print("  quit                     - Exit")
    print("=" * 80)

    try:
        while True:
            left_wp = len(left.waypoints) if left.waypoints else 0
            right_wp = len(right.waypoints) if right.waypoints else 0
            prompt = f"[L:{left_wp} R:{right_wp}] > "
            line = input(prompt).strip()

            if not line:
                continue

            parts = line.split()
            cmd = parts[0].lower()

            # Determine which arm (if applicable)
            arm: ArmState | None = None
            if cmd in ("left", "l"):
                arm = left
                parts = parts[1:]  # Remove arm selector
                cmd = parts[0].lower() if parts else ""
            elif cmd in ("right", "r"):
                arm = right
                parts = parts[1:]
                cmd = parts[0].lower() if parts else ""

            # ARM-SPECIFIC COMMANDS
            if arm is not None:
                if arm.num_joints is None:
                    print(f"  {arm.name.upper()} arm not connected")
                    continue

                # ADD waypoint
                if cmd == "add" and len(parts) >= arm.num_joints + 1:
                    joints = parse_joint_input(
                        " ".join(parts[1 : arm.num_joints + 1]), arm.num_joints
                    )
                    if joints:
                        arm.waypoints.append(joints)  # type: ignore[union-attr]
                        arm.generated_trajectory = None
                        deg = [f"{math.degrees(j):.1f}" for j in joints]
                        print(
                            f"  {arm.name.upper()} waypoint {len(arm.waypoints)}: [{', '.join(deg)}] deg"  # type: ignore[arg-type]
                        )
                    else:
                        print(f"  Invalid values (need {arm.num_joints} in degrees)")

                # HERE - add current position
                elif cmd == "here":
                    joints = setter.get_current_joints(arm)
                    if joints:
                        arm.waypoints.append(joints)  # type: ignore[union-attr]
                        arm.generated_trajectory = None
                        deg = [f"{math.degrees(j):.1f}" for j in joints]
                        print(
                            f"  {arm.name.upper()} waypoint {len(arm.waypoints)}: [{', '.join(deg)}] deg"  # type: ignore[arg-type]
                        )
                    else:
                        print("  No joint state available")

                # CURRENT
                elif cmd == "current":
                    joints = setter.get_current_joints(arm)
                    if joints:
                        deg = [f"{math.degrees(j):.1f}" for j in joints]
                        print(f"  {arm.name.upper()}: [{', '.join(deg)}] deg")
                    else:
                        print("  No joint state available")

                # LIST
                elif cmd == "list":
                    preview_waypoints(arm)

                # DELETE
                elif cmd == "delete" and len(parts) >= 2:
                    try:
                        idx = int(parts[1]) - 1
                        if arm.waypoints and 0 <= idx < len(arm.waypoints):
                            arm.waypoints.pop(idx)
                            arm.generated_trajectory = None
                            print(f"  Deleted {arm.name} waypoint {idx + 1}")
                        else:
                            wp_count = len(arm.waypoints) if arm.waypoints else 0
                            print(f"  Invalid index (1-{wp_count})")
                    except ValueError:
                        print("  Invalid index")

                # CLEAR
                elif cmd == "clear":
                    if arm.waypoints:
                        arm.waypoints.clear()
                    arm.generated_trajectory = None
                    print(f"  {arm.name.upper()} waypoints cleared")

                else:
                    print(f"  Unknown command for {arm.name}: {cmd}")

            # GLOBAL COMMANDS
            elif cmd == "preview":
                # Generate trajectories if needed
                for a in [left, right]:
                    if a.waypoints and len(a.waypoints) >= 2:
                        try:
                            a.generated_trajectory = setter.generate_trajectory(a)
                        except Exception as e:
                            print(f"  Error generating {a.name} trajectory: {e}")
                            a.generated_trajectory = None

                preview_trajectory(left)
                preview_trajectory(right)

            elif cmd == "run" and len(parts) >= 2:
                target = parts[1].lower()

                # Determine which arms to run
                arms_to_run: list[ArmState] = []
                if target in ("left", "l"):
                    arms_to_run = [left]
                elif target in ("right", "r"):
                    arms_to_run = [right]
                elif target == "both":
                    arms_to_run = [left, right]
                else:
                    print("  Usage: run left|right|both")
                    continue

                # Generate trajectories if needed
                for a in arms_to_run:
                    if not a.waypoints or len(a.waypoints) < 2:
                        print(f"  {a.name.upper()}: Need at least 2 waypoints")
                        continue

                    if a.generated_trajectory is None:
                        try:
                            a.generated_trajectory = setter.generate_trajectory(a)
                        except Exception as e:
                            print(f"  Error generating {a.name} trajectory: {e}")
                            continue

                # Preview and confirm
                valid_arms = [a for a in arms_to_run if a.generated_trajectory is not None]
                if not valid_arms:
                    print("  No valid trajectories to run")
                    continue

                for a in valid_arms:
                    preview_trajectory(a)

                arm_names = ", ".join(a.name.upper() for a in valid_arms)
                confirm = input(f"\n  Run on {arm_names}? [y/N]: ").strip().lower()
                if confirm == "y":
                    print("\n  Publishing trajectories...")
                    for a in valid_arms:
                        if a.generated_trajectory:
                            setter.publish_trajectory(a, a.generated_trajectory)

            elif cmd == "vel" and len(parts) >= 3:
                arm_name = parts[1].lower()
                target_arm: ArmState | None = (
                    left
                    if arm_name in ("left", "l")
                    else right
                    if arm_name in ("right", "r")
                    else None
                )
                if target_arm is None or target_arm.generator is None:
                    print("  Usage: vel left|right <value>")
                    continue
                try:
                    vel = float(parts[2])
                    if vel <= 0:
                        print("  Velocity must be positive")
                    else:
                        target_arm.generator.set_limits(vel, target_arm.generator.max_acceleration)
                        target_arm.generated_trajectory = None
                        print(f"  {target_arm.name.upper()} max velocity: {vel:.2f} rad/s")
                except ValueError:
                    print("  Invalid velocity value")

            elif cmd in ("quit", "exit", "q"):
                break

            else:
                print(f"  Unknown command: {cmd}")

    except KeyboardInterrupt:
        print("\n\nExiting...")


def main() -> int:
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(description="Dual-Arm Interactive Trajectory Setter")
    parser.add_argument(
        "--left-joint-topic",
        type=str,
        default="/xarm/left/joint_states",
        help="Left arm joint state topic",
    )
    parser.add_argument(
        "--right-joint-topic",
        type=str,
        default="/xarm/right/joint_states",
        help="Right arm joint state topic",
    )
    parser.add_argument(
        "--left-trajectory-topic",
        type=str,
        default="/xarm/left/trajectory",
        help="Left arm trajectory topic",
    )
    parser.add_argument(
        "--right-trajectory-topic",
        type=str,
        default="/xarm/right/trajectory",
        help="Right arm trajectory topic",
    )
    args = parser.parse_args()

    print("\n" + "=" * 80)
    print("Dual-Arm Trajectory Setter")
    print("=" * 80)
    print("\nRun 'dimos run xarm-dual-trajectory' in another terminal first!")
    print("=" * 80)

    setter = DualTrajectorySetter(
        left_joint_topic=args.left_joint_topic,
        right_joint_topic=args.right_joint_topic,
        left_trajectory_topic=args.left_trajectory_topic,
        right_trajectory_topic=args.right_trajectory_topic,
    )

    if not setter.start():
        print("\nWarning: Could not connect to both arms")
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

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
Interactive Target Pose Publisher for Cartesian Motion Control.

Interactive terminal UI for publishing absolute target poses to /target_pose topic.
Pure publisher - OUT channel only, no subscriptions or driver connections.
"""

import math
import sys
import time

from dimos import core
from dimos.msgs.geometry_msgs import PoseStamped, Vector3, Quaternion


class TargetSetter:
    """
    Publishes target poses to /target_pose topic.

    Subscribes to /xarm/current_pose to get current TCP pose for:
    - Preserving orientation when left blank
    - Supporting relative mode movements
    """

    def __init__(self):
        """Initialize the target setter."""
        # Create LCM transport for publishing targets
        self.target_pub = core.LCMTransport("/target_pose", PoseStamped)

        # Subscribe to current pose from controller
        self.current_pose_sub = core.LCMTransport("/xarm/current_pose", PoseStamped)
        self.latest_current_pose = None

        print("TargetSetter initialized")
        print(f"  Publishing to: /target_pose")
        print(f"  Subscribing to: /xarm/current_pose")

    def start(self):
        """Start subscribing to current pose."""
        self.current_pose_sub.subscribe(self._on_current_pose)
        print("  Waiting for current pose...")
        # Wait for initial pose
        for _ in range(50):  # 5 second timeout
            if self.latest_current_pose is not None:
                print("  ✓ Current pose received")
                return True
            time.sleep(0.1)
        print("  ⚠ Warning: No current pose received (timeout)")
        return False

    def _on_current_pose(self, msg: PoseStamped) -> None:
        """Callback for current pose updates."""
        self.latest_current_pose = msg

    def publish_pose(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """
        Publish target pose (absolute world frame coordinates).

        Args:
            x, y, z: Position in meters
            roll, pitch, yaw: Orientation in radians (0, 0, 0 = preserve current)
        """
        # Check if orientation is identity (0, 0, 0) - preserve current orientation
        is_identity = abs(roll) < 1e-6 and abs(pitch) < 1e-6 and abs(yaw) < 1e-6

        if is_identity and self.latest_current_pose is not None:
            # Use current orientation
            orientation = self.latest_current_pose.orientation
            print(f"\n✓ Published target (preserving current orientation):")
        else:
            # Convert Euler to Quaternion
            euler = Vector3(roll, pitch, yaw)
            quat = Quaternion.from_euler(euler)
            orientation = [quat.x, quat.y, quat.z, quat.w]
            print(f"\n✓ Published target:")

        pose = PoseStamped(
            ts=time.time(),
            frame_id="world",
            position=[x, y, z],
            orientation=orientation,
        )

        self.target_pub.broadcast(None, pose)

        print(f"  Position: x={x:.4f}m, y={y:.4f}m, z={z:.4f}m")
        print(
            f"  Orientation: roll={math.degrees(roll):.1f}°, "
            f"pitch={math.degrees(pitch):.1f}°, yaw={math.degrees(yaw):.1f}°"
        )


def interactive_mode(setter):
    """
    Interactive mode: repeatedly prompt for target poses.

    Args:
        setter: TargetSetter instance
    """
    print("\n" + "=" * 80)
    print("Interactive Target Setter")
    print("=" * 80)
    print("Mode: WORLD FRAME (absolute coordinates)")
    print("\nEnter target coordinates (Ctrl+C to quit)")
    print("=" * 80)

    try:
        while True:
            print("\n" + "-" * 80)

            # Get position
            try:
                print("\nEnter target position (in meters):")
                x_str = input("  x (m): ").strip()
                y_str = input("  y (m): ").strip()
                z_str = input("  z (m): ").strip()

                if not x_str or not y_str or not z_str:
                    print("⚠ All position coordinates required")
                    continue

                x = float(x_str)
                y = float(y_str)
                z = float(z_str)

                # Get orientation
                print(
                    "\nEnter orientation (in degrees, leave blank to preserve current orientation):"
                )
                roll_str = input("  roll (°): ").strip()
                pitch_str = input("  pitch (°): ").strip()
                yaw_str = input("  yaw (°): ").strip()

                roll = math.radians(float(roll_str)) if roll_str else 0.0
                pitch = math.radians(float(pitch_str)) if pitch_str else 0.0
                yaw = math.radians(float(yaw_str)) if yaw_str else 0.0

                # Publish
                setter.publish_pose(x, y, z, roll, pitch, yaw)

            except ValueError as e:
                print(f"⚠ Invalid input: {e}")
                continue

    except KeyboardInterrupt:
        print("\n\nExiting interactive mode...")


def print_banner():
    """Print welcome banner."""
    print("\n" + "=" * 80)
    print("xArm Target Pose Publisher")
    print("=" * 80)
    print("\nPublishes absolute target poses to /target_pose topic.")
    print("Subscribes to /xarm/current_pose for orientation preservation.")
    print("=" * 80)


def main():
    """Main entry point."""
    print_banner()

    # Create setter and start subscribing to current pose
    setter = TargetSetter()
    if not setter.start():
        print("\n⚠ Warning: Could not get current pose - controller may not be running")
        print("Make sure example_cartesian_control.py is running in another terminal!")
        response = input("Continue anyway? [y/N]: ").strip().lower()
        if response != "y":
            return 0

    try:
        # Run interactive mode
        interactive_mode(setter)
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")

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

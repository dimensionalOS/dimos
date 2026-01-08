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
Manipulation Client - Interactive CLI for ManipulationModule

This script connects to a running ManipulationModule via LCM RPC and provides
planning_tester.py-like functionality:
- Add/remove obstacles (visible in Drake/Meshcat)
- Plan motions to poses or joint configurations
- Preview planned paths in Drake/Meshcat visualization
- Execute planned trajectories in MuJoCo simulation
- Query state, EE pose, collision status

Workflow:
    1. Start the manipulation blueprint (ManipulationModule + MuJoCo)
    2. Run this client to interact
    3. Add obstacles → see them in Drake and MuJoCo
    4. Plan a path → preview in Drake → execute in MuJoCo

Usage:
    # Start the manipulation blueprint first:
    dimos run xarm6-manipulation

    # Then run this client interactively:
    python manipulation_client.py

    # Or use command-line args for scripting:
    python manipulation_client.py --add-box table 0.5 0.0 0.0 0.6 0.4 0.02
    python manipulation_client.py --plan-pose 0.4 0.0 0.3 3.14 0.0 0.0
    python manipulation_client.py --preview
    python manipulation_client.py --execute

Interactive Commands:
    # Query
    state, ee, joints, url

    # Immediate motion (plan + execute in one step)
    pose x y z [r p y]
    joint j1 j2 j3 j4 j5 j6

    # Plan/Preview/Execute workflow (2-button approach)
    plan pose x y z [r p y]   - Plan only (see in Drake)
    plan joint j1 j2 ...      - Plan only (see in Drake)
    preview [speed]           - Animate path in Drake/Meshcat
    execute                   - Send trajectory to MuJoCo

    # Obstacles
    box name x y z w h d [r p y]
    sphere name x y z radius
    remove id
    clear
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from dimos.protocol.rpc import LCMRPC


class ManipulationClient:
    """Client for interacting with ManipulationModule via LCM RPC.

    Provides planning_tester.py-like functionality via RPC:
    - Plan paths (without executing) and preview in Drake/Meshcat
    - Execute planned trajectories in MuJoCo
    - Add/remove obstacles (visible in both Drake and MuJoCo)
    """

    def __init__(self) -> None:
        self.rpc = LCMRPC()
        self.rpc.start()
        self.module_name = "ManipulationModule"
        print("ManipulationClient connected via LCM RPC")

    def _call(self, method: str, *args, **kwargs) -> any:
        """Call an RPC method on ManipulationModule."""
        topic = f"{self.module_name}/{method}"
        try:
            result, _ = self.rpc.call_sync(topic, (args, kwargs), rpc_timeout=30.0)
            return result
        except TimeoutError:
            print(f"RPC call to '{method}' timed out")
            return None
        except Exception as e:
            print(f"RPC error: {e}")
            return None

    def get_state(self) -> str:
        """Get current manipulation state."""
        return self._call("get_state_name")

    def get_ee_pose(self) -> list[float] | None:
        """Get current end-effector pose [x, y, z, roll, pitch, yaw]."""
        return self._call("get_ee_pose")

    def get_joints(self) -> list[float] | None:
        """Get current joint positions."""
        return self._call("get_current_joints")

    def move_to_pose(
        self, x: float, y: float, z: float, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0
    ) -> bool:
        """Move to end-effector pose."""
        print(
            f"Moving to pose: ({x:.3f}, {y:.3f}, {z:.3f}) orientation: ({roll:.2f}, {pitch:.2f}, {yaw:.2f})"
        )
        return self._call("move_to_pose", x, y, z, roll, pitch, yaw)

    def move_to_joints(self, joints: list[float]) -> bool:
        """Move to joint configuration."""
        print(f"Moving to joints: {[f'{j:.3f}' for j in joints]}")
        return self._call("move_to_joints", joints)

    def add_box(
        self,
        name: str,
        x: float,
        y: float,
        z: float,
        width: float,
        height: float,
        depth: float,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
    ) -> str:
        """Add a box obstacle."""
        print(
            f"Adding box '{name}' at ({x:.3f}, {y:.3f}, {z:.3f}) size ({width:.3f}, {height:.3f}, {depth:.3f})"
        )
        return self._call("add_box_obstacle", name, x, y, z, width, height, depth, roll, pitch, yaw)

    def add_sphere(self, name: str, x: float, y: float, z: float, radius: float) -> str:
        """Add a sphere obstacle."""
        print(f"Adding sphere '{name}' at ({x:.3f}, {y:.3f}, {z:.3f}) radius {radius:.3f}")
        return self._call("add_sphere_obstacle", name, x, y, z, radius)

    def remove_obstacle(self, obstacle_id: str) -> bool:
        """Remove an obstacle."""
        return self._call("remove_obstacle", obstacle_id)

    def clear_obstacles(self) -> bool:
        """Clear all obstacles."""
        return self._call("clear_obstacles")

    def reset(self) -> bool:
        """Reset manipulation state to IDLE."""
        return self._call("reset")

    # =========================================================================
    # Plan/Preview/Execute Workflow (like planning_tester.py)
    # =========================================================================

    def plan_to_pose(
        self, x: float, y: float, z: float, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0
    ) -> bool:
        """Plan motion to pose WITHOUT executing. Use preview() then execute()."""
        print(
            f"Planning to pose: ({x:.3f}, {y:.3f}, {z:.3f}) orientation: ({roll:.2f}, {pitch:.2f}, {yaw:.2f})"
        )
        return self._call("plan_to_pose", x, y, z, roll, pitch, yaw)

    def plan_to_joints(self, joints: list[float]) -> bool:
        """Plan motion to joints WITHOUT executing. Use preview() then execute()."""
        print(f"Planning to joints: {[f'{j:.3f}' for j in joints]}")
        return self._call("plan_to_joints", joints)

    def preview(self, speed: float = 1.0) -> bool:
        """Preview the planned path in Drake/Meshcat visualizer."""
        print(f"Previewing path in Drake (speed={speed}x)...")
        return self._call("preview_path_in_drake", speed)

    def execute(self) -> bool:
        """Execute the planned trajectory (send to MuJoCo/controller)."""
        print("Executing planned trajectory...")
        return self._call("execute_planned")

    def has_plan(self) -> bool:
        """Check if there's a planned path ready."""
        return self._call("has_planned_path")

    def get_viz_url(self) -> str:
        """Get the Meshcat visualization URL."""
        return self._call("get_visualization_url")

    def is_collision_free(self, joints: list[float]) -> bool:
        """Check if joint configuration is collision-free."""
        return self._call("is_collision_free", joints)

    def get_debug_info(self) -> dict:
        """Get debug info about joint state flow."""
        return self._call("get_debug_info")

    def stop(self) -> None:
        """Stop the RPC client."""
        self.rpc.stop()

    def get_orientation(
        self, orientation_values: list[float] | None = None
    ) -> tuple[float, float, float]:
        """Get orientation (roll, pitch, yaw) from provided values or current EE pose.

        Args:
            orientation_values: Optional list of orientation values [roll, pitch, yaw] or partial [roll] or [roll, pitch]

        Returns:
            Tuple of (roll, pitch, yaw) in radians

        Raises:
            RuntimeError: If orientation_values is not provided and current EE pose cannot be retrieved
        """
        if orientation_values is not None and len(orientation_values) > 0:
            roll = orientation_values[0]
            pitch = orientation_values[1] if len(orientation_values) > 1 else 0.0
            yaw = orientation_values[2] if len(orientation_values) > 2 else 0.0
            return roll, pitch, yaw

        # Get current EE pose and use its orientation
        ee_pose = self.get_ee_pose()
        if ee_pose is not None and len(ee_pose) >= 6:
            roll, pitch, yaw = ee_pose[3], ee_pose[4], ee_pose[5]
            print(f"Using current orientation: roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}")
            return roll, pitch, yaw
        else:
            # Raise error if EE pose not available
            raise RuntimeError(
                "Could not get current end-effector pose. Please provide orientation explicitly (roll, pitch, yaw)."
            )


def interactive_mode(client: ManipulationClient) -> None:
    """Run interactive CLI mode."""
    print("\n" + "=" * 60)
    print("Manipulation Client - Interactive Mode")
    print("=" * 60)
    print("\nQuery Commands:")
    print("  state              - Get current state")
    print("  ee                 - Get end-effector pose")
    print("  joints             - Get joint positions")
    print("  url                - Get Drake/Meshcat visualization URL")
    print("\nImmediate Motion (plan + execute in one step):")
    print("  pose x y z [r p y] - Move to pose")
    print("  joint j1 j2 ...    - Move to joint config")
    print("\nPlan/Preview/Execute Workflow:")
    print("  plan pose x y z [r p y]  - Plan to pose (don't execute)")
    print("  plan joint j1 j2 ...     - Plan to joints (don't execute)")
    print("  preview [speed]          - Preview path in Drake/Meshcat")
    print("  execute                  - Execute planned trajectory in MuJoCo")
    print("  hasplan                  - Check if path is planned")
    print("\nObstacles:")
    print("  box name x y z w h d [r p y] - Add box obstacle")
    print("  sphere name x y z radius     - Add sphere obstacle")
    print("  remove id          - Remove obstacle")
    print("  clear              - Clear all obstacles")
    print("\nOther:")
    print("  collision j1 j2... - Check collision at joint config")
    print("  reset              - Reset state to IDLE")
    print("  help               - Show this help")
    print("  quit               - Exit")
    print("=" * 60)

    while True:
        try:
            cmd = input("\n> ").strip()
            if not cmd:
                continue

            parts = cmd.split()
            action = parts[0].lower()

            if action == "quit" or action == "q":
                break

            elif action == "help" or action == "h":
                # Re-print help
                print("\nQuery: state, ee, joints, url")
                print("Immediate: pose x y z [r p y], joint j1 j2...")
                print(
                    "Plan/Preview/Execute: plan pose/joint ..., preview [speed], execute, hasplan"
                )
                print("Obstacles: box name x y z w h d, sphere name x y z r, remove id, clear")
                print("Other: collision j1 j2..., reset, quit")

            elif action == "state":
                state = client.get_state()
                print(f"State: {state}")

            elif action == "ee":
                pose = client.get_ee_pose()
                if pose:
                    print(f"EE Pose: x={pose[0]:.4f}, y={pose[1]:.4f}, z={pose[2]:.4f}")
                    print(f"         roll={pose[3]:.4f}, pitch={pose[4]:.4f}, yaw={pose[5]:.4f}")

            elif action == "joints":
                joints = client.get_joints()
                if joints:
                    print(f"Joints: {[f'{j:.4f}' for j in joints]}")

            elif action == "url":
                url = client.get_viz_url()
                if url:
                    print(f"Meshcat URL: {url}")
                else:
                    print("Visualization not enabled or URL not available")

            # Immediate motion (plan + execute in one step)
            elif action == "pose":
                if len(parts) < 4:
                    print("Usage: pose x y z [roll pitch yaw]")
                    continue
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                orientation_values = [float(v) for v in parts[4:7]] if len(parts) > 4 else None
                roll, pitch, yaw = client.get_orientation(orientation_values)
                success = client.move_to_pose(x, y, z, roll, pitch, yaw)
                print(f"Success: {success}")

            elif action == "joint":
                if len(parts) < 2:
                    print("Usage: joint j1 j2 j3 j4 j5 j6")
                    continue
                joints = [float(j) for j in parts[1:]]
                success = client.move_to_joints(joints)
                print(f"Success: {success}")

            # Plan/Preview/Execute workflow
            elif action == "plan":
                if len(parts) < 2:
                    print("Usage: plan pose x y z [r p y]")
                    print("       plan joint j1 j2 j3 j4 j5 j6")
                    continue
                subaction = parts[1].lower()
                if subaction == "pose":
                    if len(parts) < 5:
                        print("Usage: plan pose x y z [roll pitch yaw]")
                        continue
                    x, y, z = float(parts[2]), float(parts[3]), float(parts[4])
                    orientation_values = [float(v) for v in parts[5:8]] if len(parts) > 5 else None
                    roll, pitch, yaw = client.get_orientation(orientation_values)
                    success = client.plan_to_pose(x, y, z, roll, pitch, yaw)
                    print(f"Planning success: {success}")
                    if success:
                        print("Use 'preview' to see path in Drake, then 'execute' to run in MuJoCo")
                elif subaction == "joint":
                    if len(parts) < 3:
                        print("Usage: plan joint j1 j2 j3 j4 j5 j6")
                        continue
                    joints = [float(j) for j in parts[2:]]
                    success = client.plan_to_joints(joints)
                    print(f"Planning success: {success}")
                    if success:
                        print("Use 'preview' to see path in Drake, then 'execute' to run in MuJoCo")
                else:
                    print("Usage: plan pose x y z ... OR plan joint j1 j2 ...")

            elif action == "preview":
                speed = float(parts[1]) if len(parts) > 1 else 1.0
                success = client.preview(speed)
                print(f"Preview success: {success}")

            elif action == "execute":
                success = client.execute()
                print(f"Execute success: {success}")

            elif action == "hasplan":
                has = client.has_plan()
                print(f"Has planned path: {has}")

            # Obstacles
            elif action == "box":
                if len(parts) < 8:
                    print("Usage: box name x y z width height depth [roll pitch yaw]")
                    continue
                name = parts[1]
                x, y, z = float(parts[2]), float(parts[3]), float(parts[4])
                w, h, d = float(parts[5]), float(parts[6]), float(parts[7])
                roll = float(parts[8]) if len(parts) > 8 else 0.0
                pitch = float(parts[9]) if len(parts) > 9 else 0.0
                yaw = float(parts[10]) if len(parts) > 10 else 0.0
                obstacle_id = client.add_box(name, x, y, z, w, h, d, roll, pitch, yaw)
                print(f"Obstacle ID: {obstacle_id}")

            elif action == "sphere":
                if len(parts) < 6:
                    print("Usage: sphere name x y z radius")
                    continue
                name = parts[1]
                x, y, z = float(parts[2]), float(parts[3]), float(parts[4])
                radius = float(parts[5])
                obstacle_id = client.add_sphere(name, x, y, z, radius)
                print(f"Obstacle ID: {obstacle_id}")

            elif action == "remove":
                if len(parts) < 2:
                    print("Usage: remove obstacle_id")
                    continue
                success = client.remove_obstacle(parts[1])
                print(f"Removed: {success}")

            elif action == "clear":
                success = client.clear_obstacles()
                print(f"Cleared: {success}")

            # Other
            elif action == "collision":
                if len(parts) < 2:
                    print("Usage: collision j1 j2 j3 j4 j5 j6")
                    continue
                joints = [float(j) for j in parts[1:]]
                is_free = client.is_collision_free(joints)
                status = "COLLISION-FREE" if is_free else "IN COLLISION"
                print(f"Status: {status}")

            elif action == "reset":
                success = client.reset()
                print(f"Reset: {success}")

            elif action == "debug":
                info = client.get_debug_info()
                print("Debug Info:")
                for key, value in info.items():
                    print(f"  {key}: {value}")

            else:
                print(f"Unknown command: {action}. Type 'help' for commands.")

        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except ValueError as e:
            print(f"Invalid value: {e}")
        except Exception as e:
            print(f"Error: {e}")


def main() -> None:
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Client for ManipulationModule",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # Query commands
    parser.add_argument("--state", action="store_true", help="Get current state")
    parser.add_argument("--ee", action="store_true", help="Get EE pose")
    parser.add_argument("--joints", action="store_true", help="Get joint positions")
    parser.add_argument("--url", action="store_true", help="Get Meshcat visualization URL")

    # Immediate motion (plan + execute)
    parser.add_argument(
        "--move-pose",
        nargs="+",
        type=float,
        metavar="FLOAT",
        help="Move to pose: x y z [roll pitch yaw]",
    )
    parser.add_argument(
        "--move-joints",
        nargs="+",
        type=float,
        metavar="FLOAT",
        help="Move to joints: j1 j2 j3 j4 j5 j6",
    )

    # Plan/Preview/Execute workflow
    parser.add_argument(
        "--plan-pose",
        nargs="+",
        type=float,
        metavar="FLOAT",
        help="Plan to pose (no execute): x y z [roll pitch yaw]",
    )
    parser.add_argument(
        "--plan-joints",
        nargs="+",
        type=float,
        metavar="FLOAT",
        help="Plan to joints (no execute): j1 j2 j3 j4 j5 j6",
    )
    parser.add_argument(
        "--preview",
        nargs="?",
        type=float,
        const=1.0,
        metavar="SPEED",
        help="Preview planned path in Drake",
    )
    parser.add_argument(
        "--execute", action="store_true", help="Execute planned trajectory in MuJoCo"
    )

    # Obstacles
    parser.add_argument(
        "--add-box",
        nargs="+",
        metavar="ARG",
        help="Add box: name x y z width height depth [roll pitch yaw]",
    )
    parser.add_argument(
        "--add-sphere",
        nargs=5,
        metavar="ARG",
        help="Add sphere: name x y z radius",
    )
    parser.add_argument("--clear", action="store_true", help="Clear all obstacles")

    # Other
    parser.add_argument("--reset", action="store_true", help="Reset state")
    parser.add_argument("-i", "--interactive", action="store_true", help="Interactive mode")

    args = parser.parse_args()

    client = ManipulationClient()

    try:
        # If no args, default to interactive mode
        if len(sys.argv) == 1:
            interactive_mode(client)
            return

        if args.interactive:
            interactive_mode(client)
            return

        # Query commands
        if args.state:
            state = client.get_state()
            print(f"State: {state}")

        if args.ee:
            pose = client.get_ee_pose()
            if pose:
                print(f"EE Pose: {pose}")

        if args.joints:
            joints = client.get_joints()
            if joints:
                print(f"Joints: {joints}")

        if args.url:
            url = client.get_viz_url()
            print(f"Meshcat URL: {url}" if url else "Visualization not available")

        # Immediate motion
        if args.move_pose:
            p = args.move_pose
            x, y, z = p[0], p[1], p[2]
            orientation_values = p[3:6] if len(p) > 3 else None
            roll, pitch, yaw = client.get_orientation(orientation_values)
            success = client.move_to_pose(x, y, z, roll, pitch, yaw)
            print(f"Success: {success}")

        if args.move_joints:
            success = client.move_to_joints(args.move_joints)
            print(f"Success: {success}")

        # Plan/Preview/Execute workflow
        if args.plan_pose:
            p = args.plan_pose
            x, y, z = p[0], p[1], p[2]
            orientation_values = p[3:6] if len(p) > 3 else None
            roll, pitch, yaw = client.get_orientation(orientation_values)
            success = client.plan_to_pose(x, y, z, roll, pitch, yaw)
            print(f"Planning success: {success}")

        if args.plan_joints:
            success = client.plan_to_joints(args.plan_joints)
            print(f"Planning success: {success}")

        if args.preview is not None:
            success = client.preview(args.preview)
            print(f"Preview success: {success}")

        if args.execute:
            success = client.execute()
            print(f"Execute success: {success}")

        # Obstacles
        if args.add_box:
            b = args.add_box
            name = b[0]
            x, y, z = float(b[1]), float(b[2]), float(b[3])
            w, h, d = float(b[4]), float(b[5]), float(b[6])
            roll = float(b[7]) if len(b) > 7 else 0.0
            pitch = float(b[8]) if len(b) > 8 else 0.0
            yaw = float(b[9]) if len(b) > 9 else 0.0
            obstacle_id = client.add_box(name, x, y, z, w, h, d, roll, pitch, yaw)
            print(f"Obstacle ID: {obstacle_id}")

        if args.add_sphere:
            s = args.add_sphere
            obstacle_id = client.add_sphere(
                s[0], float(s[1]), float(s[2]), float(s[3]), float(s[4])
            )
            print(f"Obstacle ID: {obstacle_id}")

        if args.clear:
            success = client.clear_obstacles()
            print(f"Cleared: {success}")

        if args.reset:
            success = client.reset()
            print(f"Reset: {success}")

    finally:
        client.stop()


if __name__ == "__main__":
    main()

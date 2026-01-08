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
Interactive Planning Tester

A comprehensive testing tool for the manipulation planning stack:
- Robot visualization and joint control
- Obstacle management (add, move, remove)
- Inverse kinematics (IK) testing
- Motion planning testing
- Path visualization

Usage:
    python planning_tester.py [--robot ROBOT]

    Supported robots:
        piper   - Agilex Piper 6-DOF arm (default)
        xarm6   - UFactory xArm 6-DOF arm
        xarm7   - UFactory xArm 7-DOF arm

    Examples:
        python planning_tester.py --robot piper
        python planning_tester.py --robot xarm6

Commands:
    # Robot Control
    joints (j)   - Set robot joint positions
    home         - Move robot to home position
    random       - Move robot to random configuration
    ee           - Show end-effector pose
    collision    - Check current collision status

    # Planning
    ik           - Solve inverse kinematics to target pose
    plan (p)     - Plan path to goal configuration

    # Obstacles
    add (a)      - Add obstacle
    move (m)     - Move obstacle
    remove (r)   - Remove obstacle
    list (l)     - List obstacles
    clear        - Clear all obstacles

    # Other
    help (h)     - Show help
    quit (q)     - Exit
"""

from __future__ import annotations

from pathlib import Path
import sys
import time
from typing import TYPE_CHECKING

import numpy as np

from dimos.manipulation.planning.factory import (
    create_kinematics,
    create_planner,
    create_world,
)
from dimos.manipulation.planning.spec import (
    Obstacle,
    ObstacleType,
    RobotModelConfig,
)
from dimos.manipulation.planning.utils.path_utils import interpolate_path

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.manipulation.planning.spec import (
        KinematicsSpec,
        PlannerSpec,
        WorldSpec,
    )


class PlanningTester:
    """Interactive planning tester with robot, obstacles, IK, and planning."""

    # Supported robot configurations
    ROBOT_CONFIGS = {
        "piper": {
            "name": "piper",
            "urdf_subpath": "piper/piper_description/urdf/piper_description.urdf",
            "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            "end_effector_link": "link6",
            "base_link": "base_link",
            "package_name": "piper_description",
            "package_subpath": "piper/piper_description",
            "is_xacro": False,
        },
        "xarm6": {
            "name": "xarm6",
            "urdf_subpath": "xarm/xarm_description/urdf/xarm_device.urdf.xacro",
            "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            "end_effector_link": "link6",
            "base_link": "link_base",
            "package_name": "xarm_description",
            "package_subpath": "xarm/xarm_description",
            "is_xacro": True,
            "xacro_args": {"dof": "6", "limited": "true"},
        },
        "xarm7": {
            "name": "xarm7",
            "urdf_subpath": "xarm/xarm_description/urdf/xarm_device.urdf.xacro",
            "joint_names": [
                "joint1",
                "joint2",
                "joint3",
                "joint4",
                "joint5",
                "joint6",
                "joint7",
            ],
            "end_effector_link": "link7",
            "base_link": "link_base",
            "package_name": "xarm_description",
            "package_subpath": "xarm/xarm_description",
            "is_xacro": True,
            "xacro_args": {"dof": "7", "limited": "true"},
        },
    }

    def __init__(self, robot_type: str = "piper"):
        """Initialize the planning tester.

        Args:
            robot_type: Robot type to use ("piper", "xarm6", "xarm7")
        """
        if robot_type not in self.ROBOT_CONFIGS:
            raise ValueError(
                f"Unknown robot type: {robot_type}. Supported: {list(self.ROBOT_CONFIGS.keys())}"
            )
        self._robot_type = robot_type
        self._world: WorldSpec | None = None
        self._kinematics: KinematicsSpec | None = None
        self._planner: PlannerSpec | None = None
        self._robot_id: str | None = None
        self._obstacles: dict[str, Obstacle] = {}
        self._obstacle_counter = 0
        self._current_joints: NDArray[np.float64] | None = None

    def setup(self) -> bool:
        """Set up the planning stack."""
        print("=" * 60)
        print(f"Planning Tester - {self._robot_type.upper()}")
        print("=" * 60)

        # Create components
        print("\n1. Creating planning stack...")
        self._world = create_world(backend="drake", enable_viz=True)
        self._kinematics = create_kinematics(backend="drake")
        # Use larger step sizes for faster exploration
        self._planner = create_planner(
            name="rrt_connect",
            backend="drake",
            step_size=0.2,  # Larger steps for faster tree growth
            connect_step_size=0.15,  # Faster connect attempts
            goal_tolerance=0.15,  # Slightly more lenient
            collision_step_size=0.05,  # Coarser collision checking
        )
        print("   World, Kinematics, Planner created")

        # Load robot
        print("\n2. Loading robot...")
        if not self._load_robot():
            return False

        # Finalize
        print("\n3. Finalizing world...")
        self._world.finalize()
        print("   World finalized")

        # Initialize robot position (DOF from robot config)
        num_joints = len(self.ROBOT_CONFIGS[self._robot_type]["joint_names"])
        self._current_joints = np.zeros(num_joints)
        self._world.sync_from_joint_state(self._robot_id, self._current_joints)
        self._world.publish_to_meshcat()

        # Add default test obstacle (commented out for dynamic obstacle testing)
        # print("\n4. Adding test obstacle...")
        # self._add_default_obstacle()

        # Show URL
        if hasattr(self._world, "get_meshcat_url"):
            url = self._world.get_meshcat_url()
            print(f"\n   Meshcat URL: {url}")
            print("   Open this URL in your browser to see the visualization.")

        return True

    def _add_default_obstacle(self) -> None:
        """Add a default test obstacle."""
        obstacle_pose = np.eye(4)
        obstacle_pose[:3, 3] = [0.0, 0.0, 0.33]  # Above the robot base

        obstacle = Obstacle(
            name="test_obstacle",
            obstacle_type=ObstacleType.BOX,
            pose=obstacle_pose,
            dimensions=(0.05, 0.05, 0.05),
            color=(1.0, 0.0, 0.0, 0.8),
        )

        self._world.add_obstacle(obstacle)
        self._obstacles[obstacle.name] = obstacle
        self._world.publish_to_meshcat()
        self._obstacle_counter = 1

        print(f"   Added obstacle: {obstacle.name} at {obstacle_pose[:3, 3]}")

    def _load_robot(self) -> bool:
        """Load the robot model based on selected robot type."""
        robot_cfg = self.ROBOT_CONFIGS[self._robot_type]

        # Path: examples -> planning -> manipulation -> dimos -> hardware
        base_path = Path(__file__).parent.parent.parent.parent / "hardware" / "manipulators"
        urdf_path = base_path / robot_cfg["urdf_subpath"]

        if not urdf_path.exists():
            print(f"   ERROR: Robot URDF not found: {urdf_path}")
            return False

        # Build xacro args if needed
        xacro_args = robot_cfg.get("xacro_args", {})

        config = RobotModelConfig(
            name=robot_cfg["name"],
            urdf_path=str(urdf_path),
            base_pose=np.eye(4),
            joint_names=robot_cfg["joint_names"],
            end_effector_link=robot_cfg["end_effector_link"],
            base_link=robot_cfg["base_link"],
            package_paths={
                robot_cfg["package_name"]: str(base_path / robot_cfg["package_subpath"]),
            },
            auto_convert_meshes=True,
            xacro_args=xacro_args,
        )

        self._robot_id = self._world.add_robot(config)
        print(f"   Loaded robot: {self._robot_id} ({self._robot_type})")
        return True

    def run(self) -> None:
        """Run the interactive loop."""
        self._print_help()

        while True:
            try:
                cmd = input("\n> ").strip().lower()

                if cmd == "quit" or cmd == "q":
                    print("Exiting...")
                    break
                elif cmd == "help" or cmd == "h":
                    self._print_help()
                # Robot commands
                elif cmd == "joints" or cmd == "j":
                    self._set_joints()
                elif cmd == "home":
                    self._go_home()
                elif cmd == "random":
                    self._go_random()
                elif cmd == "ee":
                    self._show_ee_pose()
                # IK/Planning commands
                elif cmd == "ik":
                    self._solve_ik()
                elif cmd == "plan" or cmd == "p":
                    self._plan_path()
                elif cmd == "collision":
                    self._check_collision()
                # Obstacle commands
                elif cmd == "add" or cmd == "a":
                    self._add_obstacle()
                elif cmd == "move" or cmd == "m":
                    self._move_obstacle()
                elif cmd == "remove" or cmd == "r":
                    self._remove_obstacle()
                elif cmd == "list" or cmd == "l":
                    self._list_obstacles()
                elif cmd == "clear":
                    self._clear_obstacles()
                else:
                    print(f"Unknown command: {cmd}")
                    print("Type 'help' for available commands.")

            except KeyboardInterrupt:
                print("\nExiting...")
                break
            except EOFError:
                print("\nExiting...")
                break

    def _print_help(self) -> None:
        """Print help message."""
        print("\n" + "=" * 40)
        print("Commands:")
        print("=" * 40)
        print("\nRobot Control:")
        print("  joints (j)  - Set joint positions")
        print("  home        - Go to home position (zeros)")
        print("  random      - Go to random configuration")
        print("  ee          - Show end-effector pose")
        print("  collision   - Check collision status")
        print("\nPlanning:")
        print("  ik          - Solve IK to target pose")
        print("  plan (p)    - Plan path to goal")
        print("\nObstacles:")
        print("  add (a)     - Add obstacle")
        print("  move (m)    - Move obstacle")
        print("  remove (r)  - Remove obstacle")
        print("  list (l)    - List obstacles")
        print("  clear       - Clear all obstacles")
        print("\nOther:")
        print("  help (h)    - Show this help")
        print("  quit (q)    - Exit")

    # =========================================================================
    # Robot Control
    # =========================================================================

    def _set_joints(self) -> None:
        """Set robot joint positions."""
        print("\nEnter 6 joint values (space-separated, in radians):")
        print(f"Current: {self._format_array(self._current_joints)}")

        try:
            values_str = input("Joints: ").strip()
            if not values_str:
                print("Cancelled.")
                return

            values = [float(x) for x in values_str.replace(",", " ").split()]
            if len(values) != 6:
                print("Expected 6 values.")
                return

            joints = np.array(values)
            self._move_robot_to(joints)

        except ValueError as e:
            print(f"Invalid input: {e}")

    def _go_home(self) -> None:
        """Move robot to home position."""
        print("Moving to home position...")
        self._move_robot_to(np.zeros(6))

    def _go_random(self) -> None:
        """Move robot to random configuration."""
        print("Moving to random configuration...")
        lower, upper = self._world.get_joint_limits(self._robot_id)
        joints = np.random.uniform(lower * 0.8, upper * 0.8)  # Stay within 80% of limits
        self._move_robot_to(joints)

    def _move_robot_to(self, joints: NDArray[np.float64], animate: bool = True) -> None:
        """Move robot to target joints with optional animation."""
        if animate and self._current_joints is not None:
            # Interpolate for smooth motion
            path = [self._current_joints, joints]
            interpolated = interpolate_path(path, resolution=0.05)

            for waypoint in interpolated:
                self._world.sync_from_joint_state(self._robot_id, waypoint)
                self._world.publish_to_meshcat()
                time.sleep(0.02)
        else:
            self._world.sync_from_joint_state(self._robot_id, joints)
            self._world.publish_to_meshcat()

        self._current_joints = joints.copy()

        # Show result
        with self._world.scratch_context() as ctx:
            self._world.set_positions(ctx, self._robot_id, joints)
            ee_pose = self._world.get_ee_pose(ctx, self._robot_id)
            is_free = self._world.is_collision_free(ctx, self._robot_id)

        print(f"Joints: {self._format_array(joints)}")
        print(f"EE Position: {self._format_array(ee_pose[:3, 3])}")
        print(f"Collision-free: {is_free}")

    def _show_ee_pose(self) -> None:
        """Show current end-effector pose."""
        with self._world.scratch_context() as ctx:
            self._world.set_positions(ctx, self._robot_id, self._current_joints)
            ee_pose = self._world.get_ee_pose(ctx, self._robot_id)

        print("\nEnd-Effector Pose:")
        print(f"  Position: {self._format_array(ee_pose[:3, 3])}")
        print("  Rotation matrix:")
        for i in range(3):
            print(f"    [{ee_pose[i, 0]:7.4f}, {ee_pose[i, 1]:7.4f}, {ee_pose[i, 2]:7.4f}]")

    def _check_collision(self) -> None:
        """Check current collision status."""
        with self._world.scratch_context() as ctx:
            self._world.set_positions(ctx, self._robot_id, self._current_joints)
            is_free = self._world.is_collision_free(ctx, self._robot_id)
            min_dist = self._world.get_min_distance(ctx, self._robot_id)

        status = "COLLISION-FREE" if is_free else "IN COLLISION"
        print(f"\nCollision Status: {status}")
        print(f"Minimum distance: {min_dist:.4f} m")

    # =========================================================================
    # IK and Planning
    # =========================================================================

    def _solve_ik(self) -> None:
        """Solve IK to target pose."""
        print("\nInverse Kinematics Solver")
        print("-" * 40)

        # Get target position
        print("Enter target position (x, y, z) or 'current' for current EE position:")
        pos_str = input("Position: ").strip()

        if pos_str.lower() == "current":
            with self._world.scratch_context() as ctx:
                self._world.set_positions(ctx, self._robot_id, self._current_joints)
                current_pose = self._world.get_ee_pose(ctx, self._robot_id)
            target_pos = current_pose[:3, 3]
            print(f"Using current: {self._format_array(target_pos)}")
        else:
            try:
                parts = [float(x) for x in pos_str.replace(",", " ").split()]
                if len(parts) != 3:
                    print("Expected 3 values (x, y, z)")
                    return
                target_pos = np.array(parts)
            except ValueError:
                print("Invalid position")
                return

        # Build target pose (keep current orientation or use default)
        print("Use current orientation? (y/n, default=y):")
        orient_choice = input("Choice: ").strip().lower()

        if orient_choice == "n":
            # Use a default pointing-down orientation
            target_pose = np.array(
                [
                    [1, 0, 0, target_pos[0]],
                    [0, -1, 0, target_pos[1]],
                    [0, 0, -1, target_pos[2]],
                    [0, 0, 0, 1],
                ]
            )
        else:
            with self._world.scratch_context() as ctx:
                self._world.set_positions(ctx, self._robot_id, self._current_joints)
                target_pose = self._world.get_ee_pose(ctx, self._robot_id).copy()
            target_pose[:3, 3] = target_pos

        # Solve IK
        print("\nSolving IK...")
        result = self._kinematics.solve(
            world=self._world,
            robot_id=self._robot_id,
            target_pose=target_pose,
            seed=self._current_joints,
            check_collision=True,
        )

        if result.is_success():
            print("IK SUCCESS!")
            print(f"  Position error: {result.position_error:.6f} m")
            print(f"  Orientation error: {result.orientation_error:.6f} rad")
            print(f"  Solution: {self._format_array(result.joint_positions)}")

            # Move to solution?
            move = input("\nMove robot to solution? (y/n): ").strip().lower()
            if move == "y":
                self._move_robot_to(result.joint_positions)
        else:
            print(f"IK FAILED: {result.status.name}")
            print(f"  Message: {result.message}")

    def _plan_path(self) -> None:
        """Plan path to goal configuration."""
        print("\nMotion Planner")
        print("-" * 40)

        print("Enter goal joint configuration (6 values) or 'ik' to solve IK first:")
        goal_str = input("Goal: ").strip()

        if goal_str.lower() == "ik":
            # Use IK to get goal
            self._solve_ik()
            return

        try:
            parts = [float(x) for x in goal_str.replace(",", " ").split()]
            if len(parts) != 6:
                print("Expected 6 values")
                return
            q_goal = np.array(parts)
        except ValueError:
            print("Invalid input")
            return

        # Check goal validity
        with self._world.scratch_context() as ctx:
            self._world.set_positions(ctx, self._robot_id, q_goal)
            goal_free = self._world.is_collision_free(ctx, self._robot_id)

        if not goal_free:
            print("WARNING: Goal configuration is in collision!")
            proceed = input("Plan anyway? (y/n): ").strip().lower()
            if proceed != "y":
                return

        # Plan
        print("\nPlanning path...")

        result = self._planner.plan_joint_path(
            world=self._world,
            robot_id=self._robot_id,
            q_start=self._current_joints,
            q_goal=q_goal,
            timeout=10.0,
        )

        if result.is_success():
            print("PLANNING SUCCESS!")
            print(f"  Waypoints: {len(result.path)}")
            print(f"  Planning time: {result.planning_time:.3f} s")
            print(f"  Path length: {result.path_length:.3f} rad")

            # Execute path?
            execute = input("\nExecute path? (y/n): ").strip().lower()
            if execute == "y":
                self._execute_path(result.path)
        else:
            print(f"PLANNING FAILED: {result.status.name}")
            print(f"  Message: {result.message}")

    def _execute_path(self, path: list[NDArray[np.float64]], duration: float = 5.0) -> None:
        """Execute a planned path with visualization.

        Args:
            path: Path to execute
            duration: Total animation duration in seconds (default 5s for clear visualization)
        """
        print(f"Executing path ({duration:.1f}s animation)...")

        # Interpolate for smooth visualization
        interpolated = interpolate_path(path, resolution=0.02)
        num_frames = len(interpolated)

        # Calculate frame delay based on total duration
        dt = duration / max(num_frames - 1, 1)

        for i, waypoint in enumerate(interpolated):
            self._world.sync_from_joint_state(self._robot_id, waypoint)
            self._world.publish_to_meshcat()

            if i % max(num_frames // 5, 1) == 0:
                progress = (i / num_frames) * 100
                print(f"  Progress: {progress:.0f}%")

            time.sleep(dt)

        self._current_joints = path[-1].copy()
        print("Path execution complete!")

    # =========================================================================
    # Obstacle Management
    # =========================================================================

    def _add_obstacle(self) -> None:
        """Add an obstacle."""
        print("\nObstacle types:")
        print("  1. Box")
        print("  2. Sphere")
        print("  3. Cylinder")

        try:
            type_choice = input("Select type (1/2/3): ").strip()

            if type_choice == "1":
                obs_type = ObstacleType.BOX
                dims = self._get_box_dimensions()
            elif type_choice == "2":
                obs_type = ObstacleType.SPHERE
                dims = self._get_sphere_dimensions()
            elif type_choice == "3":
                obs_type = ObstacleType.CYLINDER
                dims = self._get_cylinder_dimensions()
            else:
                print("Invalid choice.")
                return

            pos = self._get_position("Position")
            color = self._get_random_color()

            self._obstacle_counter += 1
            name = f"obstacle_{self._obstacle_counter}"

            pose = np.eye(4)
            pose[:3, 3] = pos

            obstacle = Obstacle(
                name=name,
                obstacle_type=obs_type,
                pose=pose,
                dimensions=dims,
                color=color,
            )

            self._world.add_obstacle(obstacle)
            self._obstacles[name] = obstacle
            self._world.publish_to_meshcat()

            print(f"Added obstacle: {name}")
            print(f"  Type: {obs_type.name}")
            print(f"  Position: {self._format_array(pos)}")

        except ValueError as e:
            print(f"Invalid input: {e}")

    def _move_obstacle(self) -> None:
        """Move an obstacle."""
        if not self._obstacles:
            print("No obstacles to move.")
            return

        self._list_obstacles()
        name = input("Obstacle name: ").strip()

        if name not in self._obstacles:
            print(f"Obstacle '{name}' not found.")
            return

        pos = self._get_position("New position")
        new_pose = np.eye(4)
        new_pose[:3, 3] = pos

        self._world.update_obstacle_pose(name, new_pose)
        self._obstacles[name].pose = new_pose
        self._world.publish_to_meshcat()

        print(f"Moved '{name}' to {self._format_array(pos)}")

    def _remove_obstacle(self) -> None:
        """Remove an obstacle."""
        if not self._obstacles:
            print("No obstacles to remove.")
            return

        self._list_obstacles()
        name = input("Obstacle name: ").strip()

        if name not in self._obstacles:
            print(f"Obstacle '{name}' not found.")
            return

        self._world.remove_obstacle(name)
        del self._obstacles[name]
        self._world.publish_to_meshcat()
        print(f"Removed: {name}")

    def _list_obstacles(self) -> None:
        """List all obstacles."""
        if not self._obstacles:
            print("No obstacles.")
            return

        print(f"\nObstacles ({len(self._obstacles)}):")
        for name, obs in self._obstacles.items():
            pos = obs.pose[:3, 3]
            print(f"  {name}: {obs.obstacle_type.name} at {self._format_array(pos)}")

    def _clear_obstacles(self) -> None:
        """Clear all obstacles."""
        if not self._obstacles:
            print("No obstacles to clear.")
            return

        self._world.clear_obstacles()
        self._obstacles.clear()
        self._world.publish_to_meshcat()
        print("Cleared all obstacles.")

    # =========================================================================
    # Helpers
    # =========================================================================

    def _get_box_dimensions(self) -> tuple[float, float, float]:
        """Get box dimensions."""
        print("Dimensions (w, h, d) [default: 0.1 0.1 0.1]:")
        dims_str = input("  Dims: ").strip()
        if not dims_str:
            return (0.1, 0.1, 0.1)
        parts = [float(x) for x in dims_str.replace(",", " ").split()]
        if len(parts) == 1:
            return (parts[0], parts[0], parts[0])
        return (parts[0], parts[1], parts[2])

    def _get_sphere_dimensions(self) -> tuple[float]:
        """Get sphere dimensions."""
        print("Radius [default: 0.05]:")
        r_str = input("  Radius: ").strip()
        if not r_str:
            return (0.05,)
        return (float(r_str),)

    def _get_cylinder_dimensions(self) -> tuple[float, float]:
        """Get cylinder dimensions."""
        print("Dimensions (radius, height) [default: 0.05 0.1]:")
        dims_str = input("  Dims: ").strip()
        if not dims_str:
            return (0.05, 0.1)
        parts = [float(x) for x in dims_str.replace(",", " ").split()]
        return (parts[0], parts[1])

    def _get_position(self, prompt: str = "Position") -> np.ndarray:
        """Get position from user."""
        print(f"{prompt} (x, y, z) [default: 0.3 0 0.1]:")
        pos_str = input(f"  {prompt}: ").strip()
        if not pos_str:
            return np.array([0.3, 0.0, 0.1])
        parts = [float(x) for x in pos_str.replace(",", " ").split()]
        return np.array(parts)

    def _get_random_color(self) -> tuple[float, float, float, float]:
        """Get a random color."""
        hue = np.random.random()
        # HSV to RGB
        h = hue * 6
        c = 0.7
        x = c * (1 - abs(h % 2 - 1))
        if h < 1:
            r, g, b = c, x, 0
        elif h < 2:
            r, g, b = x, c, 0
        elif h < 3:
            r, g, b = 0, c, x
        elif h < 4:
            r, g, b = 0, x, c
        elif h < 5:
            r, g, b = x, 0, c
        else:
            r, g, b = c, 0, x
        return (r + 0.3, g + 0.3, b + 0.3, 0.8)

    def _format_array(self, arr: np.ndarray | None) -> str:
        """Format array for display."""
        if arr is None:
            return "None"
        return "[" + ", ".join(f"{x:.4f}" for x in arr) + "]"


def main():
    """Run the planning tester."""
    import argparse

    parser = argparse.ArgumentParser(
        description="Interactive planning tester for manipulation stack",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Supported robots:
  piper   - Agilex Piper 6-DOF arm (default)
  xarm6   - UFactory xArm 6-DOF arm
  xarm7   - UFactory xArm 7-DOF arm

Examples:
  python planning_tester.py --robot piper
  python planning_tester.py --robot xarm6
        """,
    )
    parser.add_argument(
        "--robot",
        type=str,
        default="piper",
        choices=list(PlanningTester.ROBOT_CONFIGS.keys()),
        help="Robot type to use (default: piper)",
    )

    args = parser.parse_args()

    tester = PlanningTester(robot_type=args.robot)
    if tester.setup():
        tester.run()
    else:
        print("Setup failed.")
        sys.exit(1)


if __name__ == "__main__":
    main()

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
Interactive Obstacle Tester

A lightweight CLI tool for managing obstacles that syncs with the planning tester.
Obstacles are persisted to a JSON file that the planning tester watches.

Usage:
    # Headless mode (for use alongside planning tester)
    python obstacle_tester.py

    # Standalone mode with own visualization
    python obstacle_tester.py --viz

Commands:
    add     - Add a new obstacle (box, sphere, cylinder)
    move    - Move an existing obstacle
    remove  - Remove an obstacle
    list    - List all obstacles
    clear   - Remove all obstacles
    quit    - Exit the tester

Multi-Process Usage:
    Terminal 1: python planning_tester.py     # Owns robot + visualization
    Terminal 2: python obstacle_tester.py     # Manages obstacles (auto-synced)
"""

from __future__ import annotations

import sys
from typing import TYPE_CHECKING

import numpy as np

from dimos.manipulation.planning.examples.obstacle_store import ObstacleStore
from dimos.manipulation.planning.spec import Obstacle, ObstacleType

if TYPE_CHECKING:
    from pathlib import Path

    from dimos.manipulation.planning.spec import WorldSpec


class ObstacleTester:
    """Interactive obstacle management tool with file-based persistence.

    This tester manages obstacles through a JSON store that can be watched
    by the planning tester for automatic synchronization.
    """

    def __init__(self, enable_viz: bool = False, store_path: Path | str | None = None):
        """Initialize the tester.

        Args:
            enable_viz: Whether to enable standalone Meshcat visualization
            store_path: Custom path for obstacle store file
        """
        self._store = ObstacleStore(store_path)
        self._obstacle_counter = len(self._store)
        self._enable_viz = enable_viz
        self._world: WorldSpec | None = None

    def setup(self) -> None:
        """Set up the tester."""
        print("=" * 60)
        print("Obstacle Tester - Obstacle Management CLI")
        print("=" * 60)
        print(f"\nStore file: {self._store.path}")

        if self._enable_viz:
            self._setup_visualization()
        else:
            print("\nRunning in headless mode (use --viz for standalone visualization)")
            print("Obstacles will sync to planning tester automatically.\n")

        # Show existing obstacles
        existing = self._store.get_obstacles()
        if existing:
            print(f"Loaded {len(existing)} existing obstacles from store.")
            self._obstacle_counter = len(existing)

    def _setup_visualization(self) -> None:
        """Set up standalone visualization."""
        from dimos.manipulation.planning.factory import create_world

        print("\nSetting up standalone visualization...")
        self._world = create_world(backend="drake", enable_viz=True)
        self._world.finalize()

        if hasattr(self._world, "get_meshcat_url"):
            url = self._world.get_meshcat_url()
            print(f"Meshcat URL: {url}")
            print("Open this URL in your browser.\n")

        # Load existing obstacles into visualization
        for obs_data in self._store.get_obstacles().values():
            obstacle = obs_data.to_obstacle()
            self._world.add_obstacle(obstacle)
        if hasattr(self._world, "publish_to_meshcat"):
            self._world.publish_to_meshcat()

    def run(self) -> None:
        """Run the interactive loop."""
        self._print_help()

        while True:
            try:
                cmd = input("\n[obstacles]> ").strip().lower()

                if cmd == "quit" or cmd == "q":
                    print("Exiting...")
                    break
                elif cmd == "help" or cmd == "h":
                    self._print_help()
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
                elif cmd == "reload":
                    self._reload_store()
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
        print("\nCommands:")
        print("  add (a)     - Add a new obstacle")
        print("  move (m)    - Move an existing obstacle")
        print("  remove (r)  - Remove an obstacle")
        print("  list (l)    - List all obstacles")
        print("  clear       - Remove all obstacles")
        print("  reload      - Reload obstacles from file")
        print("  help (h)    - Show this help")
        print("  quit (q)    - Exit")

    def _add_obstacle(self) -> None:
        """Add a new obstacle interactively."""
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

            # Get position
            pos = self._get_position("Position")

            # Get color
            color = self._get_color()

            # Generate name
            self._obstacle_counter += 1
            name = f"obstacle_{self._obstacle_counter}"

            # Create pose matrix
            pose = np.eye(4)
            pose[:3, 3] = pos

            # Create obstacle
            obstacle = Obstacle(
                name=name,
                obstacle_type=obs_type,
                pose=pose,
                dimensions=dims,
                color=color,
            )

            # Add to store and save
            self._store.add_obstacle(obstacle)
            self._store.save()

            # Update visualization if enabled
            if self._world is not None:
                self._world.add_obstacle(obstacle)
                if hasattr(self._world, "publish_to_meshcat"):
                    self._world.publish_to_meshcat()

            print(f"\nAdded obstacle: {name}")
            print(f"  Type: {obs_type.name}")
            print(f"  Position: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
            print(f"  Dimensions: {dims}")
            print("  (Synced to store)")

        except ValueError as e:
            print(f"Invalid input: {e}")

    def _get_box_dimensions(self) -> tuple[float, float, float]:
        """Get box dimensions from user."""
        print("Enter dimensions (width, height, depth) or press Enter for default [0.1, 0.1, 0.1]:")
        dims_str = input("  Dimensions: ").strip()

        if not dims_str:
            return (0.1, 0.1, 0.1)

        parts = [float(x.strip()) for x in dims_str.replace(",", " ").split()]
        if len(parts) == 1:
            return (parts[0], parts[0], parts[0])
        elif len(parts) == 3:
            return (parts[0], parts[1], parts[2])
        else:
            raise ValueError("Expected 1 or 3 values for box dimensions")

    def _get_sphere_dimensions(self) -> tuple[float]:
        """Get sphere dimensions from user."""
        print("Enter radius or press Enter for default [0.05]:")
        radius_str = input("  Radius: ").strip()

        if not radius_str:
            return (0.05,)

        return (float(radius_str),)

    def _get_cylinder_dimensions(self) -> tuple[float, float]:
        """Get cylinder dimensions from user."""
        print("Enter dimensions (radius, height) or press Enter for default [0.05, 0.1]:")
        dims_str = input("  Dimensions: ").strip()

        if not dims_str:
            return (0.05, 0.1)

        parts = [float(x.strip()) for x in dims_str.replace(",", " ").split()]
        if len(parts) == 2:
            return (parts[0], parts[1])
        else:
            raise ValueError("Expected 2 values for cylinder dimensions (radius, height)")

    def _get_position(self, prompt: str = "Position") -> np.ndarray:
        """Get position from user."""
        print(f"Enter {prompt} (x, y, z) or press Enter for default [0.3, 0, 0.1]:")
        pos_str = input(f"  {prompt}: ").strip()

        if not pos_str:
            return np.array([0.3, 0.0, 0.1])

        parts = [float(x.strip()) for x in pos_str.replace(",", " ").split()]
        if len(parts) == 3:
            return np.array(parts)
        else:
            raise ValueError("Expected 3 values for position (x, y, z)")

    def _get_color(self) -> tuple[float, float, float, float]:
        """Get color from user."""
        print("Enter color (r, g, b, a) or press Enter for random:")
        color_str = input("  Color: ").strip()

        if not color_str:
            # Random color with good saturation
            hue = np.random.random()
            r, g, b = self._hsv_to_rgb(hue, 0.7, 0.9)
            return (r, g, b, 0.8)

        parts = [float(x.strip()) for x in color_str.replace(",", " ").split()]
        if len(parts) == 3:
            return (parts[0], parts[1], parts[2], 0.8)
        elif len(parts) == 4:
            return (parts[0], parts[1], parts[2], parts[3])
        else:
            raise ValueError("Expected 3 or 4 values for color (r, g, b[, a])")

    def _hsv_to_rgb(self, h: float, s: float, v: float) -> tuple[float, float, float]:
        """Convert HSV to RGB."""
        if s == 0.0:
            return (v, v, v)
        i = int(h * 6.0)
        f = (h * 6.0) - i
        p = v * (1.0 - s)
        q = v * (1.0 - s * f)
        t = v * (1.0 - s * (1.0 - f))
        i = i % 6
        if i == 0:
            return (v, t, p)
        if i == 1:
            return (q, v, p)
        if i == 2:
            return (p, v, t)
        if i == 3:
            return (p, q, v)
        if i == 4:
            return (t, p, v)
        return (v, p, q)

    def _move_obstacle(self) -> None:
        """Move an existing obstacle."""
        obstacles = self._store.get_obstacles()
        if not obstacles:
            print("No obstacles to move. Add one first.")
            return

        self._list_obstacles()

        name = input("Enter obstacle name to move: ").strip()
        if name not in obstacles:
            print(f"Obstacle '{name}' not found.")
            return

        print("\nMove options:")
        print("  1. Set absolute position")
        print("  2. Move relative (delta)")

        choice = input("Select (1/2): ").strip()

        try:
            obs_data = obstacles[name]
            current_pos = np.array(obs_data.pose)[:3, 3].copy()

            if choice == "1":
                new_pos = self._get_position("New position")
            elif choice == "2":
                delta = self._get_position("Delta (dx, dy, dz)")
                new_pos = current_pos + delta
            else:
                print("Invalid choice.")
                return

            # Update pose
            new_pose = np.array(obs_data.pose)
            new_pose[:3, 3] = new_pos

            # Update in store and save
            self._store.update_pose(name, new_pose)
            self._store.save()

            # Update visualization if enabled
            if self._world is not None:
                self._world.update_obstacle_pose(name, new_pose)
                if hasattr(self._world, "publish_to_meshcat"):
                    self._world.publish_to_meshcat()

            print(
                f"Moved '{name}' from [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]"
            )
            print(f"                  to [{new_pos[0]:.3f}, {new_pos[1]:.3f}, {new_pos[2]:.3f}]")
            print("  (Synced to store)")

        except ValueError as e:
            print(f"Invalid input: {e}")

    def _remove_obstacle(self) -> None:
        """Remove an obstacle."""
        obstacles = self._store.get_obstacles()
        if not obstacles:
            print("No obstacles to remove.")
            return

        self._list_obstacles()

        name = input("Enter obstacle name to remove: ").strip()
        if name not in obstacles:
            print(f"Obstacle '{name}' not found.")
            return

        # Remove from store and save
        self._store.remove_obstacle(name)
        self._store.save()

        # Update visualization if enabled
        if self._world is not None:
            self._world.remove_obstacle(name)
            if hasattr(self._world, "publish_to_meshcat"):
                self._world.publish_to_meshcat()

        print(f"Removed obstacle: {name}")
        print("  (Synced to store)")

    def _clear_obstacles(self) -> None:
        """Remove all obstacles."""
        obstacles = self._store.get_obstacles()
        if not obstacles:
            print("No obstacles to clear.")
            return

        confirm = input(f"Remove all {len(obstacles)} obstacles? (y/n): ").strip().lower()
        if confirm != "y":
            print("Cancelled.")
            return

        # Clear store and save
        self._store.clear()
        self._store.save()

        # Update visualization if enabled
        if self._world is not None:
            self._world.clear_obstacles()
            if hasattr(self._world, "publish_to_meshcat"):
                self._world.publish_to_meshcat()

        print("Cleared all obstacles.")
        print("  (Synced to store)")

    def _list_obstacles(self) -> None:
        """List all obstacles."""
        obstacles = self._store.get_obstacles()
        if not obstacles:
            print("No obstacles in store.")
            return

        print(f"\nObstacles ({len(obstacles)}):")
        for name, obs in obstacles.items():
            pos = np.array(obs.pose)[:3, 3]
            print(f"  {name}:")
            print(f"    Type: {obs.obstacle_type}")
            print(f"    Position: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
            print(f"    Dimensions: {obs.dimensions}")

    def _reload_store(self) -> None:
        """Reload obstacles from file."""
        self._store.load()
        obstacles = self._store.get_obstacles()
        print(f"Reloaded {len(obstacles)} obstacles from store.")

        # Sync to visualization if enabled
        if self._world is not None:
            self._world.clear_obstacles()
            for obs_data in obstacles.values():
                obstacle = obs_data.to_obstacle()
                self._world.add_obstacle(obstacle)
            if hasattr(self._world, "publish_to_meshcat"):
                self._world.publish_to_meshcat()


def main():
    """Run the obstacle tester."""
    enable_viz = "--viz" in sys.argv or "-v" in sys.argv

    tester = ObstacleTester(enable_viz=enable_viz)
    tester.setup()
    tester.run()


if __name__ == "__main__":
    main()

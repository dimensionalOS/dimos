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
Manipulation Client - IPython interface for ManipulationModule

Usage:
    # Start coordinator and planner first:
    dimos run coordinator-mock
    dimos run xarm7-planner-coordinator

    # Run interactive client:
    python -m dimos.manipulation.planning.examples.manipulation_client

Commands (call directly, no prefix needed):
    joints()                # Get current joint positions
    ee()                    # Get end-effector pose
    state()                 # Get manipulation state
    url()                   # Get Meshcat visualization URL

    plan([0.1, ...])        # Plan to joint config
    plan_pose(x, y, z)      # Plan to cartesian pose
    preview()               # Preview path in Meshcat
    execute()               # Execute via coordinator

    box("name", x, y, z, w, h, d)   # Add box obstacle
    sphere("name", x, y, z, r)      # Add sphere obstacle
    cylinder("name", x, y, z, r, l) # Add cylinder obstacle
    remove("obstacle_id")           # Remove obstacle
"""

from __future__ import annotations

from typing import Any, cast

from dimos.msgs.geometry_msgs import Pose, Quaternion, Vector3
from dimos.protocol.rpc import LCMRPC


class ManipulationClient:
    """RPC client for ManipulationModule with IPython-friendly API."""

    def __init__(self) -> None:
        self._rpc = LCMRPC()
        self._rpc.start()
        self._module = "ManipulationModule"
        print("Connected to ManipulationModule via LCM RPC")

    def _call(self, method: str, *args: Any, **kwargs: Any) -> Any:
        """Call RPC method."""
        try:
            result, _ = self._rpc.call_sync(
                f"{self._module}/{method}", (list(args), kwargs), rpc_timeout=30.0
            )
            return result
        except TimeoutError:
            print(f"Timeout: {method}")
            return None
        except Exception as e:
            print(f"Error: {e}")
            return None

    # =========================================================================
    # Query Methods
    # =========================================================================

    def state(self) -> str:
        """Get manipulation state."""
        return cast("str", self._call("get_state"))

    def joints(self, robot_name: str | None = None) -> list[float] | None:
        """Get current joint positions.

        Args:
            robot_name: Robot to query (required if multiple robots configured)
        """
        return cast("list[float] | None", self._call("get_current_joints", robot_name))

    def ee(self, robot_name: str | None = None) -> Pose | None:
        """Get end-effector pose.

        Args:
            robot_name: Robot to query (required if multiple robots configured)
        """
        return cast("Pose | None", self._call("get_ee_pose", robot_name))

    def url(self) -> str | None:
        """Get Meshcat visualization URL."""
        return cast("str | None", self._call("get_visualization_url"))

    def robots(self) -> list[str]:
        """List configured robots."""
        return cast("list[str]", self._call("list_robots"))

    def info(self, robot_name: str | None = None) -> dict[str, Any] | None:
        """Get robot info."""
        return cast("dict[str, Any] | None", self._call("get_robot_info", robot_name))

    # =========================================================================
    # Planning Methods
    # =========================================================================

    def plan(self, joints: list[float], robot_name: str | None = None) -> bool:
        """Plan to joint configuration.

        Args:
            joints: Target joint configuration
            robot_name: Robot to plan for (required if multiple robots configured)
        """
        print(
            f"Planning to: {[f'{j:.3f}' for j in joints]}"
            + (f" for {robot_name}" if robot_name else "")
        )
        return cast("bool", self._call("plan_to_joints", joints, robot_name=robot_name))

    def plan_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float | None = None,
        pitch: float | None = None,
        yaw: float | None = None,
        robot_name: str | None = None,
    ) -> bool:
        """Plan to cartesian pose. Uses current orientation if not specified.

        Args:
            x, y, z: Target position
            roll, pitch, yaw: Target orientation (uses current if not specified)
            robot_name: Robot to plan for (required if multiple robots configured)
        """
        # Get current orientation if not provided
        if roll is None or pitch is None or yaw is None:
            ee = self.ee(robot_name)
            if ee is None:
                print("Cannot get current orientation - specify roll, pitch, yaw explicitly")
                return False
            roll = roll if roll is not None else ee.roll
            pitch = pitch if pitch is not None else ee.pitch
            yaw = yaw if yaw is not None else ee.yaw

        print(
            f"Planning to: ({x:.3f}, {y:.3f}, {z:.3f}) rpy=({roll:.2f}, {pitch:.2f}, {yaw:.2f})"
            + (f" for {robot_name}" if robot_name else "")
        )
        pose = Pose(
            position=Vector3(x, y, z),
            orientation=Quaternion.from_euler(Vector3(roll, pitch, yaw)),
        )
        return cast("bool", self._call("plan_to_pose", pose, robot_name=robot_name))

    def preview(self, duration: float = 3.0, robot_name: str | None = None) -> bool:
        """Preview planned path in Meshcat.

        Args:
            duration: Animation duration in seconds
            robot_name: Robot to preview (required if multiple robots configured)
        """
        return cast("bool", self._call("preview_path", duration, robot_name=robot_name))

    def execute(self, robot_name: str | None = None) -> bool:
        """Execute planned trajectory via coordinator."""
        return cast("bool", self._call("execute", robot_name))

    def has_plan(self) -> bool:
        """Check if path is planned."""
        return cast("bool", self._call("has_planned_path"))

    def clear_plan(self) -> bool:
        """Clear planned path."""
        return cast("bool", self._call("clear_planned_path"))

    # =========================================================================
    # Obstacle Methods
    # =========================================================================

    def box(
        self,
        name: str,
        x: float,
        y: float,
        z: float,
        w: float,
        h: float,
        d: float,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
    ) -> str:
        """Add box obstacle."""
        pose = Pose(
            position=Vector3(x, y, z),
            orientation=Quaternion.from_euler(Vector3(roll, pitch, yaw)),
        )
        return cast("str", self._call("add_obstacle", name, pose, "box", [w, h, d]))

    def sphere(self, name: str, x: float, y: float, z: float, radius: float) -> str:
        """Add sphere obstacle."""
        pose = Pose(position=Vector3(x, y, z))
        return cast("str", self._call("add_obstacle", name, pose, "sphere", [radius]))

    def cylinder(
        self, name: str, x: float, y: float, z: float, radius: float, length: float
    ) -> str:
        """Add cylinder obstacle."""
        pose = Pose(position=Vector3(x, y, z))
        return cast("str", self._call("add_obstacle", name, pose, "cylinder", [radius, length]))

    def remove(self, obstacle_id: str) -> bool:
        """Remove obstacle."""
        return cast("bool", self._call("remove_obstacle", obstacle_id))

    # =========================================================================
    # Gripper Methods
    # =========================================================================

    def set_gripper(self, position: float, robot_name: str | None = None) -> bool:
        """Set gripper position in meters.

        Args:
            position: Gripper position in meters
            robot_name: Robot to control (required if multiple robots configured)
        """
        return cast("bool", self._call("set_gripper", position, robot_name=robot_name))

    def get_gripper(self, robot_name: str | None = None) -> float | None:
        """Get gripper position in meters.

        Args:
            robot_name: Robot to query (required if multiple robots configured)
        """
        return cast("float | None", self._call("get_gripper", robot_name=robot_name))

    def open_gripper(self, robot_name: str | None = None) -> bool:
        """Open gripper fully.

        Args:
            robot_name: Robot to control (required if multiple robots configured)
        """
        return cast("bool", self._call("open_gripper", robot_name=robot_name))

    def close_gripper(self, robot_name: str | None = None) -> bool:
        """Close gripper fully.

        Args:
            robot_name: Robot to control (required if multiple robots configured)
        """
        return cast("bool", self._call("close_gripper", robot_name=robot_name))

    # =========================================================================
    # Utility Methods
    # =========================================================================

    def collision(self, joints: list[float], robot_name: str | None = None) -> bool:
        """Check if joint config is collision-free.

        Args:
            joints: Joint configuration to check
            robot_name: Robot to check (required if multiple robots configured)
        """
        return cast("bool", self._call("is_collision_free", joints, robot_name))

    def reset(self) -> bool:
        """Reset to IDLE state."""
        return cast("bool", self._call("reset"))

    def cancel(self) -> bool:
        """Cancel current motion."""
        return cast("bool", self._call("cancel"))

    def status(self, robot_name: str | None = None) -> dict[str, object] | None:
        """Get trajectory execution status."""
        return cast("dict[str, object] | None", self._call("get_trajectory_status", robot_name))

    def stop(self) -> None:
        """Stop RPC client."""
        self._rpc.stop()

    def __repr__(self) -> str:
        return f"ManipulationClient(state={self.state()})"


def main() -> None:
    """Start IPython shell with ManipulationClient."""
    try:
        from IPython import embed
    except ImportError:
        print("IPython not installed. Run: pip install ipython")
        return

    c = ManipulationClient()

    # Expose methods directly in user namespace (no 'c.' prefix needed)
    user_ns = {
        "c": c,
        "client": c,
        # Query methods
        "state": c.state,
        "joints": c.joints,
        "ee": c.ee,
        "url": c.url,
        "robots": c.robots,
        "info": c.info,
        # Planning methods
        "plan": c.plan,
        "plan_pose": c.plan_pose,
        "preview": c.preview,
        "execute": c.execute,
        "has_plan": c.has_plan,
        "clear_plan": c.clear_plan,
        # Obstacle methods
        "box": c.box,
        "sphere": c.sphere,
        "cylinder": c.cylinder,
        "remove": c.remove,
        # Gripper methods
        "set_gripper": c.set_gripper,
        "get_gripper": c.get_gripper,
        "open_gripper": c.open_gripper,
        "close_gripper": c.close_gripper,
        # Utility methods
        "collision": c.collision,
        "reset": c.reset,
        "cancel": c.cancel,
        "status": c.status,
    }

    banner = """
Manipulation Client - IPython Interface
========================================

Commands (no prefix needed):
  joints()                # Get joint positions
  ee()                    # Get end-effector pose
  url()                   # Get Meshcat URL
  state()                 # Get manipulation state
  robots()                # List configured robots

Planning:
  plan([0.1, 0.2, ...])   # Plan to joint config
  plan_pose(0.4, 0, 0.3)  # Plan to cartesian pose (keeps orientation)
  plan_pose(0.4, 0, 0.3, roll=0, pitch=3.14, yaw=0)  # With orientation
  preview()               # Preview path in Meshcat
  execute()               # Execute via coordinator

Obstacles:
  box("name", x, y, z, width, height, depth)        # Add box
  box("table", 0.5, 0, -0.02, 1.0, 0.6, 0.04)       # Example: table
  sphere("name", x, y, z, radius)                   # Add sphere
  cylinder("name", x, y, z, radius, length)         # Add cylinder
  remove("obstacle_id")                             # Remove obstacle

Gripper:
  open_gripper()              # Open gripper fully
  close_gripper()             # Close gripper fully
  set_gripper(0.05)           # Set gripper position (meters)
  get_gripper()               # Get gripper position (meters)

Utility:
  collision([0.1, ...])   # Check if config is collision-free
  reset()                 # Reset to IDLE state
  cancel()                # Cancel current motion

Type help(command) for details, e.g. help(box)
"""
    print(banner)

    try:
        embed(user_ns=user_ns, colors="neutral")  # type: ignore[no-untyped-call]
    finally:
        c.stop()


if __name__ == "__main__":
    main()

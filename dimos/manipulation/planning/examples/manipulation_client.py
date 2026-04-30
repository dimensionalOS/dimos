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
RPC client for interacting with a running ManipulationModule.

Usage:
    # Start a manipulation blueprint in another terminal first:
    #   dimos run xarm7-planner-coordinator
    #
    # Then run this client:
    python -i -m dimos.manipulation.planning.examples.manipulation_client

Available functions:
    joints()              Get current joint positions
    ee()                  Get end-effector pose
    state()               Get module state (IDLE, PLANNING, EXECUTING, ...)
    plan(joints)          Plan to joint configuration, e.g. plan([0.1]*7)
    plan_pose(x,y,z)      Plan to Cartesian pose
    preview(duration)     Preview planned path in Meshcat
    execute()             Execute planned trajectory via coordinator
    home()                Move to home position
    url()                 Get Meshcat visualization URL
    robots()              List configured robots
    info(robot)           Get robot config details
    gripper(pos)          Set gripper position (0.0=closed, 0.85=open)
    add_box(name,x,y,z)   Add box obstacle
    add_sphere(name,x,y,z) Add sphere obstacle
    add_cylinder(name,x,y,z) Add cylinder obstacle
    remove(id)            Remove obstacle by ID
    collision_free(joints) Check if config is collision-free
"""

# mypy: disable-error-code=no-any-return
from __future__ import annotations

from typing import Any

from dimos.core.rpc_client import RPCClient
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3

_client = RPCClient(None, ManipulationModule)


def joints(robot_name: str | None = None) -> list[float] | None:
    """Get current joint positions."""
    return _client.get_current_joints(robot_name)


def ee(robot_name: str | None = None) -> Pose | None:
    """Get end-effector pose."""
    return _client.get_ee_pose(robot_name)


def state() -> str:
    """Get module state."""
    return _client.get_state()


def plan(target_joints: list[float], robot_name: str | None = None) -> bool:
    """Plan to joint configuration. e.g. plan([0.1]*7)"""
    from dimos.msgs.sensor_msgs.JointState import JointState

    js = JointState(position=target_joints)
    return _client.plan_to_joints(js, robot_name)


def plan_pose(
    x: float,
    y: float,
    z: float,
    roll: float | None = None,
    pitch: float | None = None,
    yaw: float | None = None,
    robot_name: str | None = None,
) -> bool:
    """Plan to Cartesian pose. Preserves current orientation if rpy not given."""
    if roll is not None or pitch is not None or yaw is not None:
        orientation = Quaternion.from_euler(Vector3(x=roll or 0, y=pitch or 0, z=yaw or 0))
    else:
        # Preserve current EE orientation
        current = _client.get_ee_pose(robot_name)
        orientation = current.orientation if current else Quaternion(0, 0, 0, 1)
    target = Pose(position=Vector3(x=x, y=y, z=z), orientation=orientation)
    return _client.plan_to_pose(target, robot_name)


def preview(duration: float = 3.0, robot_name: str | None = None) -> bool:
    """Preview planned path in Meshcat."""
    return _client.preview_path(duration, robot_name)


def execute(robot_name: str | None = None) -> bool:
    """Execute planned trajectory via coordinator."""
    return _client.execute(robot_name)


def home(robot_name: str | None = None) -> bool:
    """Plan and execute move to home position."""
    from dimos.msgs.sensor_msgs.JointState import JointState

    home_joints = _client.get_robot_info(robot_name).get("home_joints", [0.0] * 7)
    success = _client.plan_to_joints(JointState(position=home_joints), robot_name)
    if success:
        return _client.execute(robot_name)
    return False


def url() -> str | None:
    """Get Meshcat visualization URL."""
    return _client.get_visualization_url()


def robots() -> list[str]:
    """List configured robots."""
    return _client.list_robots()


def info(robot_name: str | None = None) -> dict[str, Any] | None:
    """Get robot config details."""
    return _client.get_robot_info(robot_name)


def gripper(position: float, robot_name: str | None = None) -> str:
    """Set gripper position (0.0=closed, 0.85=open)."""
    return _client.set_gripper(position, robot_name)


def add_box(
    name: str, x: float, y: float, z: float, w: float = 0.05, h: float = 0.05, d: float = 0.05
) -> str | None:
    """Add a box obstacle. e.g. add_box("cube", 0.3, 0, 0.2)"""
    pose = Pose(position=Vector3(x=x, y=y, z=z), orientation=Quaternion(0, 0, 0, 1))
    return _client.add_obstacle(name, pose, "box", [w, h, d], None)


def add_sphere(name: str, x: float, y: float, z: float, radius: float = 0.05) -> str | None:
    """Add a sphere obstacle. e.g. add_sphere("ball", 0.3, 0, 0.2)"""
    pose = Pose(position=Vector3(x=x, y=y, z=z), orientation=Quaternion(0, 0, 0, 1))
    return _client.add_obstacle(name, pose, "sphere", [radius], None)


def add_cylinder(
    name: str, x: float, y: float, z: float, radius: float = 0.03, height: float = 0.1
) -> str | None:
    """Add a cylinder obstacle. e.g. add_cylinder("can", 0.3, 0, 0.2)"""
    pose = Pose(position=Vector3(x=x, y=y, z=z), orientation=Quaternion(0, 0, 0, 1))
    return _client.add_obstacle(name, pose, "cylinder", [radius, height], None)


def remove(obstacle_id: str) -> bool:
    """Remove an obstacle by ID (returned from add_*)."""
    return _client.remove_obstacle(obstacle_id)


def collision_free(target_joints: list[float], robot_name: str | None = None) -> bool:
    """Check if a joint configuration is collision-free."""
    return _client.is_collision_free(target_joints, robot_name)


def commands() -> None:
    """Print available functions and raw RPC methods."""
    print("=== Client Functions ===")
    for name, obj in sorted(globals().items()):
        if callable(obj) and not name.startswith("_") and obj.__module__ == __name__:
            doc = (obj.__doc__ or "").split("\n")[0]
            print(f"  {name:25s} {doc}")


def stop() -> None:
    """Stop the RPC client."""
    _client.stop_rpc_client()


if __name__ == "__main__":
    print("Manipulation RPC client ready.")
    print("Type commands() for available functions.")
    print("Try: joints(), plan([0.1]*7), preview(), execute()")

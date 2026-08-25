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

"""Small RPC client for a running :class:`ManipulationModule`.

Start ``dimos run xarm7-planner-coordinator``, then run this module with
``python -i -m dimos.manipulation.planning.examples.manipulation_client``.
"""

from __future__ import annotations

from typing import cast

from dimos.core.rpc_client import RPCClient
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_spec import (
    CommandResult,
    ExecutionResult,
    ManipulationSnapshot,
    MoveResult,
    PlanningGroupInfo,
    PlanResult,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState

_client = RPCClient(None, ManipulationModule)


def groups() -> tuple[PlanningGroupInfo, ...]:
    """List opaque planning-group IDs and capabilities."""
    return cast("tuple[PlanningGroupInfo, ...]", _client.list_planning_groups())


def state() -> ManipulationSnapshot:
    """Get one snapshot containing every planning group."""
    return cast("ManipulationSnapshot", _client.get_state())


def plan_joints(
    positions: list[float],
    planning_group: str | None = None,
    speed_scale: float | None = None,
) -> PlanResult:
    """Plan joint motion without executing it."""
    group = _select_group(planning_group)
    target = JointState(name=list(group.joint_names), position=positions)
    return cast("PlanResult", _client.plan_to_joints({group.id: target}, speed_scale))


def plan_pose(
    x: float,
    y: float,
    z: float,
    planning_group: str | None = None,
    speed_scale: float | None = None,
) -> PlanResult:
    """Plan an absolute world-frame pose while preserving orientation."""
    group = _select_group(planning_group, pose_capable=True)
    current = state().groups[group.id].end_effector_pose
    if current is None:
        raise RuntimeError(f"End-effector pose is unavailable for {group.id!r}")
    target = PoseStamped(
        frame_id="world",
        position=Vector3(x, y, z),
        orientation=current.orientation,
    )
    return cast("PlanResult", _client.plan_to_poses({group.id: target}, speed_scale))


def execute(blocking: bool = True, timeout: float | None = None) -> ExecutionResult:
    """Consume and execute the pending plan."""
    return cast("ExecutionResult", _client.execute(blocking, timeout))


def wait(timeout: float | None = None) -> ExecutionResult:
    """Wait for the current execution's terminal JTT status."""
    return cast("ExecutionResult", _client.wait_for_execution(timeout))


def move_linear(
    dx: float = 0.0,
    dy: float = 0.0,
    dz: float = 0.0,
    planning_group: str | None = None,
    check_collision: bool = False,
    blocking: bool = True,
) -> MoveResult:
    """Move an end effector by a world-frame translation."""
    return cast(
        "MoveResult",
        _client.move_linear(
            dx,
            dy,
            dz,
            planning_group,
            check_collision,
            None,
            blocking,
            None,
        ),
    )


def gripper(position: float, planning_group: str | None = None) -> CommandResult:
    """Set gripper opening in metres."""
    return cast("CommandResult", _client.set_gripper_position(position, planning_group))


def cancel() -> ExecutionResult:
    """Cancel planning or active execution."""
    return cast("ExecutionResult", _client.cancel())


def _select_group(
    planning_group: str | None,
    *,
    pose_capable: bool = False,
) -> PlanningGroupInfo:
    candidates = tuple(
        group
        for group in groups()
        if (planning_group is None or group.id == planning_group)
        and (not pose_capable or group.tip_frame is not None)
    )
    if len(candidates) != 1:
        raise ValueError("Planning group is missing or ambiguous")
    return candidates[0]


def add_box(
    name: str,
    x: float,
    y: float,
    z: float,
    w: float = 0.05,
    h: float = 0.05,
    d: float = 0.05,
) -> str | None:
    """Add a box obstacle."""
    pose = Pose(position=Vector3(x=x, y=y, z=z), orientation=Quaternion(0, 0, 0, 1))
    return cast("str | None", _client.add_obstacle(name, pose, "box", [w, h, d], None))


def update_obstacle(
    name: str,
    pose: Pose,
    shape: str,
    dimensions: list[float] | None = None,
    mesh_path: str | None = None,
    color: list[float] | None = None,
) -> bool:
    """Replace a complete obstacle."""
    return cast("bool", _client.update_obstacle(name, pose, shape, dimensions, mesh_path, color))


def update_box(
    name: str,
    x: float,
    y: float,
    z: float,
    w: float,
    h: float,
    d: float,
    color: list[float] | None = None,
) -> bool:
    """Replace a complete box obstacle."""
    pose = Pose(position=Vector3(x=x, y=y, z=z), orientation=Quaternion(0, 0, 0, 1))
    return update_obstacle(name, pose, "box", [w, h, d], color=color)


def update_sphere(
    name: str,
    x: float,
    y: float,
    z: float,
    radius: float,
    color: list[float] | None = None,
) -> bool:
    """Replace a complete sphere obstacle."""
    pose = Pose(position=Vector3(x=x, y=y, z=z), orientation=Quaternion(0, 0, 0, 1))
    return update_obstacle(name, pose, "sphere", [radius], color=color)


def update_cylinder(
    name: str,
    x: float,
    y: float,
    z: float,
    radius: float,
    height: float,
    color: list[float] | None = None,
) -> bool:
    """Replace a complete cylinder obstacle."""
    pose = Pose(position=Vector3(x=x, y=y, z=z), orientation=Quaternion(0, 0, 0, 1))
    return update_obstacle(name, pose, "cylinder", [radius, height], color=color)


def update_pose(
    name: str,
    x: float,
    y: float,
    z: float,
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 0.0,
) -> bool:
    """Move an obstacle without changing its geometry."""
    pose = Pose(
        position=Vector3(x=x, y=y, z=z),
        orientation=Quaternion.from_euler(Vector3(x=roll, y=pitch, z=yaw)),
    )
    return cast("bool", _client.update_obstacle_pose(name, pose))


def remove(obstacle_id: str) -> bool:
    """Remove an obstacle by ID."""
    return cast("bool", _client.remove_obstacle(obstacle_id))


def stop() -> None:
    """Stop the RPC client."""
    _client.stop_rpc_client()


if __name__ == "__main__":
    print("Manipulation RPC client ready: groups(), state(), plan_joints(), execute()")

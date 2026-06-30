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

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from typing import TYPE_CHECKING, cast

from dimos.manipulation.planning.groups.models import PlanningGroup
from dimos.manipulation.planning.spec.models import PlanningGroupID, RobotName
from dimos.manipulation.visualization.types import TargetSetEvaluation
from dimos.manipulation.visualization.viser.state import FeasibilityStatus
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState

if TYPE_CHECKING:
    from dimos.manipulation.manipulation_module import ManipulationModule
    from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor


def copy_joint_state(joint_state: JointState | None) -> JointState | None:
    return None if joint_state is None else JointState(joint_state)


def get_current_joint_state(
    world_monitor: WorldMonitor,
    manipulation_module: ManipulationModule,
    robot_name: RobotName,
) -> JointState | None:
    current = manipulation_module.get_current_joint_state(robot_name)
    if current is None:
        robot_id = manipulation_module.robot_id_for_name(robot_name)
        if robot_id is not None:
            current = world_monitor.get_current_joint_state(robot_id)
    return copy_joint_state(current)


def is_state_stale(
    world_monitor: WorldMonitor,
    manipulation_module: ManipulationModule,
    robot_name: RobotName,
    max_age: float = 1.0,
) -> bool:
    robot_id = manipulation_module.robot_id_for_name(robot_name)
    return True if robot_id is None else world_monitor.is_state_stale(robot_id, max_age)


def get_ee_pose(
    world_monitor: WorldMonitor,
    manipulation_module: ManipulationModule,
    groups: Sequence[PlanningGroup],
    robot_name: RobotName,
    joint_state: JointState | None = None,
) -> Pose | None:
    group_id = primary_pose_group_id(groups, robot_name)
    if group_id is None:
        return None
    copied = copy_joint_state(joint_state)
    fk = manipulation_module.forward_kinematics(group_id, copied)
    if fk.pose is not None:
        return cast("Pose", fk.pose)
    robot_id = manipulation_module.robot_id_for_name(robot_name)
    if robot_id is None:
        return None
    get_world_ee_pose = getattr(world_monitor, "get_ee_pose", None)
    if callable(get_world_ee_pose):
        return cast("Pose | None", get_world_ee_pose(robot_id, copy_joint_state(joint_state)))
    return None


def evaluate_joint_target_set(
    manipulation_module: ManipulationModule,
    joint_targets: Mapping[PlanningGroupID, JointState],
) -> TargetSetEvaluation:
    if not joint_targets:
        return {"success": False, "status": "INVALID", "message": "No joint target"}
    names: list[str] = []
    positions: list[float] = []
    for target in joint_targets.values():
        copied = copy_joint_state(target)
        if copied is None:
            continue
        names.extend(str(name) for name in copied.name)
        positions.extend(float(position) for position in copied.position)
    return evaluate_global_target_set(
        manipulation_module,
        tuple(joint_targets),
        JointState({"name": names, "position": positions}),
    )


def evaluate_pose_target_set(
    manipulation_module: ManipulationModule,
    pose_targets: Mapping[PlanningGroupID, Pose],
    auxiliary_groups: Sequence[PlanningGroupID] = (),
    seed: JointState | None = None,
    check_collision: bool = True,
) -> TargetSetEvaluation:
    if not pose_targets:
        return {"success": False, "status": "INVALID", "message": "No pose target"}
    stamped_targets = {group_id: stamped_pose(pose) for group_id, pose in pose_targets.items()}
    group_ids = tuple(dict.fromkeys((*stamped_targets.keys(), *auxiliary_groups)))
    ik = manipulation_module.inverse_kinematics(
        pose_targets=stamped_targets,
        auxiliary_group_ids=auxiliary_groups,
        seed=copy_joint_state(seed),
    )
    if not ik.is_success() or ik.joint_state is None:
        return {
            "success": False,
            "status": ik.status.name,
            "message": ik.message,
            "collision_free": False,
            "group_ids": group_ids,
            "target_joints": None,
            "position_error": ik.position_error,
            "orientation_error": ik.orientation_error,
        }
    return evaluate_global_target_set(
        manipulation_module,
        group_ids,
        ik.joint_state,
        status=ik.status.name,
        message=ik.message,
        position_error=ik.position_error,
        orientation_error=ik.orientation_error,
        check_collision=check_collision,
    )


def evaluate_global_target_set(
    manipulation_module: ManipulationModule,
    group_ids: tuple[PlanningGroupID, ...],
    target_joints: JointState,
    *,
    status: str = "FEASIBLE",
    message: str = "Target is collision-free",
    position_error: float = 0.0,
    orientation_error: float = 0.0,
    check_collision: bool = True,
) -> TargetSetEvaluation:
    target = copy_joint_state(target_joints)
    if target is None:
        return {"success": False, "status": "INVALID", "message": "No target joints"}
    collision = manipulation_module.check_collision(target) if check_collision else None
    collision_free = True if collision is None else collision.collision_free is True
    diagnostics = {
        group_id: "Target collision check skipped" if collision is None else collision.message
        for group_id in group_ids
    }
    result_message = message
    if collision is None:
        result_message = "Target collision check skipped"
    elif not collision_free:
        result_message = collision.message
    group_poses: dict[PlanningGroupID, Pose | None] = {}
    for group_id in group_ids:
        fk = manipulation_module.forward_kinematics(group_id, target)
        if fk.pose is not None:
            group_poses[group_id] = cast("Pose", fk.pose)
        elif fk.status != "INVALID":
            group_poses[group_id] = None
    return {
        "success": collision_free,
        "status": status
        if collision_free
        else collision.status
        if collision is not None
        else "COLLISION",
        "message": result_message,
        "collision_free": collision_free,
        "group_ids": group_ids,
        "target_joints": target,
        "group_diagnostics": diagnostics,
        "group_poses": group_poses,
        "position_error": position_error,
        "orientation_error": orientation_error,
    }


def primary_pose_group_id(
    groups: Sequence[PlanningGroup], robot_name: RobotName
) -> PlanningGroupID | None:
    for group in groups:
        if group.robot_name == robot_name and group.has_pose_target:
            return group.id
    return None


def stamped_pose(pose: Pose | PoseStamped) -> PoseStamped:
    if isinstance(pose, PoseStamped):
        return pose
    return PoseStamped(frame_id="world", position=pose.position, orientation=pose.orientation)


def joint_values_by_name(robot_name: str, joint_state: JointState | None) -> dict[str, float]:
    if joint_state is None:
        return {}
    values: dict[str, float] = {}
    for name, position in zip(joint_state.name, joint_state.position, strict=False):
        name_str = str(name)
        position_float = float(position)
        values[name_str] = position_float
        if "/" in name_str:
            values[name_str.rsplit("/", 1)[1]] = position_float
        else:
            values[f"{robot_name}/{name_str}"] = position_float
    return values


def pose_from_transform_values(position: Sequence[float], wxyz: Sequence[float]) -> Pose:
    px, py, pz = (float(value) for value in position)
    qw, qx, qy, qz = (float(value) for value in wxyz)
    return Pose({"position": [px, py, pz], "orientation": [qx, qy, qz, qw]})


def feasibility_status(
    status: str,
    success: bool,
    collision_free: bool,
) -> FeasibilityStatus:
    normalized = status.upper()
    if success and collision_free:
        return FeasibilityStatus.FEASIBLE
    if normalized in {"COLLISION", "COLLISION_AT_START", "COLLISION_AT_GOAL"}:
        return FeasibilityStatus.COLLISION
    if normalized in {"NO_SOLUTION", "SINGULARITY", "JOINT_LIMITS", "TIMEOUT"}:
        return FeasibilityStatus.IK_FAILED
    return FeasibilityStatus.INVALID


def update_target_visual_state(
    scene: object,
    groups: Mapping[PlanningGroupID, PlanningGroup],
    selected_group_ids: Sequence[PlanningGroupID],
    robot_id_for_name: Callable[[RobotName], object | None],
    feasible: bool,
) -> None:
    set_visual_state = getattr(scene, "set_target_visual_state", None)
    if not callable(set_visual_state):
        return
    updated: set[str] = set()
    for group_id in selected_group_ids:
        group_id_str = str(group_id)
        if group_id_str not in updated:
            set_visual_state(group_id_str, feasible)
            updated.add(group_id_str)
        group = groups.get(group_id_str)
        if group is None:
            continue
        robot_id = robot_id_for_name(str(group.robot_name))
        robot_id_str = None if robot_id is None else str(robot_id)
        if robot_id_str is None or robot_id_str in updated:
            continue
        set_visual_state(robot_id_str, feasible)
        updated.add(robot_id_str)

# Copyright 2026 Dimensional Inc.
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

"""Group-aware, visualization-local panel operations.

This boundary deliberately composes complete robot states before talking to the
public world monitor.  It is not a planner API.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import TYPE_CHECKING, cast

from dimos.manipulation.planning.groups.models import PlanningGroup
from dimos.manipulation.planning.spec.models import PlanningGroupID, RobotName
from dimos.manipulation.visualization.types import RobotInfo, TargetSetEvaluation
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState

if TYPE_CHECKING:
    from dimos.manipulation.manipulation_module import ManipulationModule
    from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor
    from dimos.manipulation.planning.spec.config import RobotModelConfig
    from dimos.manipulation.planning.spec.models import WorldRobotID


def copy_joint_state(state: JointState | None) -> JointState | None:
    return None if state is None else JointState(state)


def group_display_name(group: PlanningGroup) -> str:
    """Return the source panel's concise group button label."""
    return (
        str(group.robot_name)
        if str(group.group_name) == "manipulator"
        else f"{group.robot_name} {group.group_name}"
    )


def current_state(
    world_monitor: WorldMonitor, module: ManipulationModule, robot_name: RobotName
) -> JointState | None:
    robot_id = module.robot_id_for_name(robot_name)
    return (
        None
        if robot_id is None
        else copy_joint_state(world_monitor.get_current_joint_state(robot_id))
    )


def _positions_by_name(robot_name: RobotName, state: JointState) -> dict[str, float] | None:
    if len(state.name) != len(state.position):
        return None
    values: dict[str, float] = {}
    for name, value in zip(state.name, state.position, strict=True):
        name_string = str(name)
        values[name_string] = float(value)
        values[name_string.rsplit("/", 1)[-1]] = float(value)
        values[f"{robot_name}/{name_string.rsplit('/', 1)[-1]}"] = float(value)
    return values


def _complete_states(
    world_monitor: WorldMonitor,
    module: ManipulationModule,
    groups: Sequence[PlanningGroup],
    target: JointState,
) -> dict[RobotName, JointState] | None:
    """Overlay selected global joints onto one fixed current snapshot per robot."""
    if len(target.name) != len(target.position):
        return None
    target_values = {
        str(name): float(value) for name, value in zip(target.name, target.position, strict=True)
    }
    complete: dict[RobotName, JointState] = {}
    for robot_name in dict.fromkeys(group.robot_name for group in groups):
        config = module.get_robot_config(robot_name)
        baseline = current_state(world_monitor, module, robot_name)
        if config is None or baseline is None:
            return None
        baseline_values = _positions_by_name(robot_name, baseline)
        if baseline_values is None:
            return None
        positions: list[float] = []
        for local_name in config.joint_names:
            global_name = f"{robot_name}/{local_name}"
            # Targets crossing this boundary are always global.  A local name
            # is ambiguous as soon as more than one robot is selected.
            value = target_values.get(global_name)
            if value is None:
                value = baseline_values.get(local_name)
            if value is None:
                return None
            positions.append(float(value))
        complete[robot_name] = JointState({"name": list(config.joint_names), "position": positions})
    return complete


def complete_states_for_targets(
    world_monitor: WorldMonitor,
    module: ManipulationModule,
    group_ids: Sequence[PlanningGroupID],
    joint_targets: Mapping[PlanningGroupID, JointState],
) -> dict[RobotName, JointState] | None:
    """Compose selected global targets over one current snapshot per robot."""
    groups_by_id = {group.id: group for group in module.list_planning_groups()}
    groups = tuple(groups_by_id[group_id] for group_id in group_ids if group_id in groups_by_id)
    if len(groups) != len(group_ids) or not groups:
        return None
    names: list[str] = []
    positions: list[float] = []
    for group in groups:
        target = joint_targets.get(group.id)
        if target is None or len(target.name) != len(target.position):
            return None
        expected = tuple(str(name) for name in group.joint_names)
        if tuple(str(name) for name in target.name) != expected:
            return None
        names.extend(expected)
        positions.extend(float(position) for position in target.position)
    return _complete_states(
        world_monitor, module, groups, JointState({"name": names, "position": positions})
    )


def evaluate_joint_target_set(
    world_monitor: WorldMonitor,
    module: ManipulationModule,
    group_ids: Sequence[PlanningGroupID],
    joint_targets: Mapping[PlanningGroupID, JointState],
) -> TargetSetEvaluation:
    groups = tuple(module.list_planning_groups())
    by_id = {group.id: group for group in groups}
    selected = tuple(by_id[group_id] for group_id in group_ids if group_id in by_id)
    if len(selected) != len(group_ids) or not selected:
        return {"success": False, "status": "INVALID", "message": "Unknown planning group"}
    complete = complete_states_for_targets(world_monitor, module, group_ids, joint_targets)
    if complete is None:
        return {"success": False, "status": "INVALID", "message": "Malformed joint target"}
    names = [str(name) for group in selected for name in group.joint_names]
    positions = [
        float(position) for group in selected for position in joint_targets[group.id].position
    ]
    return evaluate_global_target_set(
        world_monitor,
        module,
        selected,
        JointState({"name": names, "position": positions}),
        complete,
    )


def evaluate_pose_target_set(
    world_monitor: WorldMonitor,
    module: ManipulationModule,
    pose_targets: Mapping[PlanningGroupID, Pose],
    auxiliary_group_ids: Sequence[PlanningGroupID] = (),
    seed: JointState | None = None,
) -> TargetSetEvaluation:
    if not pose_targets:
        return {"success": False, "status": "INVALID", "message": "No pose target"}
    stamped = {
        group_id: PoseStamped(
            frame_id="world", position=pose.position, orientation=pose.orientation
        )
        for group_id, pose in pose_targets.items()
    }
    group_ids = tuple(dict.fromkeys((*stamped, *auxiliary_group_ids)))
    ik = module.inverse_kinematics(
        pose_targets=stamped,
        auxiliary_group_ids=auxiliary_group_ids,
        seed=copy_joint_state(seed),
        check_collision=True,
    )
    if not ik.is_success() or ik.joint_state is None:
        return {
            "success": False,
            "status": ik.status.name,
            "message": ik.message,
            "collision_free": False,
            "group_ids": group_ids,
        }
    groups_by_id = {group.id: group for group in module.list_planning_groups()}
    groups = tuple(groups_by_id[group_id] for group_id in group_ids if group_id in groups_by_id)
    if len(groups) != len(group_ids):
        return {"success": False, "status": "INVALID", "message": "Unknown planning group"}
    return evaluate_global_target_set(world_monitor, module, groups, ik.joint_state)


def evaluate_global_target_set(
    world_monitor: WorldMonitor,
    module: ManipulationModule,
    groups: Sequence[PlanningGroup],
    target: JointState,
    complete_states: Mapping[RobotName, JointState] | None = None,
) -> TargetSetEvaluation:
    complete = complete_states or _complete_states(world_monitor, module, groups, target)
    if complete is None:
        return {"success": False, "status": "INVALID", "message": "Incomplete robot target state"}
    poses: dict[PlanningGroupID, PoseStamped | None] = {}
    diagnostics: dict[PlanningGroupID, str] = {}
    valid = True
    for group in groups:
        robot_id = module.robot_id_for_name(group.robot_name)
        state = complete[group.robot_name]
        if robot_id is None or not world_monitor.is_state_valid(robot_id, state):
            valid = False
            diagnostics[group.id] = "Target is in collision or violates limits"
        else:
            diagnostics[group.id] = "Target is collision-free for this robot"
        try:
            poses[group.id] = world_monitor.get_group_ee_pose(group.id, state)
        except ValueError:
            poses[group.id] = None
    return {
        "success": valid,
        "status": "FEASIBLE" if valid else "COLLISION",
        "message": ("Target is collision-free for each robot" if valid else "Target is infeasible"),
        "collision_free": valid,
        "group_ids": tuple(group.id for group in groups),
        "target_joints": JointState(target),
        "group_diagnostics": diagnostics,
        "group_poses": poses,
    }


class GroupPanelBackend:
    """Small UI boundary over the PR4 public APIs (never planner internals)."""

    def __init__(self, world_monitor: WorldMonitor, module: ManipulationModule) -> None:
        self.world_monitor = world_monitor
        self.module = module

    def list_robots(self) -> list[RobotName]:
        return list(self.module.list_robots())

    def list_planning_groups(self) -> list[PlanningGroup]:
        return list(self.module.list_planning_groups())

    def robot_items(self) -> list[tuple[RobotName, WorldRobotID, RobotModelConfig]]:
        return self.module.robot_items()

    def robot_id_for_name(self, robot_name: RobotName) -> WorldRobotID | None:
        return self.module.robot_id_for_name(robot_name)

    def get_robot_config(self, robot_name: RobotName) -> RobotModelConfig | None:
        return self.module.get_robot_config(robot_name)

    def get_robot_info(self, robot_name: RobotName) -> RobotInfo | None:
        info = self.module.get_robot_info(robot_name)
        return None if info is None else cast("RobotInfo", info)

    def get_init_joints(self, robot_name: RobotName) -> JointState | None:
        return copy_joint_state(self.module.get_init_joints(robot_name))

    def get_current_joint_state(self, robot_name: RobotName) -> JointState | None:
        return current_state(self.world_monitor, self.module, robot_name)

    def get_ee_pose(
        self, robot_name: RobotName, state: JointState | None = None
    ) -> PoseStamped | None:
        group = next(
            (
                item
                for item in self.list_planning_groups()
                if item.robot_name == robot_name and item.has_pose_target
            ),
            None,
        )
        if group is None:
            return None
        try:
            return self.world_monitor.get_group_ee_pose(group.id, state)
        except ValueError:
            return None

    def is_state_stale(self, robot_name: RobotName, max_age: float = 1.0) -> bool:
        robot_id = self.module.robot_id_for_name(robot_name)
        return robot_id is None or self.world_monitor.is_state_stale(robot_id, max_age)

    def get_module_state(self) -> str:
        return str(self.module.get_state())

    def get_error(self) -> str:
        return self.module.get_error()

    def reset(self) -> bool:
        return self.module.reset().is_success()

    def joints_from_values(self, names: Sequence[str], values: Sequence[float]) -> JointState:
        return JointState({"name": list(names), "position": [float(value) for value in values]})

    def evaluate_joint_target_set(
        self, group_ids: Sequence[PlanningGroupID], targets: Mapping[PlanningGroupID, JointState]
    ) -> TargetSetEvaluation:
        return evaluate_joint_target_set(self.world_monitor, self.module, group_ids, targets)

    def evaluate_pose_target_set(
        self,
        pose_targets: Mapping[PlanningGroupID, Pose],
        auxiliary_group_ids: Sequence[PlanningGroupID] = (),
        seed: JointState | None = None,
    ) -> TargetSetEvaluation:
        return evaluate_pose_target_set(
            self.world_monitor, self.module, pose_targets, auxiliary_group_ids, seed
        )

    def complete_states_for_targets(
        self, group_ids: Sequence[PlanningGroupID], targets: Mapping[PlanningGroupID, JointState]
    ) -> dict[RobotName, JointState] | None:
        return complete_states_for_targets(self.world_monitor, self.module, group_ids, targets)

    def cancel(self) -> bool:
        return self.module.cancel()

    def clear_planned_path(self) -> bool:
        return self.module.clear_planned_path()

    def plan_to_joints(self, joints: JointState, robot_name: RobotName | None = None) -> bool:
        if robot_name is None:
            return False
        group = next(
            (item for item in self.list_planning_groups() if item.robot_name == robot_name), None
        )
        return group is not None and self.module.plan_to_joint_targets({group.id: joints})

    def plan_to_selected_joints(
        self, group_ids: Sequence[PlanningGroupID], targets: Mapping[PlanningGroupID, JointState]
    ) -> bool:
        """Plan the captured target for every selected group."""
        if complete_states_for_targets(self.world_monitor, self.module, group_ids, targets) is None:
            return False
        return bool(targets) and self.module.plan_to_joint_targets(
            cast("Mapping[PlanningGroupID | PlanningGroup, JointState]", targets)
        )

    def preview_path(self) -> bool:
        return self.module.preview_plan()

    def execute(self) -> bool:
        return self.module.execute()

    def snapshot_selected_robots(
        self, group_ids: Sequence[PlanningGroupID]
    ) -> dict[str, JointState] | None:
        groups = {group.id: group for group in self.list_planning_groups()}
        snapshots: dict[str, JointState] = {}
        for group_id in group_ids:
            group = groups.get(group_id)
            if group is None:
                return None
            name = str(group.robot_name)
            if name in snapshots:
                continue
            state = self.get_current_joint_state(group.robot_name)
            if state is None or len(state.name) != len(state.position):
                return None
            snapshots[name] = state
        return snapshots

    def snapshots_match(self, snapshots: Mapping[str, JointState], tolerance: float) -> bool:
        for robot_name, expected in snapshots.items():
            current = self.get_current_joint_state(robot_name)
            if (
                current is None
                or len(current.name) != len(current.position)
                or len(expected.name) != len(expected.position)
            ):
                return False
            actual = _positions_by_name(robot_name, current)
            desired = _positions_by_name(robot_name, expected)
            if actual is None or desired is None or actual.keys() != desired.keys():
                return False
            if any(abs(actual[name] - desired[name]) > tolerance for name in desired):
                return False
        return True

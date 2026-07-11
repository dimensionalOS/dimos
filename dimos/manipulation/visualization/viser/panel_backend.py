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

"""Viser panel adapter over static session metadata and ManipulationOperator."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import cast

from dimos.manipulation.manipulation_operator import (
    JointTargetRequest,
    ManipulationOperator,
    PoseTargetRequest,
    TargetEvaluationResult,
)
from dimos.manipulation.planning.groups.models import PlanningGroup
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.models import PlanningGroupID, PlanningSceneInfo, RobotName
from dimos.manipulation.visualization.types import RobotInfo, TargetSetEvaluation
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState


def copy_joint_state(state: JointState | None) -> JointState | None:
    return None if state is None else JointState(state)


def group_display_name(group: PlanningGroup) -> str:
    return (
        str(group.robot_name)
        if str(group.group_name) == "manipulator"
        else f"{group.robot_name} {group.group_name}"
    )


class ViserPanelBackend:
    """Viser panel boundary backed by operator/session data only."""

    def __init__(
        self,
        scene: PlanningSceneInfo,
        operator: ManipulationOperator | object,
        current_states: Mapping[str, JointState],
    ) -> None:
        self._scene = scene
        self._operator = operator
        self._current_states = current_states
        self._robots_by_name = {
            config.name: (robot_id, config) for robot_id, config in scene.robots.items()
        }
        self._groups_by_id = {group.id: group for group in scene.planning_groups}

    @property
    def operator(self) -> ManipulationOperator:
        return cast("ManipulationOperator", self._operator)

    def list_robots(self) -> list[RobotName]:
        return [config.name for config in self._scene.robots.values()]

    def list_planning_groups(self) -> list[PlanningGroup]:
        return list(self._scene.planning_groups)

    def robot_items(self) -> list[tuple[RobotName, str, RobotModelConfig]]:
        return [
            (config.name, str(robot_id), config) for robot_id, config in self._scene.robots.items()
        ]

    def robot_id_for_name(self, robot_name: RobotName) -> str | None:
        item = self._robots_by_name.get(robot_name)
        return None if item is None else str(item[0])

    def get_robot_config(self, robot_name: RobotName) -> RobotModelConfig | None:
        item = self._robots_by_name.get(robot_name)
        return None if item is None else item[1]

    def get_robot_info(self, robot_name: RobotName) -> RobotInfo | None:
        item = self._robots_by_name.get(robot_name)
        if item is None:
            return None
        robot_id, config = item
        groups = [group for group in self._scene.planning_groups if group.robot_name == robot_name]
        init = self.get_init_joints(robot_name)
        return cast(
            "RobotInfo",
            {
                "name": config.name,
                "world_robot_id": str(robot_id),
                "joint_names": config.joint_names,
                "planning_groups": groups,
                "end_effector_link": config.end_effector_link if groups else None,
                "base_link": config.base_link,
                "max_velocity": config.max_velocity,
                "max_acceleration": config.max_acceleration,
                "has_joint_name_mapping": bool(config.joint_name_mapping),
                "coordinator_task_name": config.coordinator_task_name,
                "home_joints": config.home_joints,
                "pre_grasp_offset": config.pre_grasp_offset,
                "init_joints": None if init is None else list(init.position),
            },
        )

    def get_init_joints(self, robot_name: RobotName) -> JointState | None:
        init = self.operator.get_init_joints(robot_name)
        if init is None:
            return None
        config = self.get_robot_config(robot_name)
        if config is None:
            return JointState(init)
        values = self._local_values_for_robot(robot_name, init)
        if any(name not in values for name in config.joint_names):
            return JointState(init)
        return JointState(
            {
                "name": list(config.joint_names),
                "position": [values[name] for name in config.joint_names],
            }
        )

    def get_current_joint_state(self, robot_name: RobotName) -> JointState | None:
        robot_id = self.robot_id_for_name(robot_name)
        return None if robot_id is None else copy_joint_state(self._current_states.get(robot_id))

    def get_ee_pose(
        self, robot_name: RobotName, state: JointState | None = None
    ) -> PoseStamped | None:
        _ = state
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
        return self.get_group_ee_pose(group.id)

    def get_group_ee_pose(self, group_id: PlanningGroupID) -> PoseStamped | None:
        group = self._groups_by_id.get(group_id)
        if group is None:
            return None
        targets = self._current_target_for_group(group)
        if group_id not in targets:
            return None
        pose = (
            self.evaluate_joint_target_set((group_id,), targets)
            .get("group_poses", {})
            .get(group_id)
        )
        return cast("PoseStamped | None", pose)

    def _current_target_for_group(self, group: PlanningGroup) -> dict[PlanningGroupID, JointState]:
        current = self.get_current_joint_state(group.robot_name)
        if current is None or len(current.name) != len(current.position):
            return {}
        values = self._local_values_for_robot(group.robot_name, current)
        if any(name not in values for name in group.local_joint_names):
            return {}
        return {
            group.id: JointState(
                {
                    "name": list(group.joint_names),
                    "position": [values[name] for name in group.local_joint_names],
                }
            )
        }

    def _local_values_for_robot(self, robot_name: RobotName, state: JointState) -> dict[str, float]:
        config = self.get_robot_config(robot_name)
        if config is None or len(state.name) != len(state.position):
            return {}
        raw = {
            str(name): float(value) for name, value in zip(state.name, state.position, strict=True)
        }
        values: dict[str, float] = {}
        for local_name in config.joint_names:
            global_name = f"{robot_name}/{local_name}"
            if local_name in raw:
                values[local_name] = raw[local_name]
            elif global_name in raw:
                values[local_name] = raw[global_name]
        return values

    def is_state_stale(self, robot_name: RobotName, max_age: float = 1.0) -> bool:
        _ = max_age
        return self.get_current_joint_state(robot_name) is None

    def get_module_state(self) -> str:
        return self.operator.status().state

    def get_error(self) -> str:
        return self.operator.status().error

    def reset(self) -> bool:
        return self.operator.reset().success

    def joints_from_values(self, names: Sequence[str], values: Sequence[float]) -> JointState:
        return JointState({"name": list(names), "position": [float(value) for value in values]})

    def evaluate_joint_target_set(
        self, group_ids: Sequence[PlanningGroupID], targets: Mapping[PlanningGroupID, JointState]
    ) -> TargetSetEvaluation:
        names: list[str] = []
        positions: list[float] = []
        for group_id in group_ids:
            target = targets.get(group_id)
            if target is None:
                return {"success": False, "status": "INVALID", "message": "Incomplete joint target"}
            names.extend(str(name) for name in target.name)
            positions.extend(float(value) for value in target.position)
        return self._evaluation_to_dict(
            self.operator.evaluate_joint_target(
                JointTargetRequest(
                    tuple(group_ids), JointState({"name": names, "position": positions})
                )
            )
        )

    def evaluate_pose_target_set(
        self,
        pose_targets: Mapping[PlanningGroupID, Pose],
        auxiliary_group_ids: Sequence[PlanningGroupID] = (),
        seed: JointState | None = None,
    ) -> TargetSetEvaluation:
        stamped = {
            group_id: PoseStamped(
                frame_id="world", position=pose.position, orientation=pose.orientation
            )
            for group_id, pose in pose_targets.items()
        }
        return self._evaluation_to_dict(
            self.operator.evaluate_pose_target(
                PoseTargetRequest(stamped, tuple(auxiliary_group_ids), copy_joint_state(seed))
            )
        )

    def cancel(self) -> bool:
        return self.operator.cancel().success

    def clear_planned_path(self) -> bool:
        return self.operator.clear_plan().success

    def plan_to_selected_joints(
        self, group_ids: Sequence[PlanningGroupID], targets: Mapping[PlanningGroupID, JointState]
    ) -> bool:
        names: list[str] = []
        positions: list[float] = []
        for group_id in group_ids:
            target = targets.get(group_id)
            if target is None:
                return False
            names.extend(str(name) for name in target.name)
            positions.extend(float(value) for value in target.position)
        return self.operator.plan_to_joints(
            JointTargetRequest(tuple(group_ids), JointState({"name": names, "position": positions}))
        ).success

    def preview_path(self) -> bool:
        return self.operator.preview().success

    def execute(self) -> bool:
        return self.operator.execute().success

    @staticmethod
    def _evaluation_to_dict(result: TargetEvaluationResult) -> TargetSetEvaluation:
        return cast(
            "TargetSetEvaluation",
            {
                "success": result.success,
                "status": result.status,
                "message": result.message,
                "collision_free": result.collision_free,
                "group_ids": result.group_ids,
                "target_joints": result.target_joints,
                "group_diagnostics": dict(result.group_diagnostics),
                "group_poses": dict(result.group_poses),
            },
        )

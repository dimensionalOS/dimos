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

from collections.abc import Sequence
from typing import TYPE_CHECKING, cast

from dimos.manipulation.planning.spec.enums import IKStatus
from dimos.manipulation.visualization.types import RobotInfo, TargetEvaluation
from dimos.msgs.sensor_msgs.JointState import JointState

if TYPE_CHECKING:
    from dimos.manipulation.manipulation_module import ManipulationModule
    from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor
    from dimos.manipulation.planning.spec.config import RobotModelConfig
    from dimos.manipulation.planning.spec.models import JointPath, RobotName, WorldRobotID
    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


def copy_joint_state(joint_state: JointState | None) -> JointState | None:
    """Make a local copy of a JointState-like message for rendering."""
    return None if joint_state is None else JointState(joint_state)


class InProcessViserAdapter:
    """Small in-process boundary between Viser callbacks and manipulation internals."""

    def __init__(
        self,
        *,
        world_monitor: WorldMonitor,
        manipulation_module: ManipulationModule,
    ) -> None:
        self._world_monitor = world_monitor
        self._module = manipulation_module

    def list_robots(self) -> list[RobotName]:
        return list(self._module.list_robots())

    def robot_items(self) -> list[tuple[RobotName, WorldRobotID, RobotModelConfig]]:
        return self._module.robot_items()

    def robot_id_for_name(self, robot_name: RobotName) -> WorldRobotID | None:
        return self._module.robot_id_for_name(robot_name)

    def robot_name_for_id(self, robot_id: WorldRobotID) -> RobotName | None:
        return self._module.robot_name_for_id(robot_id)

    def get_robot_config(self, robot_name: RobotName) -> RobotModelConfig | None:
        return self._module.get_robot_config(robot_name)

    def get_robot_info(self, robot_name: RobotName) -> RobotInfo | None:
        info = self._module.get_robot_info(robot_name)
        if info is None:
            return None
        return info

    def get_init_joints(self, robot_name: RobotName) -> JointState | None:
        return copy_joint_state(self._module.get_init_joints(robot_name))

    def get_current_joint_state(self, robot_name: RobotName) -> JointState | None:
        robot_id = self.robot_id_for_name(robot_name)
        if robot_id is None:
            return None
        return copy_joint_state(self._world_monitor.get_current_joint_state(robot_id))

    def is_state_stale(self, robot_name: RobotName, max_age: float = 1.0) -> bool:
        robot_id = self.robot_id_for_name(robot_name)
        return True if robot_id is None else self._world_monitor.is_state_stale(robot_id, max_age)

    def get_ee_pose(
        self, robot_name: RobotName, joint_state: JointState | None = None
    ) -> PoseStamped | None:
        robot_id = self.robot_id_for_name(robot_name)
        if robot_id is None:
            return None
        return self._world_monitor.get_ee_pose(robot_id, copy_joint_state(joint_state))

    def evaluate_joint_target(self, joints: JointState, robot_name: RobotName) -> TargetEvaluation:
        """Evaluate a legacy robot-scoped joint target through group-scoped APIs."""
        legacy_evaluate = getattr(self._module, "evaluate_joint_target", None)
        if callable(legacy_evaluate):
            result = cast("TargetEvaluation", legacy_evaluate(JointState(joints), robot_name))
            joint_state = result.get("joint_state")
            result["joint_state"] = copy_joint_state(
                joint_state if isinstance(joint_state, JointState) else None
            )
            return result
        default_group_id = getattr(self._module, "_default_group_id_for_robot", None)
        joint_target_to_global = getattr(self._module, "_joint_target_to_global_names", None)
        if not callable(default_group_id) or not callable(joint_target_to_global):
            robot_id = self.robot_id_for_name(robot_name)
            valid = (
                False
                if robot_id is None
                else bool(self._world_monitor.is_state_valid(robot_id, JointState(joints)))
            )
            return {
                "success": valid,
                "status": "FEASIBLE" if valid else "COLLISION",
                "message": "Target is valid" if valid else "Target is in collision",
                "collision_free": valid,
                "joint_state": JointState(joints),
            }
        group_id = self._module._default_group_id_for_robot(robot_name)
        if group_id is None:
            return {"success": False, "status": "INVALID", "message": "No default group"}
        target = self._module._joint_target_to_global_names(group_id, JointState(joints))
        if target is None:
            return {"success": False, "status": "INVALID", "message": "Invalid joint target"}
        collision = self._module.check_collision(target)
        collision_free = collision.collision_free is True
        return {
            "success": collision_free,
            "status": collision.status,
            "message": collision.message,
            "collision_free": collision_free,
            "joint_state": copy_joint_state(target),
        }

    def evaluate_pose_target(self, pose: Pose, robot_name: RobotName) -> TargetEvaluation:
        """Evaluate a legacy robot-scoped pose target through group-scoped IK."""
        ik = self._module.inverse_kinematics_single(pose, robot_name=robot_name)
        if not ik.is_success() or ik.joint_state is None:
            return {
                "success": False,
                "status": ik.status.name,
                "message": ik.message,
                "collision_free": False,
                "joint_state": None,
                "position_error": ik.position_error,
                "orientation_error": ik.orientation_error,
            }
        collision = self._module.check_collision(ik.joint_state)
        collision_free = collision.collision_free is True
        return {
            "success": collision_free,
            "status": collision.status if not collision_free else IKStatus.SUCCESS.name,
            "message": collision.message,
            "collision_free": collision_free,
            "joint_state": copy_joint_state(ik.joint_state),
            "position_error": ik.position_error,
            "orientation_error": ik.orientation_error,
        }

    def get_planned_path(self, robot_name: RobotName) -> JointPath | None:
        legacy_get_planned_path = getattr(self._module, "get_planned_path", None)
        if callable(legacy_get_planned_path):
            legacy_path = legacy_get_planned_path(robot_name)
            if legacy_path is None:
                return None
            if not isinstance(legacy_path, list):
                return None
            copied = [copy_joint_state(point) for point in legacy_path]
            return [point for point in copied if point is not None]
        planned_paths = getattr(self._module, "_planned_paths", None)
        if isinstance(planned_paths, dict):
            path_obj = planned_paths.get(robot_name)
            if isinstance(path_obj, list):
                copied = [copy_joint_state(point) for point in path_obj]
                return [point for point in copied if point is not None]
        plan = getattr(self._module, "_last_plan", None)
        config = self.get_robot_config(robot_name)
        current = self.get_current_joint_state(robot_name)
        if plan is None or config is None or current is None:
            return None
        path: JointPath = []
        current_by_name = dict(zip(current.name, current.position, strict=False))
        for waypoint in plan.path:
            selected = dict(zip(waypoint.name, waypoint.position, strict=False))
            positions: list[float] = []
            for local_name in config.joint_names:
                global_name = f"{robot_name}/{local_name}"
                if global_name in selected:
                    positions.append(float(selected[global_name]))
                elif local_name in current_by_name:
                    positions.append(float(current_by_name[local_name]))
                else:
                    return None
            path.append(JointState(name=list(config.joint_names), position=positions))
        return path

    def get_planned_trajectory_duration(self, robot_name: RobotName) -> float | None:
        path = self.get_planned_path(robot_name)
        return None if path is None else float(max(len(path) - 1, 0))

    def get_module_state(self) -> str:
        return str(self._module.get_state())

    def get_error(self) -> str:
        return self._module.get_error()

    def reset(self) -> bool:
        return self._module.reset().is_success()

    def plan_to_pose(self, pose: Pose, robot_name: RobotName | None = None) -> bool:
        return self._module.plan_to_pose(pose, robot_name)

    def plan_to_joints(self, joints: JointState, robot_name: RobotName | None = None) -> bool:
        return self._module.plan_to_joints(joints, robot_name)

    def preview_path(self, robot_name: RobotName | None = None) -> object:
        preview_path = getattr(self._module, "preview_path", None)
        if callable(preview_path):
            return preview_path(robot_name=robot_name)
        return self._module.preview_plan(robot_name=robot_name)

    def execute(self, robot_name: RobotName | None = None) -> bool:
        execute_plan = getattr(self._module, "execute_plan", None)
        if callable(execute_plan):
            return bool(execute_plan())
        execute = getattr(self._module, "execute", None)
        return bool(execute(robot_name)) if callable(execute) else False

    def cancel(self) -> bool:
        return self._module.cancel()

    def clear_planned_path(self) -> bool:
        return self._module.clear_planned_path()

    @staticmethod
    def joints_from_values(joint_names: Sequence[str], values: Sequence[float]) -> JointState:
        return JointState(
            {
                "name": list(joint_names),
                "position": [float(value) for value in values],
            }
        )

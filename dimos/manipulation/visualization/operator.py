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

"""UI-neutral facade for interactive manipulation."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
import math
from typing import TYPE_CHECKING, cast

from dimos.manipulation.planning.groups.models import PlanningGroup
from dimos.manipulation.planning.spec.models import GeneratedPlan, PlanningGroupID, RobotName
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState

if TYPE_CHECKING:
    from dimos.manipulation.manipulation_module import ManipulationModule
    from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor


@dataclass(frozen=True)
class OperatorStatus:
    """Compact dynamic manipulation status."""

    state: str
    error: str
    has_plan: bool


@dataclass(frozen=True)
class JointTargetRequest:
    """Canonical selected joint target request."""

    group_ids: tuple[PlanningGroupID, ...]
    target: JointState


@dataclass(frozen=True)
class PoseTargetRequest:
    """Explicit world-frame pose target request."""

    pose_targets: Mapping[PlanningGroupID, PoseStamped]
    auxiliary_group_ids: tuple[PlanningGroupID, ...] = ()
    seed: JointState | None = None


@dataclass(frozen=True)
class TargetEvaluationResult:
    """Advisory selected-domain target evaluation."""

    success: bool
    status: str
    message: str
    collision_free: bool = False
    group_ids: tuple[PlanningGroupID, ...] = ()
    target_joints: JointState | None = None
    group_diagnostics: Mapping[PlanningGroupID, str] = field(default_factory=dict)
    group_poses: Mapping[PlanningGroupID, PoseStamped | None] = field(default_factory=dict)


class ManipulationOperator:
    """Concrete synchronous facade over ManipulationModule and WorldMonitor."""

    def __init__(self, module: ManipulationModule, world_monitor: WorldMonitor) -> None:
        self._module = module
        self._world_monitor = world_monitor

    def status(self) -> OperatorStatus:
        """Return compact dynamic state without topology or joint telemetry."""
        return OperatorStatus(
            state=self._module.get_state(),
            error=self._module.get_error(),
            has_plan=self._module.has_planned_path(),
        )

    def get_init_joints(self, robot_name: RobotName) -> JointState | None:
        """Return the operator-authoritative init joint state for a robot."""
        init = self._module.get_init_joints(robot_name)
        return None if init is None else JointState(init)

    def evaluate_joint_target(self, request: JointTargetRequest) -> TargetEvaluationResult:
        """Validate and evaluate a canonical global joint target."""
        groups, validation = self._validate_joint_request(request)
        if validation is not None:
            return validation
        assert groups is not None
        complete = self._complete_states(groups, request.target)
        if complete is None:
            return self._invalid(request.group_ids, "Incomplete robot target state")
        return self._evaluate_global_target(groups, JointState(request.target), complete)

    def evaluate_pose_target(self, request: PoseTargetRequest) -> TargetEvaluationResult:
        """Validate and evaluate explicit world-frame pose targets."""
        group_ids, validation = self._validate_pose_request(request)
        if validation is not None:
            return validation
        ik = self._module.inverse_kinematics(
            pose_targets=dict(request.pose_targets),
            auxiliary_group_ids=request.auxiliary_group_ids,
            seed=JointState(request.seed) if request.seed is not None else None,
            check_collision=True,
        )
        if not ik.is_success() or ik.joint_state is None:
            return TargetEvaluationResult(
                success=False,
                status=ik.status.name,
                message=ik.message,
                collision_free=False,
                group_ids=group_ids,
            )
        groups = self._groups_for_ids(group_ids)
        if groups is None:
            return self._invalid(group_ids, "Unknown planning group")
        return self._evaluate_global_target(groups, ik.joint_state)

    def plan_to_joints(self, request: JointTargetRequest) -> GeneratedPlan | None:
        groups, validation = self._validate_joint_request(request)
        if validation is not None:
            return None
        assert groups is not None
        targets = {
            group.id: JointState(
                {
                    "name": list(group.joint_names),
                    "position": list(
                        request.target.position[offset : offset + len(group.joint_names)]
                    ),
                }
            )
            for group, offset in self._group_offsets(groups)
        }
        return self._module.generate_plan_to_joint_targets(
            cast("Mapping[PlanningGroupID | PlanningGroup, JointState]", targets)
        )

    def plan_to_pose(self, request: PoseTargetRequest) -> GeneratedPlan | None:
        group_ids, validation = self._validate_pose_request(request)
        if validation is not None:
            return None
        poses = {group_id: stamped for group_id, stamped in request.pose_targets.items()}
        return self._module.generate_plan_to_pose_targets(
            cast("Mapping[PlanningGroupID | PlanningGroup, PoseStamped]", poses),
            request.auxiliary_group_ids,
        )

    def preview(self, plan: GeneratedPlan, duration: float | None = None) -> bool:
        return self._module.preview_plan(plan=plan, duration=duration)

    def execute(self, plan: GeneratedPlan) -> bool:
        return self._module.execute_plan(plan=plan)

    def cancel(self) -> bool:
        return self._module.cancel()

    def clear_plan(self) -> bool:
        return self._module.clear_planned_path()

    def reset(self) -> bool:
        result = self._module.reset()
        return result.is_success()

    def _validate_joint_request(
        self, request: JointTargetRequest
    ) -> tuple[tuple[PlanningGroup, ...] | None, TargetEvaluationResult | None]:
        groups = self._groups_for_ids(request.group_ids)
        if groups is None:
            return None, self._invalid(
                request.group_ids, "Unknown, duplicate, or overlapping planning group"
            )
        expected = tuple(name for group in groups for name in group.joint_names)
        names = tuple(str(name) for name in request.target.name)
        positions = tuple(float(value) for value in request.target.position)
        if len(names) != len(positions):
            return None, self._invalid(request.group_ids, "Joint target names and positions differ")
        if len(set(names)) != len(names):
            return None, self._invalid(request.group_ids, "Joint target contains duplicate joints")
        if names != expected:
            return None, self._invalid(
                request.group_ids, "Joint target must use exact selected global joints in order"
            )
        if any(not math.isfinite(value) for value in positions):
            return None, self._invalid(
                request.group_ids, "Joint target contains non-finite positions"
            )
        return groups, None

    def _validate_pose_request(
        self, request: PoseTargetRequest
    ) -> tuple[tuple[PlanningGroupID, ...], TargetEvaluationResult | None]:
        if not request.pose_targets:
            return (), self._invalid((), "No pose target")
        group_ids = tuple(
            dict.fromkeys((*request.pose_targets.keys(), *request.auxiliary_group_ids))
        )
        groups = self._groups_for_ids(group_ids)
        if groups is None:
            return group_ids, self._invalid(
                group_ids, "Unknown, duplicate, or overlapping planning group"
            )
        pose_group_ids = set(request.pose_targets)
        for group in groups:
            if group.id in pose_group_ids and not group.has_pose_target:
                return group_ids, self._invalid(
                    group_ids, f"Planning group '{group.id}' has no tip_link"
                )
        for group_id, pose in request.pose_targets.items():
            if pose.frame_id != "world":
                return group_ids, self._invalid(
                    group_ids, f"Unsupported pose frame for '{group_id}': {pose.frame_id}"
                )
            if not self._pose_is_finite(pose):
                return group_ids, self._invalid(
                    group_ids, f"Malformed pose target for '{group_id}'"
                )
        if request.seed is not None:
            seed_names = tuple(str(name) for name in request.seed.name)
            if len(seed_names) != len(request.seed.position) or len(set(seed_names)) != len(
                seed_names
            ):
                return group_ids, self._invalid(group_ids, "Malformed seed")
            expected = tuple(name for group in groups for name in group.joint_names)
            if seed_names != expected or any("/" not in name for name in seed_names):
                return group_ids, self._invalid(
                    group_ids, "Seed must use exact selected global joints in order"
                )
            if any(not math.isfinite(float(value)) for value in request.seed.position):
                return group_ids, self._invalid(group_ids, "Seed contains non-finite positions")
        return group_ids, None

    def _groups_for_ids(
        self, group_ids: Sequence[PlanningGroupID]
    ) -> tuple[PlanningGroup, ...] | None:
        if not group_ids or len(set(group_ids)) != len(group_ids):
            return None
        try:
            selection = self._world_monitor.planning_groups.select(tuple(group_ids))
        except (KeyError, ValueError):
            return None
        return selection.groups

    def _complete_states(
        self, groups: Sequence[PlanningGroup], target: JointState
    ) -> dict[RobotName, JointState] | None:
        values = {
            str(name): float(value)
            for name, value in zip(target.name, target.position, strict=True)
        }
        complete: dict[RobotName, JointState] = {}
        for robot_name in dict.fromkeys(group.robot_name for group in groups):
            config = self._module.get_robot_config(robot_name)
            robot_id = self._module.robot_id_for_name(robot_name)
            baseline = (
                None if robot_id is None else self._world_monitor.get_current_joint_state(robot_id)
            )
            if config is None or baseline is None or len(baseline.name) != len(baseline.position):
                return None
            baseline_values = {
                str(name): float(value)
                for name, value in zip(baseline.name, baseline.position, strict=True)
            }
            positions: list[float] = []
            for local_name in config.joint_names:
                global_name = f"{robot_name}/{local_name}"
                value = values.get(global_name, baseline_values.get(local_name))
                if value is None:
                    return None
                positions.append(value)
            complete[robot_name] = JointState(
                {"name": list(config.joint_names), "position": positions}
            )
        return complete

    def _evaluate_global_target(
        self,
        groups: Sequence[PlanningGroup],
        target: JointState,
        complete_states: Mapping[RobotName, JointState] | None = None,
    ) -> TargetEvaluationResult:
        complete = complete_states or self._complete_states(groups, target)
        if complete is None:
            return self._invalid(
                tuple(group.id for group in groups), "Incomplete robot target state"
            )
        diagnostics: dict[PlanningGroupID, str] = {}
        poses: dict[PlanningGroupID, PoseStamped | None] = {}
        valid = True
        for group in groups:
            robot_id = self._module.robot_id_for_name(group.robot_name)
            state = complete[group.robot_name]
            group_valid = bool(
                robot_id is not None and self._world_monitor.is_state_valid(robot_id, state)
            )
            valid = valid and group_valid
            diagnostics[group.id] = (
                "Target is collision-free for this robot"
                if group_valid
                else "Target is in collision or violates limits"
            )
            try:
                poses[group.id] = self._world_monitor.get_group_ee_pose(group.id, state)
            except ValueError:
                poses[group.id] = None
        return TargetEvaluationResult(
            success=valid,
            status="FEASIBLE" if valid else "COLLISION",
            message="Target is collision-free for each robot" if valid else "Target is infeasible",
            collision_free=valid,
            group_ids=tuple(group.id for group in groups),
            target_joints=JointState(target),
            group_diagnostics=diagnostics,
            group_poses=poses,
        )

    @staticmethod
    def _invalid(group_ids: Sequence[PlanningGroupID], message: str) -> TargetEvaluationResult:
        return TargetEvaluationResult(False, "INVALID", message, group_ids=tuple(group_ids))

    @staticmethod
    def _group_offsets(groups: Sequence[PlanningGroup]) -> tuple[tuple[PlanningGroup, int], ...]:
        offsets: list[tuple[PlanningGroup, int]] = []
        offset = 0
        for group in groups:
            offsets.append((group, offset))
            offset += len(group.joint_names)
        return tuple(offsets)

    @staticmethod
    def _pose_is_finite(pose: PoseStamped) -> bool:
        values = [*pose.position, *pose.orientation]
        return len(values) == 7 and all(math.isfinite(float(value)) for value in values)

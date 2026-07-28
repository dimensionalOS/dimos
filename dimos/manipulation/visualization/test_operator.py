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

"""Focused tests for the manipulation visualization operator facade."""

from pathlib import Path

from dimos.agents.skill_result import SkillResult
from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupDefinition
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import IKStatus, PlanningStatus
from dimos.manipulation.planning.spec.models import (
    GeneratedPlan,
    IKResult,
    PlanningGroupID,
    RobotName,
)
from dimos.manipulation.visualization.operator import (
    JointTargetRequest,
    ManipulationOperator,
    PoseTargetRequest,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint


def _robot_config(
    name: str = "arm",
    joint_names: list[str] | None = None,
    groups: tuple[PlanningGroup, ...] | None = None,
) -> RobotModelConfig:
    joints = joint_names or ["j0", "j1"]
    definitions = [
        PlanningGroupDefinition(
            name="manipulator",
            joint_names=tuple(joints),
            base_link="base",
            tip_link="tool",
        )
    ]
    if groups is not None:
        definitions = [
            PlanningGroupDefinition(
                name=group.group_name,
                joint_names=group.local_joint_names,
                base_link=group.base_link,
                tip_link=group.tip_link,
            )
            for group in groups
        ]
    return RobotModelConfig(
        name=name,
        model_path=Path("/robot.urdf"),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        joint_names=joints,
        base_link="base",
        planning_groups=definitions,
    )


class FakeModule:
    def __init__(self) -> None:
        self.state = "COMPLETED"
        self.error = ""
        self.has_plan = True
        self.plan = GeneratedPlan(
            group_ids=("arm/manipulator",),
            trajectory=JointTrajectory(
                joint_names=["arm/j0", "arm/j1"],
                points=[TrajectoryPoint(0.0, [0.0, 0.0]), TrajectoryPoint(1.25, [0.4, 0.5])],
            ),
            path=[JointState({"name": ["arm/j0", "arm/j1"], "position": [0.0, 0.0]})],
            status=PlanningStatus.SUCCESS,
        )
        self.robot_configs: dict[RobotName, RobotModelConfig] = {"arm": _robot_config()}
        self.robot_ids: dict[RobotName, str] = {"arm": "arm_id"}
        self.plan_joint_targets: list[dict[PlanningGroupID, JointState]] = []
        self.plan_pose_targets: list[
            tuple[dict[PlanningGroupID, PoseStamped], tuple[PlanningGroupID, ...]]
        ] = []
        self.ik_calls: list[
            tuple[
                dict[PlanningGroupID, PoseStamped], tuple[PlanningGroupID, ...], JointState | None
            ]
        ] = []
        self.plan_success = True
        self.preview_success = True
        self.execute_success = True
        self.cancel_success = True
        self.clear_success = True
        self.reset_success = True
        self.topology_calls = 0
        self.telemetry_calls = 0

    def get_state(self) -> str:
        return self.state

    def get_error(self) -> str:
        return self.error

    def has_planned_path(self) -> bool:
        return self.has_plan

    def get_robot_config(self, robot_name: RobotName) -> RobotModelConfig | None:
        self.topology_calls += 1
        return self.robot_configs.get(robot_name)

    def robot_id_for_name(self, robot_name: RobotName) -> str | None:
        return self.robot_ids.get(robot_name)

    def get_current_joint_state(self, robot_name: RobotName) -> JointState | None:
        self.telemetry_calls += 1
        return JointState(name=[f"{robot_name}/j0"], position=[0.0])

    def inverse_kinematics(
        self,
        pose_targets: dict[PlanningGroupID, PoseStamped],
        auxiliary_group_ids: tuple[PlanningGroupID, ...] = (),
        seed: JointState | None = None,
        check_collision: bool = True,
    ) -> IKResult:
        assert check_collision is True
        self.ik_calls.append((pose_targets, auxiliary_group_ids, seed))
        return IKResult(
            status=IKStatus.SUCCESS,
            joint_state=JointState(name=["arm/j0", "arm/j1"], position=[0.4, 0.5]),
            message="ok",
        )

    def plan_to_joint_targets(self, targets: dict[PlanningGroupID, JointState]) -> bool:
        self.plan_joint_targets.append(targets)
        return self.plan_success

    def generate_plan_to_joint_targets(
        self, targets: dict[PlanningGroupID, JointState]
    ) -> GeneratedPlan | None:
        self.plan_joint_targets.append(targets)
        return self.plan if self.plan_success else None

    def plan_to_pose_targets(
        self,
        targets: dict[PlanningGroupID, PoseStamped],
        auxiliary_groups: tuple[PlanningGroupID, ...] = (),
    ) -> bool:
        self.plan_pose_targets.append((targets, auxiliary_groups))
        return self.plan_success

    def generate_plan_to_pose_targets(
        self,
        targets: dict[PlanningGroupID, PoseStamped],
        auxiliary_groups: tuple[PlanningGroupID, ...] = (),
    ) -> GeneratedPlan | None:
        self.plan_pose_targets.append((targets, auxiliary_groups))
        return self.plan if self.plan_success else None

    def preview_plan(
        self, plan: GeneratedPlan | None = None, duration: float | None = None
    ) -> bool:
        return self.preview_success

    def execute_plan(self, plan: GeneratedPlan | None = None) -> bool:
        return self.execute_success

    def cancel(self) -> bool:
        return self.cancel_success

    def clear_planned_path(self) -> bool:
        return self.clear_success

    def reset(self) -> SkillResult[str]:
        if self.reset_success:
            return SkillResult.ok("reset")
        return SkillResult.fail("ERR", "no reset")


class FakeWorldMonitor:
    def __init__(self, registry: PlanningGroupRegistry) -> None:
        self.planning_groups = registry
        self.current_states: dict[str, JointState] = {
            "arm_id": JointState(name=["j0", "j1"], position=[0.0, 0.0])
        }
        self.valid = True
        self.cancel_preview_calls = 0
        self.telemetry_calls = 0

    def get_current_joint_state(self, robot_id: str) -> JointState | None:
        self.telemetry_calls += 1
        return self.current_states.get(robot_id)

    def is_state_valid(self, robot_id: str, joint_state: JointState) -> bool:
        return self.valid

    def get_group_ee_pose(
        self, group_id: PlanningGroupID, joint_state: JointState | None = None
    ) -> PoseStamped:
        return PoseStamped(
            frame_id="world", position=Vector3(1.0, 2.0, 3.0), orientation=Quaternion()
        )

    def cancel_preview_animation(self) -> None:
        self.cancel_preview_calls += 1


def _operator(
    config: RobotModelConfig | None = None,
) -> tuple[ManipulationOperator, FakeModule, FakeWorldMonitor]:
    robot_config = config or _robot_config()
    module = FakeModule()
    module.robot_configs = {robot_config.name: robot_config}
    module.robot_ids = {robot_config.name: f"{robot_config.name}_id"}
    monitor = FakeWorldMonitor(PlanningGroupRegistry([robot_config]))
    monitor.current_states = {
        f"{robot_config.name}_id": JointState(
            name=robot_config.joint_names, position=[0.0] * len(robot_config.joint_names)
        )
    }
    return ManipulationOperator(module, monitor), module, monitor  # type: ignore[arg-type]


def _joint_request(
    names: list[str] | None = None, positions: list[float] | None = None
) -> JointTargetRequest:
    return JointTargetRequest(
        group_ids=("arm/manipulator",),
        target=JointState(name=names or ["arm/j0", "arm/j1"], position=positions or [0.1, 0.2]),
    )


def _pose(frame_id: str = "world") -> PoseStamped:
    return PoseStamped(frame_id=frame_id, position=Vector3(0.1, 0.2, 0.3), orientation=Quaternion())


def test_status_is_compact_and_does_not_read_topology_or_telemetry() -> None:
    operator, module, monitor = _operator()

    status = operator.status()

    assert status.state == "COMPLETED"
    assert status.error == ""
    assert status.has_plan is True
    assert module.topology_calls == 0
    assert module.telemetry_calls == 0
    assert monitor.telemetry_calls == 0


def test_evaluate_joint_target_accepts_exact_global_selection_domain() -> None:
    operator, _, _ = _operator()

    result = operator.evaluate_joint_target(_joint_request())

    assert result.success is True
    assert result.status == "FEASIBLE"
    assert result.target_joints is not None
    assert list(result.target_joints.name) == ["arm/j0", "arm/j1"]
    assert list(result.target_joints.position) == [0.1, 0.2]
    assert result.group_diagnostics["arm/manipulator"] == "Target is collision-free for this robot"
    assert result.group_poses["arm/manipulator"] is not None


def test_joint_target_validation_rejects_bad_joint_requests() -> None:
    cases = [
        _joint_request(["j0", "j1"], [0.1, 0.2]),
        _joint_request(["arm/j0", "arm/j0"], [0.1, 0.2]),
        _joint_request(["arm/j0"], [0.1]),
        _joint_request(["arm/j0", "arm/j1", "arm/extra"], [0.1, 0.2, 0.3]),
        _joint_request(["arm/j0", "arm/j1"], [0.1, float("nan")]),
        JointTargetRequest(
            ("missing/manipulator",), JointState(name=["missing/j0"], position=[0.1])
        ),
        JointTargetRequest(
            ("arm/manipulator", "arm/manipulator"),
            JointState(name=["arm/j0", "arm/j1"], position=[0.1, 0.2]),
        ),
    ]
    operator, _, _ = _operator()

    for request in cases:
        result = operator.evaluate_joint_target(request)
        assert result.success is False
        assert result.status == "INVALID"


def test_joint_target_validation_rejects_overlapping_groups() -> None:
    groups = (
        PlanningGroup("arm/first", "arm", "first", ("arm/j0",), ("j0",), "base"),
        PlanningGroup("arm/second", "arm", "second", ("arm/j0",), ("j0",), "base"),
    )
    operator, _, _ = _operator(_robot_config(groups=groups))
    request = JointTargetRequest(
        ("arm/first", "arm/second"), JointState(name=["arm/j0", "arm/j0"], position=[0.1, 0.2])
    )

    result = operator.evaluate_joint_target(request)

    assert result.success is False
    assert result.status == "INVALID"


def test_pose_evaluation_accepts_world_frame_and_delegates_original_request() -> None:
    operator, module, _ = _operator()
    pose = _pose()
    seed = JointState(name=["arm/j0", "arm/j1"], position=[0.0, 0.0])
    request = PoseTargetRequest({"arm/manipulator": pose}, seed=seed)

    result = operator.evaluate_pose_target(request)

    assert result.success is True
    assert result.target_joints is not None
    assert list(result.target_joints.name) == ["arm/j0", "arm/j1"]
    assert module.ik_calls == [({"arm/manipulator": pose}, (), seed)]


def test_pose_validation_rejects_frame_capability_and_seed_errors() -> None:
    no_pose_group = (
        PlanningGroup("arm/no_pose", "arm", "no_pose", ("arm/j0",), ("j0",), "base", None),
    )
    no_pose_operator, _, _ = _operator(_robot_config(joint_names=["j0"], groups=no_pose_group))
    bad_seed_cases = [
        PoseTargetRequest({"arm/manipulator": _pose("camera")}),
        PoseTargetRequest(
            {"arm/manipulator": _pose()},
            seed=JointState(name=["j0", "j1"], position=[0.0, 0.0]),
        ),
        PoseTargetRequest(
            {"arm/manipulator": _pose()},
            seed=JointState(name=["arm/j0", "arm/j0"], position=[0.0, 0.0]),
        ),
    ]
    operator, _, _ = _operator()

    no_pose = no_pose_operator.evaluate_pose_target(PoseTargetRequest({"arm/no_pose": _pose()}))
    assert no_pose.success is False
    assert no_pose.status == "INVALID"
    for request in bad_seed_cases:
        result = operator.evaluate_pose_target(request)
        assert result.success is False
        assert result.status == "INVALID"


def test_planning_methods_return_exact_generated_plan() -> None:
    operator, module, _ = _operator()
    joint_request = _joint_request()
    pose = _pose()
    pose_request = PoseTargetRequest({"arm/manipulator": pose})

    joint_result = operator.plan_to_joints(joint_request)
    pose_result = operator.plan_to_pose(pose_request)

    assert joint_result is module.plan
    assert list(module.plan_joint_targets[0]["arm/manipulator"].name) == ["arm/j0", "arm/j1"]
    assert module.plan_pose_targets == [({"arm/manipulator": pose}, ())]
    assert pose_result is module.plan


def test_actions_return_typed_results_and_cancel_fallback_ownership() -> None:
    operator, module, monitor = _operator()

    assert operator.preview(module.plan, 0.5) is True
    assert operator.execute(module.plan) is True
    assert operator.clear_plan() is True
    assert operator.reset() is True
    cancel_result = operator.cancel()
    assert cancel_result is True
    assert monitor.cancel_preview_calls == 0

    module.cancel_success = False
    fallback = operator.cancel()
    assert fallback is False
    assert monitor.cancel_preview_calls == 0

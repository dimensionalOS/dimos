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

"""Focused tests for the single-model visualization operator."""

from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

from dimos.manipulation.manipulation_spec import (
    ExecutionResult,
    ExecutionStatus,
    OperationStatus,
)
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.planners.roboplan_config import RoboPlanCartesianPathConfig
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import IKStatus, PlanningStatus
from dimos.manipulation.planning.spec.models import GeneratedPlan, IKResult
from dimos.manipulation.visualization.operator import (
    CartesianTargetRequest,
    JointTargetRequest,
    ManipulationOperator,
    PoseTargetRequest,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.assets.model import RobotModel


def _config() -> RobotModelConfig:
    return RobotModelConfig(
        model=RobotModel.from_file(Path("/model.urdf")),
        joint_names=["left/j1", "left/j2", "right/j1"],
        base_link="base",
        planning_groups=[
            PlanningGroupDefinition("left_arm", ("left/j1", "left/j2"), "base", "left/tool"),
            PlanningGroupDefinition("right_arm", ("right/j1",), "base", "right/tool"),
        ],
    )


def _plan() -> GeneratedPlan:
    trajectory = JointTrajectory(
        joint_names=["left/j1", "left/j2"],
        points=[TrajectoryPoint(positions=[0.4, 0.5], time_from_start=1.0)],
    )
    return GeneratedPlan(
        group_ids=("left_arm",),
        trajectory=trajectory,
        path=[JointState(name=trajectory.joint_names, position=[0.4, 0.5])],
        status=PlanningStatus.SUCCESS,
    )


def _operator() -> tuple[ManipulationOperator, MagicMock, MagicMock]:
    config = _config()
    module = MagicMock()
    module.config = SimpleNamespace(default_speed_scale=1.0)
    module.get_operation_status.return_value = OperationStatus.COMPLETED
    module.get_error.return_value = ""
    module.has_planned_path.return_value = True
    module.get_model_config.return_value = config
    module.get_init_joints.return_value = None
    module.inverse_kinematics.return_value = IKResult(
        status=IKStatus.SUCCESS,
        joint_state=JointState(name=["left/j1", "left/j2"], position=[0.4, 0.5]),
    )
    module.generate_plan_to_joint_targets.return_value = _plan()
    module.generate_plan_to_pose_targets.return_value = _plan()
    module.generate_cartesian_plan.return_value = _plan()
    module.preview_plan.return_value = True
    module._execute_generated_plan.return_value = True
    module.cancel.return_value = ExecutionResult(ExecutionStatus.NO_EXECUTION)
    module.clear_planned_path.return_value = True

    monitor = MagicMock()
    monitor.planning_groups = PlanningGroupRegistry(config.planning_groups)
    monitor.get_current_joint_state.return_value = JointState(
        name=config.joint_names, position=[0.0, 0.0, 0.0]
    )
    monitor.is_state_valid.return_value = True
    monitor.get_group_ee_pose.return_value = PoseStamped(frame_id="world")
    return ManipulationOperator(module, monitor), module, monitor


def test_status_is_compact() -> None:
    operator, module, _ = _operator()
    assert operator.status().state == "COMPLETED"
    module.get_state.assert_not_called()
    module.get_model_config.assert_not_called()


def test_joint_evaluation_overlays_selected_target_on_complete_model_state() -> None:
    operator, _, monitor = _operator()
    request = JointTargetRequest(
        ("left_arm",),
        JointState(name=["left/j1", "left/j2"], position=[0.1, 0.2]),
    )
    result = operator.evaluate_joint_target(request)
    assert result.success
    complete = monitor.is_state_valid.call_args.args[0]
    assert complete.name == ["left/j1", "left/j2", "right/j1"]
    assert complete.position == [0.1, 0.2, 0.0]


def test_joint_evaluation_rejects_noncanonical_unknown_and_overlapping_selection() -> None:
    operator, _, _ = _operator()
    noncanonical = JointTargetRequest(
        ("left_arm",), JointState(name=["j1", "j2"], position=[0.1, 0.2])
    )
    unknown = JointTargetRequest(("missing",), JointState(name=["left/j1"], position=[0.1]))
    duplicate = JointTargetRequest(
        ("left_arm", "left_arm"),
        JointState(name=["left/j1", "left/j2"], position=[0.1, 0.2]),
    )
    assert all(
        not operator.evaluate_joint_target(request).success
        for request in (noncanonical, unknown, duplicate)
    )


def test_pose_evaluation_routes_group_id_and_canonical_seed() -> None:
    operator, module, _ = _operator()
    pose = PoseStamped(frame_id="world")
    seed = JointState(name=["left/j1", "left/j2"], position=[0.0, 0.0])
    result = operator.evaluate_pose_target(PoseTargetRequest({"left_arm": pose}, seed=seed))
    assert result.success
    module.inverse_kinematics.assert_called_once_with(
        pose_targets={"left_arm": pose}, auxiliary_group_ids=(), seed=seed, check_collision=True
    )


def test_pose_evaluation_rejects_non_world_frame_and_ambiguous_local_seed() -> None:
    operator, _, _ = _operator()
    bad_frame = PoseTargetRequest({"left_arm": PoseStamped(frame_id="camera")})
    bad_seed = PoseTargetRequest(
        {"left_arm": PoseStamped(frame_id="world")},
        seed=JointState(name=["j1", "j2"], position=[0.0, 0.0]),
    )
    assert not operator.evaluate_pose_target(bad_frame).success
    assert not operator.evaluate_pose_target(bad_seed).success


def test_planning_and_actions_return_exact_generated_plan() -> None:
    operator, module, _ = _operator()
    joint_request = JointTargetRequest(
        ("left_arm",), JointState(name=["left/j1", "left/j2"], position=[0.1, 0.2])
    )
    pose_request = PoseTargetRequest({"left_arm": PoseStamped(frame_id="world")})
    assert (
        operator.plan_to_joints(joint_request) is module.generate_plan_to_joint_targets.return_value
    )
    assert operator.plan_to_pose(pose_request) is module.generate_plan_to_pose_targets.return_value
    assert operator.preview(_plan(), 0.5)
    assert operator.execute(_plan())
    assert operator.cancel()
    assert operator.clear_plan()


def test_cartesian_planning_uses_current_group_pose() -> None:
    operator, module, monitor = _operator()
    target = PoseStamped(frame_id="world")
    config = RoboPlanCartesianPathConfig()
    result = operator.plan_cartesian(CartesianTargetRequest({"left_arm": target}, config))
    assert result is module.generate_cartesian_plan.return_value
    targets = module.generate_cartesian_plan.call_args.args[0]
    assert targets["left_arm"] == (monitor.get_group_ee_pose.return_value, target)

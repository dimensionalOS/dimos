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
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np

from dimos.manipulation.manipulation_spec import (
    CommandResult,
    CommandStatus,
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
from dimos.message_utils import time as message_time
from dimos.messages.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos.messages.sensor_msgs.msg import JointState
from dimos.messages.std_msgs.msg import Header
from dimos.messages.trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
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
        header=Header(stamp=message_time.time_from_seconds(time.time()), frame_id=""),
        joint_names=["left/j1", "left/j2"],
        points=[
            JointTrajectoryPoint(
                positions=np.asarray([0.4, 0.5], dtype=np.float64),
                velocities=np.asarray(np.zeros(len([0.4, 0.5])), dtype=np.float64),
                accelerations=np.asarray([], dtype=np.float64),
                effort=np.asarray([], dtype=np.float64),
                time_from_start=message_time.duration_from_seconds(1.0),
            )
        ],
    )
    return GeneratedPlan(
        group_ids=("left_arm",),
        trajectory=trajectory,
        path=[
            JointState(
                header=Header(stamp=message_time.time_from_seconds(time.time()), frame_id=""),
                name=trajectory.joint_names,
                position=np.asarray([0.4, 0.5], dtype=np.float64),
                velocity=np.asarray([], dtype=np.float64),
                effort=np.asarray([], dtype=np.float64),
            )
        ],
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
        joint_state=JointState(
            header=Header(stamp=message_time.time_from_seconds(time.time()), frame_id=""),
            name=["left/j1", "left/j2"],
            position=np.asarray([0.4, 0.5], dtype=np.float64),
            velocity=np.asarray([], dtype=np.float64),
            effort=np.asarray([], dtype=np.float64),
        ),
    )
    module.generate_plan_to_joint_targets.return_value = _plan()
    module.generate_plan_to_pose_targets.return_value = _plan()
    module.generate_cartesian_plan.return_value = _plan()
    module.preview_plan.return_value = CommandResult(CommandStatus.SUCCEEDED)
    module._execute_generated_plan.return_value = True
    module.cancel.return_value = ExecutionResult(ExecutionStatus.NO_EXECUTION)
    module.clear_planned_path.return_value = CommandResult(CommandStatus.SUCCEEDED)

    monitor = MagicMock()
    monitor.planning_groups = PlanningGroupRegistry(config.planning_groups)
    monitor.get_current_joint_state.return_value = JointState(
        header=Header(stamp=message_time.time_from_seconds(time.time()), frame_id=""),
        name=config.joint_names,
        position=np.asarray([0.0, 0.0, 0.0], dtype=np.float64),
        velocity=np.asarray([], dtype=np.float64),
        effort=np.asarray([], dtype=np.float64),
    )
    monitor.is_state_valid.return_value = True
    monitor.get_group_ee_pose.return_value = PoseStamped(
        header=Header(stamp=message_time.time_from_seconds(time.time()), frame_id="world"),
        pose=Pose(position=Point(0.0, 0.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
    )
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
        JointState(
            header=Header(stamp=message_time.time_from_seconds(time.time()), frame_id=""),
            name=["left/j1", "left/j2"],
            position=np.asarray([0.1, 0.2], dtype=np.float64),
            velocity=np.asarray([], dtype=np.float64),
            effort=np.asarray([], dtype=np.float64),
        ),
    )
    result = operator.evaluate_joint_target(request)
    assert result.success
    complete = monitor.is_state_valid.call_args.args[0]
    assert complete.name == ["left/j1", "left/j2", "right/j1"]
    assert complete.position.tolist() == [0.1, 0.2, 0.0]


def test_joint_evaluation_rejects_noncanonical_unknown_and_overlapping_selection() -> None:
    operator, _, _ = _operator()
    noncanonical = JointTargetRequest(
        ("left_arm",),
        JointState(
            header=Header(stamp=message_time.time_from_seconds(time.time()), frame_id=""),
            name=["j1", "j2"],
            position=np.asarray([0.1, 0.2], dtype=np.float64),
            velocity=np.asarray([], dtype=np.float64),
            effort=np.asarray([], dtype=np.float64),
        ),
    )
    unknown = JointTargetRequest(
        ("missing",),
        JointState(
            header=Header(stamp=message_time.time_from_seconds(time.time()), frame_id=""),
            name=["left/j1"],
            position=np.asarray([0.1], dtype=np.float64),
            velocity=np.asarray([], dtype=np.float64),
            effort=np.asarray([], dtype=np.float64),
        ),
    )
    duplicate = JointTargetRequest(
        ("left_arm", "left_arm"),
        JointState(
            header=Header(stamp=message_time.time_from_seconds(time.time()), frame_id=""),
            name=["left/j1", "left/j2"],
            position=np.asarray([0.1, 0.2], dtype=np.float64),
            velocity=np.asarray([], dtype=np.float64),
            effort=np.asarray([], dtype=np.float64),
        ),
    )
    assert all(
        not operator.evaluate_joint_target(request).success
        for request in (noncanonical, unknown, duplicate)
    )


def test_pose_evaluation_routes_group_id_and_canonical_seed() -> None:
    operator, module, _ = _operator()
    pose = PoseStamped(
        header=Header(stamp=message_time.time_from_seconds(time.time()), frame_id="world"),
        pose=Pose(position=Point(0.0, 0.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
    )
    seed = JointState(
        header=Header(stamp=message_time.time_from_seconds(time.time()), frame_id=""),
        name=["left/j1", "left/j2"],
        position=np.asarray([0.0, 0.0], dtype=np.float64),
        velocity=np.asarray([], dtype=np.float64),
        effort=np.asarray([], dtype=np.float64),
    )
    result = operator.evaluate_pose_target(PoseTargetRequest({"left_arm": pose}, seed=seed))
    assert result.success
    module.inverse_kinematics.assert_called_once()
    call = module.inverse_kinematics.call_args.kwargs
    assert call["pose_targets"] == {"left_arm": pose}
    assert call["auxiliary_group_ids"] == ()
    assert call["check_collision"] is True
    assert call["seed"].name == seed.name
    np.testing.assert_array_equal(call["seed"].position, seed.position)
    np.testing.assert_array_equal(call["seed"].velocity, seed.velocity)
    np.testing.assert_array_equal(call["seed"].effort, seed.effort)


def test_pose_evaluation_rejects_non_world_frame_and_ambiguous_local_seed() -> None:
    operator, _, _ = _operator()
    bad_frame = PoseTargetRequest(
        {
            "left_arm": PoseStamped(
                header=Header(stamp=message_time.time_from_seconds(time.time()), frame_id="camera"),
                pose=Pose(
                    position=Point(0.0, 0.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
                ),
            )
        }
    )
    bad_seed = PoseTargetRequest(
        {
            "left_arm": PoseStamped(
                header=Header(stamp=message_time.time_from_seconds(time.time()), frame_id="world"),
                pose=Pose(
                    position=Point(0.0, 0.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
                ),
            )
        },
        seed=JointState(
            header=Header(stamp=message_time.time_from_seconds(time.time()), frame_id=""),
            name=["j1", "j2"],
            position=np.asarray([0.0, 0.0], dtype=np.float64),
            velocity=np.asarray([], dtype=np.float64),
            effort=np.asarray([], dtype=np.float64),
        ),
    )
    assert not operator.evaluate_pose_target(bad_frame).success
    assert not operator.evaluate_pose_target(bad_seed).success


def test_planning_and_actions_return_exact_generated_plan() -> None:
    operator, module, _ = _operator()
    joint_request = JointTargetRequest(
        ("left_arm",),
        JointState(
            header=Header(stamp=message_time.time_from_seconds(time.time()), frame_id=""),
            name=["left/j1", "left/j2"],
            position=np.asarray([0.1, 0.2], dtype=np.float64),
            velocity=np.asarray([], dtype=np.float64),
            effort=np.asarray([], dtype=np.float64),
        ),
    )
    pose_request = PoseTargetRequest(
        {
            "left_arm": PoseStamped(
                header=Header(stamp=message_time.time_from_seconds(time.time()), frame_id="world"),
                pose=Pose(
                    position=Point(0.0, 0.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
                ),
            )
        }
    )
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
    target = PoseStamped(
        header=Header(stamp=message_time.time_from_seconds(time.time()), frame_id="world"),
        pose=Pose(position=Point(0.0, 0.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
    )
    config = RoboPlanCartesianPathConfig()
    result = operator.plan_cartesian(CartesianTargetRequest({"left_arm": target}, config))
    assert result is module.generate_cartesian_plan.return_value
    targets = module.generate_cartesian_plan.call_args.args[0]
    assert targets["left_arm"] == (monitor.get_group_ee_pose.return_value, target)

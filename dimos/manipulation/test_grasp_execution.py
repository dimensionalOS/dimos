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

import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.manipulation.grasp_execution import (
    execute_grasp_candidates,
    grounded_pick_and_place,
    pick_and_place_pointclouds,
)
from dimos.manipulation.manipulation_spec import (
    CommandResult,
    CommandStatus,
    ExecutionResult,
    ExecutionStatus,
    ManipulationSnapshot,
    ManipulationSpec,
    MoveResult,
    OperationStatus,
    PlanningGroupInfo,
    PlanningGroupState,
    PlanResult,
    PlanStatus,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header


def test_grounded_pick_and_place_grounds_supplied_observation_then_executes(
    mocker: MockerFixture,
) -> None:
    manipulation = mocker.Mock(spec=ManipulationSpec)
    grounded_segmentation = mocker.Mock()
    observation = mocker.Mock()
    observation.color = mocker.sentinel.color
    source_masks = [mocker.sentinel.source_mask]
    destination_masks = [mocker.sentinel.destination_mask]
    grounded_segmentation.segment_best.side_effect = [
        source_masks,
        destination_masks,
    ]
    source = mocker.Mock(pointcloud=mocker.sentinel.source_cloud)
    destination = mocker.Mock(pointcloud=mocker.sentinel.destination_cloud)
    project = mocker.patch(
        "dimos.manipulation.grasp_execution.project_depth",
        side_effect=[[source], [destination]],
    )
    execute = mocker.patch(
        "dimos.manipulation.grasp_execution.pick_and_place_pointclouds",
        return_value=CommandResult(CommandStatus.SUCCEEDED, "done"),
    )

    result = grounded_pick_and_place(
        manipulation,
        grounded_segmentation,
        observation,
        "cream cheese package",
        "wire basket",
        "panda/manipulator",
    )

    assert result.succeeded
    assert grounded_segmentation.segment_best.call_args_list == [
        mocker.call(mocker.sentinel.color, "cream cheese package"),
        mocker.call(mocker.sentinel.color, "wire basket"),
    ]
    assert project.call_args_list == [
        mocker.call(source_masks, observation),
        mocker.call(destination_masks, observation),
    ]
    execute.assert_called_once_with(
        manipulation,
        mocker.sentinel.source_cloud,
        mocker.sentinel.destination_cloud,
        "panda/manipulator",
    )


def test_execute_grasp_candidates_approaches_contacts_closes_and_retracts(
    mocker: MockerFixture,
) -> None:
    manipulation = mocker.Mock(spec=ManipulationSpec)
    manipulation.list_planning_groups.return_value = (
        PlanningGroupInfo("panda/manipulator", (), "base", "tcp", True),
    )
    manipulation.set_gripper_position.return_value = CommandResult(CommandStatus.SUCCEEDED)
    manipulation.get_state.return_value = ManipulationSnapshot(
        timestamp=1.0,
        operation_status=OperationStatus.IDLE,
        error=None,
        has_pending_plan=False,
        execution_status=ExecutionStatus.COMPLETED,
        groups={
            "panda/manipulator": PlanningGroupState(
                joints=None,
                end_effector_pose=None,
                gripper_position=0.012,
            )
        },
    )
    manipulation.plan_to_poses.side_effect = [
        PlanResult(PlanStatus.FAILED, "first unreachable"),
        PlanResult(PlanStatus.SUCCEEDED),
    ]
    manipulation.execute.return_value = ExecutionResult(ExecutionStatus.COMPLETED)
    manipulation.move_to_pose.return_value = MoveResult(
        PlanResult(PlanStatus.SUCCEEDED),
        ExecutionResult(ExecutionStatus.COMPLETED),
        (0.0, 0.0, 0.0),
        False,
    )
    mocker.patch("dimos.manipulation.grasp_execution.time.sleep")
    grasps = GraspCandidateArray(
        Header(12.5, "world"),
        [
            GraspCandidate(Pose(0.5, 0.0, 0.1)),
            GraspCandidate(Pose(0.6, 0.0, 0.1)),
        ],
    )

    result = execute_grasp_candidates(manipulation, grasps)

    assert result == CommandResult(
        CommandStatus.SUCCEEDED,
        "Executed grasp candidate 2 at 0.0120 m aperture",
    )
    assert manipulation.plan_to_poses.call_count == 2
    assert manipulation.execute.call_args.kwargs == {"blocking": True}
    assert manipulation.move_to_pose.call_count == 2
    assert manipulation.set_gripper_position.call_args_list == [
        mocker.call(0.04, "panda/manipulator"),
        mocker.call(0.0, "panda/manipulator"),
    ]


def test_execute_grasp_candidates_rejects_an_empty_array(mocker: MockerFixture) -> None:
    manipulation = mocker.Mock(spec=ManipulationSpec)

    result = execute_grasp_candidates(manipulation, GraspCandidateArray())

    assert result == CommandResult(CommandStatus.REJECTED, "No grasp candidates were provided")
    manipulation.list_planning_groups.assert_not_called()


def test_execute_grasp_candidates_retries_after_an_empty_closure(
    mocker: MockerFixture,
) -> None:
    manipulation = mocker.Mock(spec=ManipulationSpec)
    manipulation.list_planning_groups.return_value = (
        PlanningGroupInfo("panda/manipulator", (), "base", "tcp", True),
    )
    manipulation.set_gripper_position.return_value = CommandResult(CommandStatus.SUCCEEDED)
    manipulation.plan_to_poses.return_value = PlanResult(PlanStatus.SUCCEEDED)
    manipulation.execute.return_value = ExecutionResult(ExecutionStatus.COMPLETED)
    manipulation.move_to_pose.return_value = MoveResult(
        PlanResult(PlanStatus.SUCCEEDED),
        ExecutionResult(ExecutionStatus.COMPLETED),
        (0.0, 0.0, 0.0),
        False,
    )
    manipulation.get_state.side_effect = [
        ManipulationSnapshot(
            1.0,
            OperationStatus.IDLE,
            None,
            False,
            ExecutionStatus.COMPLETED,
            {"panda/manipulator": PlanningGroupState(None, None, 0.0)},
        ),
        ManipulationSnapshot(
            2.0,
            OperationStatus.IDLE,
            None,
            False,
            ExecutionStatus.COMPLETED,
            {"panda/manipulator": PlanningGroupState(None, None, 0.009)},
        ),
    ]
    mocker.patch("dimos.manipulation.grasp_execution.time.sleep")
    grasps = GraspCandidateArray(
        Header(12.5, "world"),
        [
            GraspCandidate(Pose(0.5, 0.0, 0.1)),
            GraspCandidate(Pose(0.6, 0.0, 0.1)),
        ],
    )

    result = execute_grasp_candidates(manipulation, grasps)

    assert result.succeeded
    assert "candidate 2" in result.message
    assert manipulation.plan_to_poses.call_count == 2
    assert manipulation.set_gripper_position.call_args_list == [
        mocker.call(0.04, "panda/manipulator"),
        mocker.call(0.0, "panda/manipulator"),
        mocker.call(0.04, "panda/manipulator"),
        mocker.call(0.0, "panda/manipulator"),
    ]


def test_pick_and_place_pointclouds_aligns_closing_axis_and_verifies_after_retract(
    mocker: MockerFixture,
) -> None:
    manipulation = mocker.Mock(spec=ManipulationSpec)
    group_id = "panda/manipulator"
    manipulation.list_planning_groups.return_value = (
        PlanningGroupInfo(group_id, (), "base", "tcp", True),
    )
    current_pose = PoseStamped(
        frame_id="world",
        position=(0.4, 0.0, 0.4),
        orientation=(1.0, 0.0, 0.0, 0.0),
    )
    manipulation.get_state.side_effect = [
        ManipulationSnapshot(
            1.0,
            OperationStatus.IDLE,
            None,
            False,
            ExecutionStatus.COMPLETED,
            {group_id: PlanningGroupState(None, current_pose, 0.04)},
        ),
        ManipulationSnapshot(
            2.0,
            OperationStatus.IDLE,
            None,
            False,
            ExecutionStatus.COMPLETED,
            {group_id: PlanningGroupState(None, current_pose, 0.012)},
        ),
    ]
    manipulation.set_gripper_position.return_value = CommandResult(CommandStatus.SUCCEEDED)
    manipulation.plan_to_poses.return_value = PlanResult(PlanStatus.SUCCEEDED)
    manipulation.execute.return_value = ExecutionResult(ExecutionStatus.COMPLETED)
    manipulation.move_to_pose.return_value = MoveResult(
        PlanResult(PlanStatus.SUCCEEDED),
        ExecutionResult(ExecutionStatus.COMPLETED),
        (0.0, 0.0, 0.0),
        False,
    )
    mocker.patch("dimos.manipulation.grasp_execution.time.sleep")
    source = PointCloud2.from_numpy(
        np.asarray(
            [
                [0.46, -0.01, 0.04],
                [0.46, 0.01, 0.10],
                [0.54, -0.01, 0.04],
                [0.54, 0.01, 0.10],
            ],
            dtype=np.float32,
        ),
        frame_id="world",
        timestamp=3.0,
    )
    receptacle = PointCloud2.from_numpy(
        np.asarray(
            [
                [0.59, 0.19, 0.03],
                [0.59, 0.19, 0.13],
                [0.61, 0.21, 0.13],
            ],
            dtype=np.float32,
        ),
        frame_id="world",
        timestamp=3.0,
    )

    result = pick_and_place_pointclouds(manipulation, source, receptacle)

    assert result.succeeded
    assert manipulation.plan_to_poses.call_count == 2
    planned_targets = [call.args[0][group_id] for call in manipulation.plan_to_poses.call_args_list]
    assert planned_targets[0].position.z == pytest.approx(0.15)
    assert planned_targets[1].position.z == pytest.approx(0.175)
    closing_axes = [target.orientation.to_rotation_matrix()[:, 0] for target in planned_targets]
    assert all(abs(axis[1]) == pytest.approx(1.0) for axis in closing_axes)
    assert all(abs(axis[0]) == pytest.approx(0.0) for axis in closing_axes)
    assert manipulation.move_to_pose.call_count == 4
    assert manipulation.get_state.call_count == 2
    place_target = manipulation.move_to_pose.call_args_list[-2].args[0]
    retreat_target = manipulation.move_to_pose.call_args_list[-1].args[0]
    assert place_target.position.x == pytest.approx(0.60)
    assert place_target.position.y == pytest.approx(0.20)
    assert place_target.position.z == pytest.approx(0.095)
    assert retreat_target.position.z == pytest.approx(0.175)
    assert manipulation.set_gripper_position.call_args_list == [
        mocker.call(0.04, group_id),
        mocker.call(0.0, group_id),
        mocker.call(0.04, group_id),
    ]

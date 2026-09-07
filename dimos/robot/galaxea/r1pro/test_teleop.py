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

"""Construction tests for mock-first R1 Pro Quest teleoperation."""

from typing import Any

import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.control.tasks.pose_target_ik import PinkPoseTargetSolver, PoseTargetIKTaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import Blueprint
from dimos.core.global_config import global_config
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.galaxea.r1pro.blueprints.manipulation.hosted_teleop import (
    R1PRO_ARM_ONLY_JOINTS,
)
from dimos.robot.galaxea.r1pro.blueprints.manipulation.teleop import (
    R1PRO_QUEST_TASK_NAME,
    coordinator_teleop_r1pro,
)
from dimos.robot.galaxea.r1pro.config import (
    R1PRO_PLANAR_BASE,
    R1PRO_UPPER_BODY_PLANNING_JOINTS,
    make_r1pro_model_config,
)
from dimos.robot.galaxea.r1pro.ready_pose import READY_POSE
from dimos.robot.galaxea.r1pro.teleop_ik import (
    NOMINAL_POSTURE,
    POSTURE_WEIGHTS,
    R1PRO_TELEOP_PINK,
    R1ProPinkPoseTargetSolver,
)
from dimos.teleop.quest.blueprints import teleop_quest_r1pro
from dimos.teleop.quest.quest_extensions import HeadsetArmTeleopModule
from dimos.utils.transform_utils import pose_to_matrix


def _module_kwargs(blueprint: Blueprint, module_type: type) -> dict[str, Any]:
    return next(atom.kwargs for atom in blueprint.blueprints if atom.module is module_type)


def test_r1pro_quest_blueprint_controls_only_arms_and_torso() -> None:
    coordinator = _module_kwargs(coordinator_teleop_r1pro, TeleopControlCoordinator)
    manipulation = _module_kwargs(coordinator_teleop_r1pro, ManipulationModule)
    task = coordinator["tasks"][0]

    assert len(coordinator["tasks"]) == 2
    assert coordinator["hardware"][0].adapter_type == "mock_whole_body"
    assert coordinator["hardware"][0].joints == list(R1PRO_UPPER_BODY_PLANNING_JOINTS)
    assert task.name == R1PRO_QUEST_TASK_NAME
    assert task.type == "teleop_ik"
    assert task.joint_names == list(R1PRO_UPPER_BODY_PLANNING_JOINTS)
    assert set(task.joint_names).isdisjoint(R1PRO_PLANAR_BASE.joint_names)
    assert task.params["bindings"] == [
        {"hand": "left", "target_frame": "left_gripper_link"},
        {"hand": "right", "target_frame": "right_gripper_link"},
    ]
    assert task.params["head_target_frame"] == "head_link"
    assert task.params["solver_type"] is R1ProPinkPoseTargetSolver
    # Viser must be reachable off-robot: with mock hardware it is the only
    # place this blueprint's motion is visible.
    assert manipulation["visualization"].backend == "viser"
    assert manipulation["visualization"].host == global_config.listen_host
    assert teleop_quest_r1pro.remapping_map == {
        (HeadsetArmTeleopModule.name, "left_controller_output"): "left_cartesian_command",
        (HeadsetArmTeleopModule.name, "right_controller_output"): "right_cartesian_command",
        (HeadsetArmTeleopModule.name, "headset_output"): "head_cartesian_command",
    }


def test_r1pro_quest_blueprint_wires_upper_body_plan_execution() -> None:
    coordinator = _module_kwargs(coordinator_teleop_r1pro, TeleopControlCoordinator)
    teleop_task = next(task for task in coordinator["tasks"] if task.type == "teleop_ik")
    trajectory_task = next(
        (task for task in coordinator["tasks"] if task.type == "trajectory"),
        None,
    )

    assert trajectory_task is not None
    assert trajectory_task.name == JOINT_TRAJECTORY_TASK_NAME
    assert trajectory_task.joint_names == list(R1PRO_UPPER_BODY_PLANNING_JOINTS)
    assert trajectory_task.priority > teleop_task.priority


def test_r1pro_head_task_frees_fore_aft_translation_and_roll(
    mocker: MockerFixture,
) -> None:
    left_task = mocker.Mock()
    right_task = mocker.Mock()
    head_task = mocker.Mock()
    tasks = {
        "frame/left_gripper_link": left_task,
        "frame/right_gripper_link": right_task,
        "frame/head_link": head_task,
        # The solver now insists on one; the tray posture is asserted separately.
        "posture/current": mocker.Mock(),
    }
    mocker.patch.object(PinkPoseTargetSolver, "_create_tasks", return_value=tasks)
    mocker.patch.object(R1ProPinkPoseTargetSolver, "_validate_frame_targets")
    solver = R1ProPinkPoseTargetSolver(
        PoseTargetIKTaskConfig(
            joint_names=R1PRO_UPPER_BODY_PLANNING_JOINTS,
            robot_model=make_r1pro_model_config(),
            target_frames=("left_gripper_link", "right_gripper_link", "head_link"),
            pink=PinkKinematicsConfig(position_cost=8.0, orientation_cost=2.0),
        )
    )

    configuration = mocker.Mock()
    configuration.model.nq = len(NOMINAL_POSTURE)
    result = solver._create_tasks(
        configuration,
        ("left_gripper_link", "right_gripper_link", "head_link"),
    )

    assert result is tasks
    # x is free: the torso is four revolute joints, so head_link cannot change
    # height without swinging forward. y and z are held, which is what makes
    # the jog a clean vertical move that never leaves the sagittal plane.
    np.testing.assert_array_equal(
        head_task.set_position_cost.call_args.args[0],
        np.array([0.0, 8.0, 8.0]),
    )
    np.testing.assert_array_equal(
        head_task.set_orientation_cost.call_args.args[0],
        np.array([0.0, 2.0, 2.0]),
    )
    left_task.set_position_cost.assert_not_called()
    right_task.set_position_cost.assert_not_called()


def _tray_solver_config() -> PoseTargetIKTaskConfig:
    return PoseTargetIKTaskConfig(
        joint_names=R1PRO_ARM_ONLY_JOINTS,
        robot_model=make_r1pro_model_config(),
        target_frames=("left_gripper_link", "right_gripper_link"),
        pink=R1PRO_TELEOP_PINK,
    )


def _tray_seed() -> JointState:
    return JointState(
        name=list(R1PRO_ARM_ONLY_JOINTS),
        position=[READY_POSE[joint] for joint in R1PRO_ARM_ONLY_JOINTS],
    )


@pytest.mark.self_hosted
def test_r1pro_teleop_aims_the_posture_task_at_the_tray_pose() -> None:
    """The shared solver regularizes toward wherever the robot already is,
    which holds no posture at all. R1 Pro has to aim at the boot pose instead."""
    solver = R1ProPinkPoseTargetSolver(_tray_solver_config())
    seed = _tray_seed()
    targets = solver.frame_poses(seed, list(solver._control_config.target_frames))

    solver.step(targets, seed, 0.01)

    tasks = next(iter(solver._control_contexts.values())).tasks
    assert tasks is not None
    np.testing.assert_allclose(
        tasks["posture/current"].cost,
        R1PRO_TELEOP_PINK.posture_cost * POSTURE_WEIGHTS,
    )
    # Both wrists nearly free, so the operator keeps the orientation they aim.
    assert POSTURE_WEIGHTS[-1] < 0.5 * POSTURE_WEIGHTS[4]
    # Started at the tray pose, so the target is the tray pose exactly.
    np.testing.assert_allclose(tasks["posture/current"].target_q, NOMINAL_POSTURE, atol=1e-9)


@pytest.mark.self_hosted
def test_r1pro_tray_posture_pull_stays_inside_the_streaming_envelope() -> None:
    """Off-posture arms must not cost hand tracking.

    Pink turns posture error straight into a demanded joint step. Unbounded,
    a far-from-posture arm demands more motion than the streaming velocity
    envelope passes, the envelope clips it per joint, and the clipping rotates
    the whole QP solution: the hands sit ~100 mm off target for as long as the
    arms are away from the tray pose. The approach limit is what prevents that.
    """
    solver = R1ProPinkPoseTargetSolver(_tray_solver_config())
    frames = list(solver._control_config.target_frames)
    # A chicken-winged arm: elbows flared and rolled, far from the tray pose.
    wandered = np.array([-0.35, 1.30, 1.10, 0.45, -0.90, 0.35, 0.60])
    mirrored = wandered * np.array([1.0, -1.0, -1.0, 1.0, -1.0, 1.0, -1.0])
    state = JointState(
        name=list(R1PRO_ARM_ONLY_JOINTS),
        position=(
            np.array([READY_POSE[joint] for joint in R1PRO_ARM_ONLY_JOINTS])
            + np.concatenate([wandered, mirrored])
        ).tolist(),
    )
    # Pin the hands exactly where they already are, so every millimetre the
    # solver gives up is the posture task's doing.
    targets = solver.frame_poses(state, frames)

    for _ in range(400):
        command = solver.step(targets, state, 0.01)
        assert command is not None
        state = JointState(name=list(command.name), position=list(command.position))

    reached = solver.frame_poses(state, frames)
    for frame in frames:
        drift = np.linalg.norm(
            np.asarray(pose_to_matrix(reached[frame]))[:3, 3]
            - np.asarray(pose_to_matrix(targets[frame]))[:3, 3]
        )
        assert drift < 0.02, f"{frame} drifted {drift * 1000:.1f} mm off a pinned target"

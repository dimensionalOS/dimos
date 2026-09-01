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
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.robot.galaxea.r1pro.blueprints.manipulation.teleop import (
    R1PRO_QUEST_TASK_NAME,
    R1ProTeleopCoordinator,
    coordinator_teleop_r1pro,
)
from dimos.robot.galaxea.r1pro.config import (
    R1PRO_PLANAR_BASE,
    R1PRO_UPPER_BODY_PLANNING_JOINTS,
    make_r1pro_model_config,
)
from dimos.robot.galaxea.r1pro.teleop_ik import R1ProPinkPoseTargetSolver
from dimos.teleop.quest.blueprints import teleop_quest_r1pro
from dimos.teleop.quest.quest_extensions import HeadsetArmTeleopModule


def _module_kwargs(blueprint: Blueprint, module_type: type) -> dict[str, Any]:
    return next(atom.kwargs for atom in blueprint.blueprints if atom.module is module_type)


def test_r1pro_quest_blueprint_controls_only_arms_and_torso() -> None:
    coordinator = _module_kwargs(coordinator_teleop_r1pro, R1ProTeleopCoordinator)
    manipulation = _module_kwargs(coordinator_teleop_r1pro, ManipulationModule)
    task = coordinator["tasks"][0]

    assert len(coordinator["tasks"]) == 1
    assert coordinator["hardware"][0].adapter_type == "mock_whole_body"
    assert coordinator["hardware"][0].joints == list(R1PRO_UPPER_BODY_PLANNING_JOINTS)
    assert task.name == R1PRO_QUEST_TASK_NAME
    assert task.type == "teleop_ik"
    assert task.joint_names == R1PRO_UPPER_BODY_PLANNING_JOINTS
    assert set(task.joint_names).isdisjoint(R1PRO_PLANAR_BASE.joint_names)
    assert task.params["bindings"] == [
        {"hand": "left", "target_frame": "left_gripper_link"},
        {"hand": "right", "target_frame": "right_gripper_link"},
    ]
    assert task.params["head_target_frame"] == "head_link"
    assert task.params["solver_type"] is R1ProPinkPoseTargetSolver
    assert manipulation["visualization"] == {"backend": "viser"}
    assert teleop_quest_r1pro.remapping_map == {
        (HeadsetArmTeleopModule.name, "left_controller_output"): "left_cartesian_command",
        (HeadsetArmTeleopModule.name, "right_controller_output"): "right_cartesian_command",
        (HeadsetArmTeleopModule.name, "headset_output"): "head_cartesian_command",
    }


def test_r1pro_quest_exposes_module_cli_teleop_mode() -> None:
    parser = BlueprintConfigParser(teleop_quest_r1pro)

    parsed = parser.parse(["--teleop-mode", "hands"], environ={})
    qualified = parser.parse(
        ["--controlcoordinator.teleop-mode", "hands"],
        environ={},
    )

    assert parsed.module_kwargs("ControlCoordinator")["teleop_mode"] == "hands"
    assert qualified.module_kwargs("ControlCoordinator")["teleop_mode"] == "hands"
    assert (
        "--teleop-mode, --ControlCoordinator.teleop-mode "
        "<Literal['headset', 'hands']> (default: headset)" in parser.format_help()
    )


def test_r1pro_quest_rejects_invalid_teleop_mode() -> None:
    parser = BlueprintConfigParser(teleop_quest_r1pro)

    with pytest.raises(BlueprintConfigError, match="headset.*hands"):
        parser.parse(["--teleop-mode", "arms"], environ={})


def test_r1pro_hands_mode_omits_head_target_and_keeps_upper_body(
    mocker: MockerFixture,
) -> None:
    kwargs = _module_kwargs(coordinator_teleop_r1pro, R1ProTeleopCoordinator)
    coordinator = R1ProTeleopCoordinator(**kwargs, teleop_mode="hands")
    mocker.patch.object(TeleopControlCoordinator, "_setup_from_config")

    try:
        coordinator._setup_from_config()
        task = coordinator.config.tasks[0]

        assert "head_target_frame" not in task.params
        assert task.joint_names == R1PRO_UPPER_BODY_JOINTS
        assert set(task.joint_names).isdisjoint(R1PRO_PLANAR_BASE.joint_names)
    finally:
        coordinator.stop()


def test_r1pro_head_task_ignores_lateral_translation_and_roll(
    mocker: MockerFixture,
) -> None:
    left_task = mocker.Mock()
    right_task = mocker.Mock()
    head_task = mocker.Mock()
    tasks = {
        "frame/left_gripper_link": left_task,
        "frame/right_gripper_link": right_task,
        "frame/head_link": head_task,
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

    result = solver._create_tasks(
        mocker.Mock(),
        ("left_gripper_link", "right_gripper_link", "head_link"),
    )

    assert result is tasks
    np.testing.assert_array_equal(
        head_task.set_position_cost.call_args.args[0],
        np.array([8.0, 0.0, 8.0]),
    )
    np.testing.assert_array_equal(
        head_task.set_orientation_cost.call_args.args[0],
        np.array([0.0, 2.0, 2.0]),
    )
    left_task.set_position_cost.assert_not_called()
    right_task.set_position_cost.assert_not_called()


def test_r1pro_hands_only_solver_does_not_require_head_task(
    mocker: MockerFixture,
) -> None:
    left_task = mocker.Mock()
    right_task = mocker.Mock()
    tasks = {
        "frame/left_gripper_link": left_task,
        "frame/right_gripper_link": right_task,
    }
    mocker.patch.object(PinkPoseTargetSolver, "_create_tasks", return_value=tasks)
    mocker.patch.object(R1ProPinkPoseTargetSolver, "_validate_frame_targets")
    solver = R1ProPinkPoseTargetSolver(
        PoseTargetIKTaskConfig(
            joint_names=tuple(R1PRO_UPPER_BODY_JOINTS),
            robot_model=make_r1pro_model_config(),
            target_frames=("left_gripper_link", "right_gripper_link"),
            pink=PinkKinematicsConfig(position_cost=8.0, orientation_cost=2.0),
        )
    )

    result = solver._create_tasks(
        mocker.Mock(),
        ("left_gripper_link", "right_gripper_link"),
    )

    assert result is tasks
    left_task.set_position_cost.assert_not_called()
    right_task.set_position_cost.assert_not_called()

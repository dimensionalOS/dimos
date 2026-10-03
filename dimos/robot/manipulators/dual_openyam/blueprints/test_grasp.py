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
from dimos.control.coordinator import TaskConfig
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.robot.manipulators.dual_openyam.blueprints.basic import DualOpenYamCoordinator
from dimos.robot.manipulators.dual_openyam.blueprints.grasp import (
    dual_openyam_grasp,
    dual_openyam_grasp_model_config,
)


def _atom(blueprint, module):  # type: ignore[no-untyped-def]
    return next(atom for atom in blueprint.active_blueprints if atom.module is module)


def test_each_arm_plans_to_its_fingertips_with_its_own_gripper() -> None:
    config = dual_openyam_grasp_model_config()

    assert config.base_pose.frame_id == "world"
    assert {group.name: group.tip_link for group in config.planning_groups} == {
        "left_manipulator": "left_tcp",
        "right_manipulator": "right_tcp",
    }
    assert {group.gripper_hardware_id for group in config.planning_groups} == {
        "left_arm",
        "right_arm",
    }
    assert config.model.load().get_joint("left_joint1") is not None


def test_grasp_blueprint_composes_both_gripper_tasks_and_the_yam_grasp_settings() -> None:
    tasks: list[TaskConfig] = _atom(dual_openyam_grasp, DualOpenYamCoordinator).kwargs["tasks"]
    assert {task.name for task in tasks if task.type == "gripper"} == {
        "left_arm_gripper",
        "right_arm_gripper",
    }
    assert _atom(dual_openyam_grasp, PickAndPlaceModule).kwargs["pregrasp_along_tool_z"] is True
    assert _atom(dual_openyam_grasp, HeuristicGraspModule).kwargs["yaw_candidates"] == 8
    manipulation = _atom(dual_openyam_grasp, ManipulationModule).kwargs
    assert manipulation["world_frame"] == "world"
    assert manipulation["static_transforms"][0].child_frame_id == "camera_link"

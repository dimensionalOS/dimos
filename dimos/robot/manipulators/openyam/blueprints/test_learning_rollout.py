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

from typing import Any

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import (
    JOINT_TRAJECTORY_TASK_NAME,
)
from dimos.core.coordination.blueprints import Blueprint
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.imitation.policy.lerobot.module import (
    POLICY_ROLLOUT_TASK_NAME,
    LeRobotPolicyModule,
)
from dimos.robot.manipulators.openyam.blueprints.learning_rollout import (
    learning_rollout_quest_openyam,
)
from dimos.robot.manipulators.openyam.config import OPENYAM_JOINTS
from dimos.robot.manipulators.openyam.learning import OPENYAM_LEARNING_PROFILE


def _module_kwargs(blueprint: Blueprint, module_type: type) -> dict[str, Any]:
    return next(
        atom.kwargs for atom in blueprint.blueprints if issubclass(atom.module, module_type)
    )


def test_rollout_uses_one_low_priority_seven_joint_trajectory_task() -> None:
    coordinator = _module_kwargs(learning_rollout_quest_openyam, ControlCoordinator)
    policy = next(task for task in coordinator["tasks"] if task.name == POLICY_ROLLOUT_TASK_NAME)
    teleop = next(task for task in coordinator["tasks"] if task.name == "teleop_openyam")
    trajectory = next(
        task for task in coordinator["tasks"] if task.name == JOINT_TRAJECTORY_TASK_NAME
    )

    assert policy.type == "trajectory"
    assert policy.joint_names == OPENYAM_JOINTS
    assert policy.priority < teleop.priority < trajectory.priority


def test_rollout_uses_the_shared_learning_profile() -> None:
    policy = _module_kwargs(learning_rollout_quest_openyam, LeRobotPolicyModule)
    camera = _module_kwargs(learning_rollout_quest_openyam, CameraModule)

    assert policy["fps"] == OPENYAM_LEARNING_PROFILE.fps
    assert policy["joint_names"] == list(OPENYAM_LEARNING_PROFILE.joint_names)
    assert policy["trajectory_task_name"] == POLICY_ROLLOUT_TASK_NAME
    assert camera["hardware"].fps == OPENYAM_LEARNING_PROFILE.fps
    assert camera["hardware"].width == OPENYAM_LEARNING_PROFILE.camera_width
    assert camera["hardware"].height == OPENYAM_LEARNING_PROFILE.camera_height

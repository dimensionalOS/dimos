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
from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.core.coordination.blueprints import Blueprint
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.imitation.policy.lerobot.module import OpenYamLeRobotPolicy
from dimos.imitation.policy.module import (
    POLICY_ROLLOUT_INSTANCE_NAME,
    POLICY_ROLLOUT_TASK_NAME,
)
from dimos.robot.manipulators.openyam.blueprints.learning_rollout import (
    build_openyam_rollout,
)
from dimos.robot.manipulators.openyam.config import OPENYAM_JOINTS
from dimos.robot.manipulators.openyam.learning import OPENYAM_CAMERA_SHAPE, OPENYAM_FPS
from dimos.teleop.quest.quest_extensions import ArmTeleopModule


def _module_kwargs(blueprint: Blueprint, module_type: type) -> dict[str, Any]:
    return next(
        atom.kwargs for atom in blueprint.blueprints if issubclass(atom.module, module_type)
    )


def test_default_rollout_is_quest_free() -> None:
    blueprint = build_openyam_rollout(
        artifact="checkpoint",
        task="pick up block",
        cameras={"wrist_image": 0},
    )
    modules = [atom.module for atom in blueprint.active_blueprints]
    coordinator = _module_kwargs(blueprint, ControlCoordinator)
    policy_task = next(
        task for task in coordinator["tasks"] if task.name == POLICY_ROLLOUT_TASK_NAME
    )

    assert ArmTeleopModule not in modules
    assert len(coordinator["tasks"]) == 1
    assert policy_task.type == "trajectory"
    assert policy_task.joint_names == OPENYAM_JOINTS


def test_quest_rollout_adds_higher_priority_takeover() -> None:
    blueprint = build_openyam_rollout(
        artifact="checkpoint",
        task="pick up block",
        cameras={"wrist_image": 0},
        quest_control=True,
    )
    coordinator = _module_kwargs(blueprint, ControlCoordinator)
    policy = next(task for task in coordinator["tasks"] if task.name == POLICY_ROLLOUT_TASK_NAME)
    teleop = next(task for task in coordinator["tasks"] if task.name == "teleop_openyam")
    trajectory = next(
        task for task in coordinator["tasks"] if task.name == JOINT_TRAJECTORY_TASK_NAME
    )

    assert policy.priority < teleop.priority < trajectory.priority


def test_rollout_binds_profile_camera_without_changing_autoconnect() -> None:
    blueprint = build_openyam_rollout(
        artifact="checkpoint",
        task="pick up block",
        cameras={"wrist_image": "/dev/camera"},
        device="cuda",
    )
    policy = _module_kwargs(blueprint, OpenYamLeRobotPolicy)
    camera = _module_kwargs(blueprint, CameraModule)

    assert policy["instance_name"] == POLICY_ROLLOUT_INSTANCE_NAME
    assert policy["artifact"] == "checkpoint"
    assert policy["task"] == "pick up block"
    assert policy["device"] == "cuda"
    assert camera["hardware"].fps == OPENYAM_FPS
    assert (camera["hardware"].height, camera["hardware"].width, 3) == OPENYAM_CAMERA_SHAPE
    assert camera["hardware"].camera_index == "/dev/camera"
    assert blueprint.remapping_map[("PolicyCamera_wrist_image", "color_image")] == "wrist_image"

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
from dimos.control.tasks.registry import control_task_registry
from dimos.control.tasks.trajectory_task.trajectory_task import (
    JOINT_TRAJECTORY_TASK_NAME,
    JointTrajectoryTask,
)
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.imitation.policy.lerobot.module import LeRobotPolicyModule
from dimos.imitation.policy.rollout_supervisor import (
    POLICY_GRIPPER_TASK_NAME,
    POLICY_ROLLOUT_TASK_NAME,
)
from dimos.robot.manipulators.openyam.blueprints.learning_rollout import (
    learning_rollout_quest_openyam,
)
from dimos.robot.manipulators.openyam.config import (
    OPENYAM_ARM_JOINTS,
    OPENYAM_GRIPPER_JOINT,
)
from dimos.robot.manipulators.openyam.learning import OPENYAM_LEARNING_PROFILE


def _module_kwargs(blueprint: Blueprint, module_type: type) -> dict[str, Any]:
    return next(
        atom.kwargs for atom in blueprint.blueprints if issubclass(atom.module, module_type)
    )


def test_rollout_routes_policy_to_inactive_low_priority_tasks() -> None:
    coordinator = _module_kwargs(learning_rollout_quest_openyam, ControlCoordinator)
    policy_arm = next(
        task for task in coordinator["tasks"] if task.name == POLICY_ROLLOUT_TASK_NAME
    )
    policy_gripper = next(
        task for task in coordinator["tasks"] if task.name == POLICY_GRIPPER_TASK_NAME
    )
    teleop = next(task for task in coordinator["tasks"] if task.name == "teleop_openyam")
    trajectory = next(
        task for task in coordinator["tasks"] if task.name == JOINT_TRAJECTORY_TASK_NAME
    )

    assert policy_arm.type == "trajectory"
    assert policy_arm.joint_names == OPENYAM_ARM_JOINTS
    assert policy_arm.priority == 10
    assert policy_arm.params == {"requires_activation": True}
    assert policy_arm.stream_bind == {"joint_command": "policy_joint_command"}
    assert policy_gripper.type == "gripper"
    assert policy_gripper.joint_names == [OPENYAM_GRIPPER_JOINT]
    assert policy_gripper.priority == 10
    assert policy_gripper.params == {"hold_duration": 0.1, "requires_activation": True}
    assert policy_gripper.stream_bind == {"gripper_command": "policy_gripper_command"}
    assert teleop.priority == 20
    assert trajectory.priority == 30

    task = control_task_registry.create(policy_arm.type, policy_arm)
    assert isinstance(task, JointTrajectoryTask)
    assert task.name == POLICY_ROLLOUT_TASK_NAME
    assert not task.is_active()


def test_rollout_uses_the_shared_learning_profile() -> None:
    policy = _module_kwargs(learning_rollout_quest_openyam, LeRobotPolicyModule)
    camera = _module_kwargs(learning_rollout_quest_openyam, CameraModule)

    assert policy["fps"] == OPENYAM_LEARNING_PROFILE.fps
    assert policy["joint_names"] == list(OPENYAM_LEARNING_PROFILE.joint_names)
    assert policy["gripper_joint_name"] == OPENYAM_LEARNING_PROFILE.gripper_joint_name
    assert camera["hardware"].fps == OPENYAM_LEARNING_PROFILE.fps
    assert camera["hardware"].width == OPENYAM_LEARNING_PROFILE.camera_width
    assert camera["hardware"].height == OPENYAM_LEARNING_PROFILE.camera_height


def test_rollout_requires_policy_path_from_cli() -> None:
    parsed = BlueprintConfigParser(learning_rollout_quest_openyam).parse(
        ["--LeRobotPolicyModule.policy-path", "outputs/checkpoint"],
        environ={},
    )

    assert parsed.module_kwargs("lerobotpolicymodule")["policy_path"] == "outputs/checkpoint"

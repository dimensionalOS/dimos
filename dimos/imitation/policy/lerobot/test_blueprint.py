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
from dimos.imitation.policy.lerobot.blueprint import (
    OPENYAM_POLICY_FPS,
    learning_rollout_quest_openyam,
)
from dimos.imitation.policy.lerobot.module import LeRobotPolicyModule
from dimos.robot.manipulators.openyam.config import (
    OPENYAM_ARM_JOINTS,
    OPENYAM_GRIPPER_JOINT,
)


def _module_kwargs(blueprint: Blueprint, module_type: type) -> dict[str, Any]:
    return next(
        atom.kwargs for atom in blueprint.blueprints if issubclass(atom.module, module_type)
    )


def test_rollout_blueprint_routes_policy_to_dedicated_low_priority_tasks() -> None:
    coordinator = _module_kwargs(learning_rollout_quest_openyam, ControlCoordinator)
    policy_arm = next(task for task in coordinator["tasks"] if task.name == "policy_rollout")
    policy_gripper = next(task for task in coordinator["tasks"] if task.name == "policy_gripper")
    teleop = next(task for task in coordinator["tasks"] if task.name == "teleop_openyam")
    trajectory = next(
        task for task in coordinator["tasks"] if task.name == JOINT_TRAJECTORY_TASK_NAME
    )

    assert policy_arm.type == "trajectory"
    assert policy_arm.joint_names == OPENYAM_ARM_JOINTS
    assert policy_arm.priority == 10
    assert policy_arm.params == {}
    assert policy_arm.stream_bind == {"joint_command": "policy_joint_command"}
    assert policy_gripper.type == "gripper"
    assert policy_gripper.joint_names == [OPENYAM_GRIPPER_JOINT]
    assert policy_gripper.priority == 10
    assert policy_gripper.params == {"hold_duration": 0.1}
    assert policy_gripper.stream_bind == {"gripper_command": "policy_gripper_command"}
    assert teleop.priority == 20
    assert trajectory.priority == 30

    task = control_task_registry.create(policy_arm.type, policy_arm)
    assert isinstance(task, JointTrajectoryTask)
    assert task.name == "policy_rollout"


def test_rollout_blueprint_uses_one_30hz_policy_and_camera_cadence() -> None:
    policy = _module_kwargs(learning_rollout_quest_openyam, LeRobotPolicyModule)
    camera = _module_kwargs(learning_rollout_quest_openyam, CameraModule)

    assert policy["fps"] == OPENYAM_POLICY_FPS == 30.0
    assert policy["joint_names"] == [*OPENYAM_ARM_JOINTS, OPENYAM_GRIPPER_JOINT]
    assert policy["gripper_joint_name"] == OPENYAM_GRIPPER_JOINT
    assert camera["webcam"].fps == OPENYAM_POLICY_FPS


def test_rollout_blueprint_requires_policy_path_from_cli() -> None:
    parsed = BlueprintConfigParser(learning_rollout_quest_openyam).parse(
        ["--LeRobotPolicyModule.policy-path", "outputs/checkpoint"],
        environ={},
    )

    assert parsed.module_kwargs("lerobotpolicymodule")["policy_path"] == "outputs/checkpoint"

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
from dimos.imitation.policy.lerobot.blueprint import learning_rollout_quest_openyam
from dimos.robot.manipulators.openyam.config import OPENYAM_JOINTS


def _module_kwargs(blueprint: Blueprint, module_type: type) -> dict[str, Any]:
    return next(
        atom.kwargs for atom in blueprint.blueprints if issubclass(atom.module, module_type)
    )


def test_rollout_blueprint_routes_policy_to_dedicated_low_priority_trajectory() -> None:
    coordinator = _module_kwargs(learning_rollout_quest_openyam, ControlCoordinator)
    policy = next(task for task in coordinator["tasks"] if task.name == "policy_rollout")
    teleop = next(task for task in coordinator["tasks"] if task.name == "teleop_openyam")
    trajectory = next(
        task for task in coordinator["tasks"] if task.name == JOINT_TRAJECTORY_TASK_NAME
    )

    assert policy.type == "trajectory"
    assert policy.joint_names == OPENYAM_JOINTS
    assert policy.priority == 10
    assert policy.params == {}
    assert policy.stream_bind == {"joint_command": "policy_joint_command"}
    assert teleop.priority == 20
    assert trajectory.priority == 30

    task = control_task_registry.create(policy.type, policy)
    assert isinstance(task, JointTrajectoryTask)
    assert task.name == "policy_rollout"


def test_rollout_blueprint_requires_policy_path_from_cli() -> None:
    parsed = BlueprintConfigParser(learning_rollout_quest_openyam).parse(
        ["--LeRobotPolicyModule.policy-path", "outputs/checkpoint"],
        environ={},
    )

    assert parsed.module_kwargs("lerobotpolicymodule")["policy_path"] == "outputs/checkpoint"

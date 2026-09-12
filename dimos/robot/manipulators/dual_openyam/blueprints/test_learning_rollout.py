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

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.imitation.policy.module import PolicyModule
from dimos.robot.manipulators.dual_openyam.blueprints.learning_rollout import (
    dual_openyam_policy_quest_rollout,
    dual_openyam_policy_rollout,
)
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_JOINTS
from dimos.teleop.quest.quest_extensions import ArmTeleopModule


def test_backend_and_camera_devices_are_normal_module_cli_config():
    blueprint = dual_openyam_policy_rollout
    parsed = BlueprintConfigParser(blueprint).parse(
        [
            "--policy.backend",
            "lerobot",
            "--policy.policy-path",
            "checkpoint",
            "--policy.policy-joint-names",
            "null",
            "--policy.image-mapping",
            '{"left_wrist_image":"observation.images.left_wrist","right_wrist_image":"observation.images.right_wrist","overhead_image":"observation.images.overhead"}',
            "--left-wrist.serial-number",
            "left-camera",
            "--right-wrist.serial-number",
            "right-camera",
            "--controlcoordinator.left-can-port",
            "can0",
            "--controlcoordinator.right-can-port",
            "can1",
        ],
        environ={},
    )
    assert parsed.module_configs["policy"]["backend"] == "lerobot"
    assert parsed.module_configs["policy"]["policy_joint_names"] is None
    assert (
        parsed.module_configs["policy"]["image_mapping"]["overhead_image"]
        == "observation.images.overhead"
    )
    assert parsed.module_configs["left_wrist"]["serial_number"] == "left-camera"
    assert parsed.module_configs["right_wrist"]["serial_number"] == "right-camera"
    assert ArmTeleopModule not in [atom.module for atom in blueprint.active_blueprints]
    atom = next(
        atom for atom in blueprint.active_blueprints if issubclass(atom.module, PolicyModule)
    )
    assert set(atom.kwargs["image_mapping"].values()) == {"top", "left", "right"}
    coordinator = next(
        atom for atom in blueprint.active_blueprints if atom.name == "ControlCoordinator"
    )
    assert coordinator.kwargs["tasks"][0].joint_names == list(DUAL_OPENYAM_JOINTS)


def test_quest_composition_retains_manual_tasks_above_policy_priority():
    blueprint = dual_openyam_policy_quest_rollout
    assert ArmTeleopModule in [atom.module for atom in blueprint.active_blueprints]
    coordinator = next(
        atom for atom in blueprint.active_blueprints if atom.name == "ControlCoordinator"
    )
    tasks = coordinator.kwargs["tasks"]
    policy = next(task for task in tasks if task.name == "policy_rollout")
    assert policy.joint_names == list(DUAL_OPENYAM_JOINTS)
    assert all(task.priority > policy.priority for task in tasks if task.name != policy.name)

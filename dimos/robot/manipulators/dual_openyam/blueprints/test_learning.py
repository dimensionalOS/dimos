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

from pathlib import Path

import pytest

from dimos.hardware.sensors.camera.module import CameraModule
from dimos.imitation.policy.abc.module import DualOpenYamAbcPolicy
from dimos.imitation.policy.module import POLICY_ROLLOUT_INSTANCE_NAME, POLICY_ROLLOUT_TASK_NAME
from dimos.robot.manipulators.dual_openyam.blueprints.learning_collection import (
    build_dual_openyam_quest_collection,
)
from dimos.robot.manipulators.dual_openyam.blueprints.learning_rollout import (
    build_dual_openyam_abc_rollout,
)
from dimos.robot.manipulators.dual_openyam.learning import ABC_JOINTS, DualOpenYamQuestRecorder


def test_dual_collection_declares_two_distinct_camera_sources(tmp_path: Path) -> None:
    blueprint = build_dual_openyam_quest_collection(
        recording=tmp_path / "dual.mcap",
        task="fold towel",
        cameras={"left_wrist_image": 0, "right_wrist_image": 1},
        left_can_port="follower_l",
        right_can_port="follower_r",
    )
    cameras = [atom for atom in blueprint.active_blueprints if atom.module is CameraModule]

    assert any(atom.module is DualOpenYamQuestRecorder for atom in blueprint.active_blueprints)
    assert [camera.kwargs["hardware"].camera_index for camera in cameras] == [0, 1]
    assert blueprint.remapping_map[("PolicyCamera_left_wrist_image", "color_image")] == (
        "left_wrist_image"
    )
    assert blueprint.remapping_map[("PolicyCamera_right_wrist_image", "color_image")] == (
        "right_wrist_image"
    )
    coordinator = next(
        atom for atom in blueprint.active_blueprints if atom.name == "ControlCoordinator"
    )
    assert coordinator.kwargs["left_can_port"] == "follower_l"
    assert coordinator.kwargs["right_can_port"] == "follower_r"


def test_abc_rollout_requires_all_three_physical_cameras() -> None:
    with pytest.raises(ValueError, match="top_image"):
        build_dual_openyam_abc_rollout(
            artifact="checkpoint.pt",
            task="put bottles in bin",
            cameras={"left_wrist_image": 0, "right_wrist_image": 1},
        )


def test_abc_rollout_uses_released_action_order_and_stable_rpc_name() -> None:
    blueprint = build_dual_openyam_abc_rollout(
        artifact="checkpoint.pt",
        task="put bottles in bin",
        cameras={"top_image": 0, "left_wrist_image": 1, "right_wrist_image": 2},
        left_can_port="follower_l",
        right_can_port="follower_r",
    )
    policy = next(
        atom for atom in blueprint.active_blueprints if atom.module is DualOpenYamAbcPolicy
    )
    coordinator = next(
        atom for atom in blueprint.active_blueprints if atom.name == "ControlCoordinator"
    )

    assert policy.name == POLICY_ROLLOUT_INSTANCE_NAME
    rollout_task = next(
        task for task in coordinator.kwargs["tasks"] if task.name == POLICY_ROLLOUT_TASK_NAME
    )
    assert rollout_task.joint_names == list(ABC_JOINTS)
    assert coordinator.kwargs["left_can_port"] == "follower_l"
    assert coordinator.kwargs["right_can_port"] == "follower_r"

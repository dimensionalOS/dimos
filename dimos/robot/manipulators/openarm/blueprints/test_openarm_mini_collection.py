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

from __future__ import annotations

from pathlib import Path

import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint
from dimos.hardware.sensors.camera.module import WebcamModule
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.recorder import CollectionModule
from dimos.robot.manipulators.openarm.blueprints.mini_collection import (
    openarm_mini_collection,
    openarm_mini_follower_collection,
)
from dimos.robot.manipulators.openarm.blueprints.mini_teleop import OPENARM_ROUTER
from dimos.robot.manipulators.openarm.blueprints.teleop import (
    OpenArmTeleopCoordinator,
    _OpenArmManipulationModule,
)
from dimos.robot.manipulators.openarm.config import OPENARM_JOINTS
from dimos.teleop.openarm_mini.teleop_module import OpenArmMiniTeleopModule


@pytest.mark.parametrize("blueprint", [openarm_mini_collection, openarm_mini_follower_collection])
def test_collection_uses_normal_module_configuration(blueprint: Blueprint, tmp_path: Path) -> None:
    parsed = BlueprintConfigParser(blueprint).parse(
        [
            "--recorder.recording",
            str(tmp_path / "session"),
            "--episodes.task",
            "pick up the block",
            "--front.hardware.camera-index",
            "/dev/video8",
            "--left-wrist.hardware.camera-index",
            "/dev/video0",
            "--right-wrist.hardware.camera-index",
            "/dev/video2",
        ],
        environ={},
    )
    recorder = next(atom for atom in blueprint.active_blueprints if atom.name == "recorder")
    schema = recorder.kwargs["recording_schema"]
    assert schema.robot_type == "openarm"
    assert parsed.module_configs["recorder"]["recording"] == tmp_path / "session"
    assert parsed.module_configs["front"]["hardware"]["camera_index"] == "/dev/video8"
    assert parsed.module_configs["left_wrist"]["hardware"]["camera_index"] == "/dev/video0"
    assert parsed.module_configs["right_wrist"]["hardware"]["camera_index"] == "/dev/video2"


def test_state_and_action_cover_every_joint_from_the_coordinator_streams() -> None:
    recorder = next(
        atom for atom in openarm_mini_collection.active_blueprints if atom.name == "recorder"
    )
    schema = recorder.kwargs["recording_schema"]
    state = schema.observation["observation.state"]
    assert state.stream == "coordinator_joint_state"
    assert state.names == list(OPENARM_JOINTS)
    (action,) = schema.action.values()
    assert action.stream == "coordinator_joint_command"
    assert action.source_kind == "joint_position_updates"
    assert action.names == list(OPENARM_JOINTS)
    assert schema.observation["observation.images.front"].stream == "front_image"
    assert schema.observation["observation.images.left_wrist"].stream == "left_wrist_image"
    assert schema.observation["observation.images.right_wrist"].stream == "right_wrist_image"
    assert recorder.kwargs["input_types"].keys() == {
        "coordinator_joint_state",
        "coordinator_joint_command",
        "front_image",
        "left_wrist_image",
        "right_wrist_image",
    }


def test_one_machine_collection_wraps_the_one_machine_teleop_stack() -> None:
    modules = [atom.module for atom in openarm_mini_collection.active_blueprints]
    assert issubclass(modules[0], CollectionModule)
    assert modules[1:] == [
        EpisodeMonitorModule,
        OpenArmMiniTeleopModule,
        OpenArmTeleopCoordinator,
        _OpenArmManipulationModule,
        WebcamModule,
        WebcamModule,
        WebcamModule,
    ]
    assert dict(openarm_mini_collection.remapping_map) == {
        ("front", "color_image"): "front_image",
        ("front", "camera_info"): "front_camera_info",
        ("front", "tf"): "front_tf",
        ("left_wrist", "color_image"): "left_wrist_image",
        ("left_wrist", "camera_info"): "left_wrist_camera_info",
        ("left_wrist", "tf"): "left_wrist_tf",
        ("right_wrist", "color_image"): "right_wrist_image",
        ("right_wrist", "camera_info"): "right_wrist_camera_info",
        ("right_wrist", "tf"): "right_wrist_tf",
    }


def test_follower_collection_has_no_leader_and_dials_the_robot_router() -> None:
    modules = [atom.module for atom in openarm_mini_follower_collection.active_blueprints]
    assert OpenArmMiniTeleopModule not in modules
    assert OpenArmTeleopCoordinator in modules
    assert openarm_mini_follower_collection.global_config_overrides["zenoh_connect"] == (
        OPENARM_ROUTER
    )

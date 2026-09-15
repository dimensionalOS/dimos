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


import numpy as np

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.imitation.dataprep.core import resolve_field
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.dual_openyam.blueprints.learning_collection import (
    dual_openyam_quest_collection,
)
from dimos.robot.manipulators.dual_openyam.blueprints.teleop import teleop_webxr_dual_openyam
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_JOINTS
from dimos.robot.manipulators.dual_openyam.learning import DUAL_OPENYAM_COLLECTION


def test_dual_profile_projects_canonical_joint_order_and_records_both_cameras():
    profile = DUAL_OPENYAM_COLLECTION
    schema = profile.to_schema()
    message = JointState(
        name=list(reversed(DUAL_OPENYAM_JOINTS)), position=list(reversed(range(14)))
    )
    np.testing.assert_array_equal(resolve_field(message, schema.action["action"]), np.arange(14))
    np.testing.assert_array_equal(
        resolve_field(message, schema.observation["observation.state"]), np.arange(14)
    )
    assert schema.action["action"].source_kind == "joint_position_updates"
    assert schema.observation["observation.state"].source_kind == "snapshot"
    assert {
        key: feature.stream
        for key, feature in schema.observation.items()
        if feature.dtype == "video"
    } == {
        "observation.images.left_wrist": "left_wrist_image",
        "observation.images.right_wrist": "right_wrist_image",
    }
    [recorder] = [
        atom for atom in dual_openyam_quest_collection.active_blueprints if atom.name == "recorder"
    ]
    # Every Module inherits tf; the profile recorder does not record that input.
    assert {
        port.name for port in recorder.streams if port.direction == "in" and port.name != "tf"
    } == {
        "left_wrist_image",
        "right_wrist_image",
        "coordinator_joint_state",
        "applied_joint_position_command",
        "status",
    }


def test_dual_collection_configures_both_cameras_and_buses_through_run(tmp_path):
    blueprint = dual_openyam_quest_collection
    parsed = BlueprintConfigParser(blueprint).parse(
        [
            "--recorder.recording",
            str(tmp_path / "dual"),
            "--episodes.task",
            "fold towel",
            "--controlcoordinator.left-can-port",
            "follower_l",
            "--controlcoordinator.right-can-port",
            "follower_r",
            "--left-wrist.hardware.camera-index",
            "/dev/video2",
            "--right-wrist.hardware.camera-index",
            "/dev/video4",
        ],
        environ={},
    )
    cameras = [atom for atom in blueprint.active_blueprints if atom.module is CameraModule]
    assert [camera.name for camera in cameras] == ["left_wrist", "right_wrist"]
    assert blueprint.remapping_map[("left_wrist", "color_image")] == "left_wrist_image"
    assert blueprint.remapping_map[("right_wrist", "color_image")] == "right_wrist_image"
    assert parsed.module_configs["ControlCoordinator"]["left_can_port"] == "follower_l"
    assert parsed.module_configs["ControlCoordinator"]["right_can_port"] == "follower_r"
    assert parsed.module_configs["left_wrist"]["hardware"]["camera_index"] == "/dev/video2"
    assert parsed.module_configs["right_wrist"]["hardware"]["camera_index"] == "/dev/video4"


def test_collection_exposes_inherited_viser_without_changing_teleop(tmp_path):
    args = ["--recorder.recording", str(tmp_path / "dual"), "--episodes.task", "fold towel"]
    parser = BlueprintConfigParser(dual_openyam_quest_collection)
    collection = parser.parse(args, environ={}).module_kwargs("manipulationmodule")
    teleop = (
        BlueprintConfigParser(teleop_webxr_dual_openyam)
        .parse(environ={})
        .module_kwargs("manipulationmodule")
    )

    assert collection["visualization"]["host"] == "0.0.0.0"
    assert teleop["visualization"]["host"] == "127.0.0.1"
    assert collection["visualization"] == {**teleop["visualization"], "host": "0.0.0.0"}
    assert collection["model"] == teleop["model"]
    assert collection["kinematics"] == teleop["kinematics"]
    assert (
        sum(
            atom.module is ManipulationModule
            for atom in dual_openyam_quest_collection.active_blueprints
        )
        == 1
    )

    restricted = parser.parse(
        [*args, "--manipulationmodule.visualization.host", "127.0.0.1"], environ={}
    ).module_kwargs("manipulationmodule")
    assert restricted["visualization"] == teleop["visualization"]

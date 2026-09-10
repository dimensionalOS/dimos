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
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.robot.manipulators.dual_openyam.blueprints.learning_collection import (
    dual_openyam_quest_collection,
)


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

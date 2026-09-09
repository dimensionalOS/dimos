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


import pytest

from dimos.experimental.memory.rust_recorder import RustRecorder
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.robot.manipulators.dual_openyam.blueprints.learning_collection import (
    build_dual_openyam_quest_collection,
)
from dimos.robot.manipulators.dual_openyam.learning import DUAL_OPENYAM_COLLECTION


def test_dual_collection_declares_two_distinct_cameras_and_both_buses(tmp_path):
    blueprint = build_dual_openyam_quest_collection(
        recording=tmp_path / "dual.mcap",
        task="fold towel",
        cameras={"left_wrist_image": 0, "right_wrist_image": 1},
        left_can_port="follower_l",
        right_can_port="follower_r",
    )
    cameras = [atom for atom in blueprint.active_blueprints if atom.module is CameraModule]
    assert issubclass(blueprint.active_blueprints[0].module, RustRecorder)
    assert [camera.kwargs["hardware"].camera_index for camera in cameras] == [0, 1]
    assert (
        blueprint.remapping_map[("CollectionCamera_left_wrist_image", "color_image")]
        == "left_wrist_image"
    )
    assert (
        blueprint.remapping_map[("CollectionCamera_right_wrist_image", "color_image")]
        == "right_wrist_image"
    )
    coordinator = next(
        atom for atom in blueprint.active_blueprints if atom.name == "ControlCoordinator"
    )
    assert coordinator.kwargs["left_can_port"] == "follower_l"
    assert coordinator.kwargs["right_can_port"] == "follower_r"


def test_custom_profile_adds_overhead_camera_without_a_new_recorder(tmp_path):
    profile = DUAL_OPENYAM_COLLECTION.model_copy(deep=True)
    profile.observations["overhead"] = profile.observations[
        "observation.images.left_wrist"
    ].model_copy(
        update={"stream": "overhead_image"},
    )
    blueprint = build_dual_openyam_quest_collection(
        profile=profile,
        recording=tmp_path / "three.mcap",
        task="fold towel",
        cameras={"left_wrist_image": 0, "right_wrist_image": 1, "overhead_image": 2},
    )
    recorder = blueprint.active_blueprints[0]
    assert {s.name for s in recorder.streams} >= {
        "left_wrist_image",
        "right_wrist_image",
        "overhead_image",
        "status",
    }
    assert len([atom for atom in blueprint.active_blueprints if atom.module is CameraModule]) == 3


@pytest.mark.parametrize(
    ("devices", "error"),
    [
        ({"left_wrist_image": 0}, "missing cameras.*right_wrist_image"),
        ({"left_wrist_image": 0, "right_wrist_image": 1, "typo": 2}, "unknown cameras.*typo"),
    ],
)
def test_invalid_camera_bindings_fail_before_hardware_import(tmp_path, devices, error):
    with pytest.raises(ValueError, match=error):
        build_dual_openyam_quest_collection(
            recording=tmp_path / "dual.mcap",
            task="fold",
            cameras=devices,
        )

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

"""Recording composition accepts explicit producers without owning their hardware."""

import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.profile import CollectionProfile
from dimos.imitation.collection.recorder import CollectionModule, collect
from dimos.imitation.dataprep.schema import FeatureSpec
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState


@pytest.fixture
def profile():
    state = FeatureSpec(
        stream="state",
        message_type=JointState,
        field="position",
        dtype="float32",
        shape=(2,),
        names=["right", "left"],
    )
    return CollectionProfile(
        observations={
            **{
                f"observation.images.{name}": FeatureSpec(
                    stream=f"{name}_image",
                    message_type=Image,
                    field="data",
                    dtype="video",
                    shape=(480, 640, 3),
                    names=["height", "width", "channels"],
                )
                for name in ("left_wrist", "right_wrist", "overhead")
            },
            "observation.state": state,
        },
        actions={
            "action": state.model_copy(
                update={
                    "stream": "command",
                    "source_kind": "joint_position_updates",
                },
                deep=True,
            )
        },
    )


def test_collect_only_declares_recorder_and_preserves_joint_semantics(profile, tmp_path):
    graph = collect(profile=profile, recording=tmp_path / "session")
    [recorder] = graph.active_blueprints
    assert recorder.module is CollectionModule
    assert recorder.kwargs["recording"] == tmp_path / "session"
    schema = recorder.kwargs["recording_schema"]
    assert schema.observation["observation.state"].names == ["right", "left"]
    assert schema.action["action"].source_kind == "joint_position_updates"
    assert schema.observation["observation.state"].source_kind == "snapshot"
    assert schema.action["action"].stream == "command"


@pytest.mark.parametrize("camera_module", [CameraModule, RealSenseCamera])
def test_three_cameras_can_be_replaced_without_changing_profile(profile, camera_module):
    names = ("left_wrist", "right_wrist", "overhead")
    graph = autoconnect(
        collect(profile=profile),
        EpisodeMonitorModule.blueprint(instance_name="episodes"),
        *(camera_module.blueprint(instance_name=name) for name in names),
    ).remappings([(name, "color_image", f"{name}_image") for name in names])
    recorder = next(atom for atom in graph.active_blueprints if atom.name == "recorder")
    effective_outputs = {
        (graph.remapping_map.get((atom.name, stream.name), stream.name), stream.type)
        for atom in graph.active_blueprints
        if atom.module is camera_module
        for stream in atom.streams
        if stream.direction == "out"
    }
    assert {(f"{name}_image", Image) for name in names} <= effective_outputs
    assert recorder.kwargs["recording_schema"].model_dump() == profile.to_schema().model_dump()
    assert [atom.name for atom in graph.active_blueprints if atom.module is camera_module] == list(
        names
    )


def test_collect_does_not_require_a_unique_coordinator(profile):
    graph = autoconnect(
        collect(profile=profile),
        ControlCoordinator.blueprint(instance_name="left"),
        ControlCoordinator.blueprint(instance_name="right"),
    )
    assert [atom.name for atom in graph.active_blueprints] == ["recorder", "left", "right"]


def test_custom_action_features_are_not_limited_to_joint_commands(profile):
    profile.actions = {
        "action.correction": FeatureSpec(
            stream="correction",
            message_type=JointState,
            field="effort",
            dtype="float32",
            shape=(1,),
            names=["left"],
        ),
        "action.selected": profile.actions["action"],
    }
    [recorder] = collect(profile=profile).active_blueprints
    schema = recorder.kwargs["recording_schema"]
    assert schema.action["action.correction"].field == "effort"
    assert schema.action["action.selected"].stream == "command"
    assert set(recorder.kwargs["input_types"]) == {
        "left_wrist_image",
        "right_wrist_image",
        "overhead_image",
        "state",
        "correction",
        "command",
    }

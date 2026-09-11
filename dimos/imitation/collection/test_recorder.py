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

import importlib
from pathlib import Path
import pickle
import sys

import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.worker_manager_python import WorkerManagerPython
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.module import Module
from dimos.core.native_module import NativeModule
from dimos.core.stream import Out, RemoteIn
from dimos.imitation.collection import recorder as recorder_module
from dimos.imitation.collection.profile import CollectionFeature, CollectionProfile
from dimos.imitation.collection.recorder import CollectionRecorderConfig, collection_recorder
from dimos.imitation.collection.recording import RecordingSchema
from dimos.imitation.dataprep.core import OutputConfig, SyncConfig
from dimos.msgs.imitation_msgs.EpisodeStatus import EpisodeStatus
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.get_all_blueprints import get_by_name


class _CameraPublisher(Module):
    color_image: Out[Image]


def _profile(camera_count):
    return CollectionProfile(
        name="test",
        robot_type="test",
        observations={
            **{
                f"images.{i}": CollectionFeature(
                    stream=f"camera_{i}",
                    message_type=Image,
                    field="data",
                    dtype="video",
                    shape=(8, 8, 3),
                    names=["height", "width", "channels"],
                )
                for i in range(camera_count)
            },
            "state": CollectionFeature(
                stream="measured",
                message_type=JointState,
                field="position",
                dtype="float32",
                shape=(1,),
                names=["joint"],
            ),
        },
        actions={
            "action": CollectionFeature(
                stream="measured",
                message_type=JointState,
                field="position",
                dtype="float32",
                shape=(1,),
                names=["joint"],
            )
        },
        sync=SyncConfig(anchor="images.0", rate_hz=30, tolerance_ms=20),
    )


@pytest.fixture
def recorder(tmp_path):
    profile = _profile(2)
    profile.actions["action"].stream = "commanded"
    atom = collection_recorder(
        profile=profile,
        recording=tmp_path / "collection.mcap",
    ).active_blueprints[0]
    instance = atom.module(**atom.kwargs)
    yield instance
    instance.stop()


@pytest.mark.parametrize("count", [1, 2, 4])
def test_factory_exposes_all_typed_inputs_before_autoconnect(count, tmp_path):
    blueprint = collection_recorder(profile=_profile(count), recording=tmp_path / "test.mcap")
    stack = autoconnect(
        blueprint,
        *[_CameraPublisher.blueprint(instance_name=f"publisher_{i}") for i in range(count)],
    ).remappings([(f"publisher_{i}", "color_image", f"camera_{i}") for i in range(count)])
    atom = stack.active_blueprints[0]
    assert {s.name: s.type for s in atom.streams if s.name != "tf"} == {
        **{f"camera_{i}": Image for i in range(count)},
        "measured": JointState,
        "status": EpisodeStatus,
    }
    assert len([s for s in atom.streams if s.name == "measured"]) == 1
    # Autoconnect groups remapped ports by (name, message type).
    for i in range(count):
        peers = [
            (peer.name, port.direction)
            for peer in stack.active_blueprints
            for port in peer.streams
            if stack.remapping_map.get((peer.name, port.name), port.name) == f"camera_{i}"
            and port.type is Image
        ]
        assert peers == [(atom.name, "in"), (f"publisher_{i}", "out")]


def test_two_shapes_and_recording_paths_have_independent_configuration(tmp_path):
    first = collection_recorder(
        profile=_profile(1), recording=tmp_path / "one.mcap", instance_name="one"
    )
    second = collection_recorder(
        profile=_profile(4), recording=tmp_path / "four.mcap", instance_name="four"
    )
    same_shape = collection_recorder(profile=_profile(1), recording=tmp_path / "other.mcap")
    one, four = autoconnect(first, second).active_blueprints
    assert one.module is same_shape.active_blueprints[0].module
    assert one.module is not four.module
    assert one.kwargs["recording"] == tmp_path / "one.mcap"
    assert four.kwargs["recording"] == tmp_path / "four.mcap"


def test_blueprint_pickle_and_reload_preserve_generated_class(tmp_path):
    blueprint = collection_recorder(profile=_profile(2), recording=tmp_path / "recording.mcap")
    module = blueprint.active_blueprints[0].module
    payload = pickle.dumps(blueprint)
    assert pickle.loads(payload).active_blueprints[0].module is module
    importlib.reload(recorder_module)
    restored = pickle.loads(payload)
    assert restored.active_blueprints[0].module is module
    assert pickle.loads(pickle.dumps(restored)).active_blueprints[0].module is module
    rebuilt = module.blueprint(**restored.active_blueprints[0].kwargs)
    assert rebuilt.active_blueprints[0].streams == restored.active_blueprints[0].streams


def test_recorder_resolves_every_connected_stream(recorder, mocker):
    for name in (
        "camera_0",
        "camera_1",
        "measured",
        "commanded",
        "status",
    ):
        getattr(recorder, name).transport = mocker.MagicMock(channel=f"dimos/{name}")
    assert {s.name: s.codec for s in recorder._stream_specs()} == {
        "camera_0": "jpeg",
        "camera_1": "jpeg",
        "measured": "lcm",
        "commanded": "lcm",
        "status": "lcm",
    }


def test_missing_input_fails_before_native_process_starts(recorder, mocker):
    start = mocker.patch("dimos.experimental.memory.rust_recorder.NativeModule.start")
    with pytest.raises(ValueError, match="Missing required collection inputs"):
        recorder.start()
    start.assert_not_called()


@pytest.mark.parametrize(
    "stream", ["status", "tf", "start", "config", "rpc", "bad-name", "_private"]
)
def test_invalid_ports_fail_at_factory_boundary(stream, tmp_path):
    profile = _profile(1)
    profile.observations["images.0"].stream = stream
    with pytest.raises(ValueError, match="reserved"):
        collection_recorder(profile=profile, recording=tmp_path / "invalid.mcap")

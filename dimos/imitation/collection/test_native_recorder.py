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
from dimos.imitation.collection import native_recorder
from dimos.imitation.collection.native_recorder import collection_recorder
from dimos.imitation.collection.profile import CollectionFeature, CollectionProfile
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
    importlib.reload(native_recorder)
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


def test_native_collection_uses_the_recorder_build_directory(recorder):
    root = Path(__file__).parents[2] / "experimental" / "memory" / "rust"
    assert Path(recorder.config.cwd) == root
    assert Path(recorder.config.executable) == root / "result/bin/dimos-memory-recorder"


@pytest.fixture
def worker_manager():
    manager = WorkerManagerPython(g=GlobalConfig(n_workers=1))
    manager.start()
    yield manager
    manager.stop()


@pytest.fixture
def deployed_recorders(worker_manager):
    proxies = []
    yield proxies
    for proxy in reversed(proxies):
        proxy.stop()


@pytest.mark.skipif_macos_bug
def test_generated_inputs_survive_forkserver_and_fresh_deployment(
    worker_manager,
    deployed_recorders,
    tmp_path,
):
    # Workers predate the class; fork inheritance cannot make this pass.
    assert worker_manager.workers[0].pid is not None
    atom = collection_recorder(
        profile=_profile(3), recording=tmp_path / "worker.mcap"
    ).active_blueprints[0]
    first = worker_manager.deploy(
        atom.module, global_config, {**atom.kwargs, "instance_name": "first"}
    )
    deployed_recorders.append(first)
    importlib.reload(native_recorder)
    reloaded = getattr(native_recorder, atom.module.__name__)
    fresh = worker_manager.deploy_fresh(
        reloaded, global_config, {**atom.kwargs, "instance_name": "fresh"}
    )
    deployed_recorders.append(fresh)
    for proxy in (first, fresh):
        for name, kind in atom.module.recording_inputs:
            port = getattr(proxy, name)
            assert isinstance(port, RemoteIn)
            assert port.type is kind
    pids = [worker.pid for worker in worker_manager.workers]
    assert len(set(pids)) == 2


def test_unsupported_message_type_is_rejected(tmp_path):
    profile = _profile(1)
    profile.observations["images.0"].message_type = str
    with pytest.raises(TypeError, match="native recording"):
        collection_recorder(profile=profile, recording=tmp_path / "invalid.mcap")


def test_local_message_type_is_rejected(tmp_path):
    class LocalMessage:
        pass

    profile = _profile(1)
    profile.observations["images.0"].message_type = LocalMessage
    with pytest.raises(ValueError, match="importable at module level"):
        collection_recorder(profile=profile, recording=tmp_path / "invalid.mcap")


def test_run_config_exposes_session_values_not_schema_or_file_path(tmp_path):
    blueprint = collection_recorder(profile=_profile(1))
    parser = BlueprintConfigParser(blueprint)
    help_text = parser.format_help()
    assert "--recorder.recording" in help_text
    assert "--recorder.format" in help_text
    assert "recording-schema" not in help_text
    assert "store.path" not in help_text
    parsed = parser.parse(
        ["--recorder.recording", str(tmp_path / "session"), "--recorder.format", "sqlite"],
        environ={},
    )
    atom = blueprint.active_blueprints[0]
    recorder = atom.module(**{**atom.kwargs, **parsed.module_kwargs(atom.name)})
    try:
        assert recorder.config.recording_store().path == str(tmp_path / "session" / "recording.db")
        assert recorder._recording_schema.observation["images.0"].stream == "camera_0"
    finally:
        recorder.stop()


def test_same_ports_keep_independent_dataset_projections(tmp_path):
    first_profile = _profile(1)
    second_profile = _profile(1)
    second_profile.observations["state"].names = ["other_joint"]
    first = collection_recorder(
        profile=first_profile, recording=tmp_path / "first"
    ).active_blueprints[0]
    second = collection_recorder(
        profile=second_profile, recording=tmp_path / "second"
    ).active_blueprints[0]
    assert first.module is second.module
    assert first.kwargs["recording_schema"].observation["state"].names == ["joint"]
    assert second.kwargs["recording_schema"].observation["state"].names == ["other_joint"]


@pytest.fixture
def connected_recorder(tmp_path, mocker):
    def make(format="mcap", **kwargs):
        atom = collection_recorder(
            profile=_profile(2), recording=tmp_path / "session", format=format
        ).active_blueprints[0]
        instance = atom.module(**atom.kwargs, **kwargs)
        for port, _ in instance.recording_inputs:
            getattr(instance, port).transport = mocker.MagicMock(channel=f"dimos/{port}")
        recorders.append(instance)
        return instance

    recorders = []
    yield make
    for instance in recorders:
        instance.stop()


@pytest.mark.parametrize(
    ("format", "payload"), [("mcap", "recording.mcap"), ("sqlite", "recording.db")]
)
def test_build_saves_portable_schema_before_native_capture(
    connected_recorder, format, payload, mocker
):
    recorder = connected_recorder(
        format, stream_remapping={"camera_0": "wrist", "status": "episodes"}
    )
    mocker.patch.object(NativeModule, "build")
    start = mocker.patch.object(NativeModule, "start")
    recorder.build()
    directory = recorder.config.recording
    schema = RecordingSchema.model_validate_json((directory / "schema.json").read_text())
    assert schema.payload == payload
    assert schema.observation["images.0"].stream == "wrist"
    assert schema.episodes.status_stream == "episodes"
    assert schema.action["action"].names == ["joint"]
    config = schema.dataprep_config(directory, OutputConfig(path=directory.parent / "dataset"))
    assert config.source == str(directory / payload)
    start.assert_not_called()
    recorder.start()
    start.assert_called_once_with()
    assert recorder.config.to_config_dict()["store"]["path"] == str(directory / payload)


def test_existing_directory_is_never_overwritten(connected_recorder, mocker):
    recorder = connected_recorder()
    recorder.config.recording.mkdir()
    marker = recorder.config.recording / "schema.json"
    marker.write_text("existing")
    mocker.patch.object(NativeModule, "build")
    start = mocker.patch.object(NativeModule, "start")
    with pytest.raises(FileExistsError):
        recorder.build()
    assert marker.read_text() == "existing"
    start.assert_not_called()


def test_missing_connections_fail_before_build_or_directory_creation(recorder, mocker):
    native_build = mocker.patch.object(NativeModule, "build")
    with pytest.raises(ValueError, match="Missing required collection inputs"):
        recorder.build()
    native_build.assert_not_called()
    assert not recorder.config.recording.exists()


def test_schema_write_failure_prevents_capture(connected_recorder, mocker):
    recorder = connected_recorder()
    mocker.patch.object(NativeModule, "build")
    start = mocker.patch.object(NativeModule, "start")
    mocker.patch.object(Path, "open", side_effect=PermissionError("not writable"))
    with pytest.raises(PermissionError, match="not writable"):
        recorder.build()
    start.assert_not_called()
    assert not recorder._prepared


def test_external_package_uses_standard_blueprint_entrypoint(tmp_path, monkeypatch):
    package = tmp_path / "vendor_robot"
    package.mkdir()
    (package / "__init__.py").write_text("")
    (package / "collection.py").write_text(
        "from dimos.core.coordination.blueprints import autoconnect\n"
        "from dimos.imitation.collection.native_recorder import collection_recorder\n"
        "from dimos.imitation.collection.profile import CollectionFeature, CollectionProfile\n"
        "from dimos.imitation.dataprep.core import SyncConfig\n"
        "from dimos.msgs.sensor_msgs.JointState import JointState\n"
        "feature = CollectionFeature(stream='joints', message_type=JointState, field='position', dtype='float32', shape=(1,), names=['joint'])\n"
        "profile = CollectionProfile(name='vendor', robot_type='vendor', observations={'state': feature}, actions={'action': feature}, sync=SyncConfig(anchor='state', rate_hz=30, tolerance_ms=20))\n"
        "collect = autoconnect(collection_recorder(profile=profile))\n"
    )
    metadata = tmp_path / "vendor_robot-1.0.dist-info"
    metadata.mkdir()
    (metadata / "METADATA").write_text("Metadata-Version: 2.1\nName: vendor-robot\nVersion: 1.0\n")
    (metadata / "entry_points.txt").write_text(
        "[dimos.blueprints]\ncollect = vendor_robot.collection:collect\n"
    )
    monkeypatch.syspath_prepend(str(tmp_path))
    try:
        blueprint = get_by_name("vendor-robot.collect")
        parsed = BlueprintConfigParser(blueprint).parse(
            ["--recording", str(tmp_path / "session")], environ={}
        )
        assert parsed.module_kwargs("recorder")["recording"] == tmp_path / "session"
        assert blueprint.active_blueprints[0].kwargs["recording_schema"].robot_type == "vendor"
        assert {port.name for port in blueprint.active_blueprints[0].streams} >= {
            "joints",
            "status",
        }
    finally:
        sys.modules.pop("vendor_robot.collection", None)
        sys.modules.pop("vendor_robot", None)

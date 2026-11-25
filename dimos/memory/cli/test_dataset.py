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

from dataclasses import asdict
import json
from pathlib import Path
import warnings

from mcap.writer import Writer
import pytest
from typer.testing import CliRunner

from dimos import constants
from dimos.cloud.data import _tag
from dimos.memory.cli.app import mem_app
from dimos.memory.cli.dataset import (
    AmbiguousDatasetError,
    open_dataset,
    open_store,
    resolve_dataset,
)
from dimos.memory.replay import resolve_db_path
from dimos.memory.replay_module import dataset_path, stream_types_of
from dimos.memory.store.mcap import McapStore
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.std_msgs.String import String
from dimos.robot.unitree.go2.dds.msgs.ControlEvent import ControlEvent
from dimos.robot.unitree.go2.dds.msgs.Telemetry import Telemetry
from dimos.robot.unitree.go2.dds.store import Go2McapStore
from dimos.utils import data


def test_native_mcap_cli_and_replay_resolve_typed_stream(tmp_path):
    path = tmp_path / "native.mcap"
    value = Imu(ts=12.5, frame_id="imu_link")
    with path.open("wb") as output:
        writer = Writer(output)
        writer.start(profile="dimos")
        channel = writer.register_channel(
            topic="imu",
            message_encoding="lcm",
            schema_id=0,
            metadata={
                "dimos.payload_type": "dimos.msgs.sensor_msgs.Imu.Imu",
                "dimos.observation_time": "publish_time",
            },
        )
        writer.add_message(
            channel_id=channel,
            log_time=20_000_000_000,
            publish_time=12_500_000_000,
            data=value.lcm_encode(),
        )
        writer.finish()

    with warnings.catch_warnings(record=True) as emitted:
        warnings.simplefilter("always", DeprecationWarning)
        with open_dataset(path) as store:
            assert type(store) is McapStore
            assert store.streams.imu.first().data.lcm_encode() == value.lcm_encode()
            assert store.streams.imu.first().ts == 12.5
        assert not [item for item in emitted if item.category is DeprecationWarning]
    assert stream_types_of(str(path)) == {"imu": Imu}
    result = CliRunner().invoke(mem_app, ["summary", str(path)])
    assert result.exit_code == 0, result.output
    assert "raw bytes" not in result.output
    assert "imu" in result.output


@pytest.mark.parametrize("with_dds", [False, True])
def test_legacy_go2_aliases_json_and_log_time_remain_available(tmp_path, with_dds):
    path = tmp_path / "legacy.mcap"
    with path.open("wb") as output:
        writer = Writer(output)
        writer.start(profile="ros2")
        # A known DDS channel selects the legacy preset. Its empty stream must
        # still retain its alias; the JSON channel proves decoding and timing.
        if with_dds:
            writer.register_channel(topic="rt/utlidar/imu", message_encoding="cdr", schema_id=0)
        channel = writer.register_channel(topic="control_log", message_encoding="json", schema_id=0)
        writer.add_message(
            channel_id=channel,
            log_time=20_000_000_000,
            publish_time=12_500_000_000,
            data=b'{"type":"velocity_input","lx":0.5,"extra":"preserved in original"}',
        )
        writer.finish()

    with open_dataset(path) as store:
        assert type(store) is McapStore
        assert store.list_streams() == (["control_log", "imu"] if with_dds else ["control_log"])
        observation = store.streams.control_log.first()
        assert observation.ts == 20.0
        assert observation.data == ControlEvent(type="velocity_input", lx=0.5)
    with pytest.warns(DeprecationWarning, match="open_dataset"):
        with Go2McapStore(path=str(path)) as old:
            assert old.streams.control_log.first().data == observation.data


def test_mixed_native_channel_is_not_overridden_by_go2_preset(tmp_path):
    path = tmp_path / "mixed.mcap"
    value = Imu(ts=12.5, frame_id="native_imu")
    with path.open("wb") as output:
        writer = Writer(output)
        writer.start(profile="dimos")
        writer.register_channel(topic="rt/lowstate", message_encoding="cdr", schema_id=0)
        channel = writer.register_channel(
            topic="rt/utlidar/imu",
            message_encoding="lcm",
            schema_id=0,
            metadata={"dimos.payload_type": "dimos.msgs.sensor_msgs.Imu.Imu"},
        )
        writer.add_message(channel_id=channel, log_time=1, publish_time=1, data=value.lcm_encode())
        writer.finish()
    with open_dataset(path) as store:
        decoded = store.stream("utlidar_imu").first().data
        assert isinstance(decoded, Imu)
        assert decoded.lcm_encode() == value.lcm_encode()


@pytest.mark.parametrize("profile", ["dimos", "ros2"])
@pytest.mark.parametrize("opener", [open_dataset, open_store])
def test_mixed_recording_preserves_legacy_json_types(tmp_path, profile, opener):
    path = tmp_path / "mixed-events.mcap"
    value = Imu(ts=12.5, frame_id="native_imu")
    expected_telemetry = Telemetry(
        type="telemetry",
        battery=0.785,
        body_h=0.3,
        current_a=1.0,
        imu_hz=100.0,
        lidar=True,
        lidar_hz=10.0,
        lowstate_hz=50.0,
        mode=0,
        obstacle=False,
        odom_hz=50.0,
        points_per_s=1000.0,
        rage=False,
        recording={"active": False},
        rss_mb=128.0,
        sportmode_hz=50.0,
        vel=[0.0, 0.0],
        yaw=0.0,
    )
    with path.open("wb") as output:
        writer = Writer(output)
        writer.start(profile=profile)
        native = writer.register_channel(
            topic="imu",
            message_encoding="lcm",
            schema_id=0,
            metadata={"dimos.payload_type": "dimos.msgs.sensor_msgs.Imu.Imu"},
        )
        writer.add_message(channel_id=native, log_time=1, publish_time=1, data=value.lcm_encode())
        for topic, payload in [
            ("telemetry", json.dumps(asdict(expected_telemetry)).encode()),
            ("control_log", b'{"type":"velocity_input","lx":0.5}'),
        ]:
            channel = writer.register_channel(topic=topic, message_encoding="json", schema_id=0)
            writer.add_message(
                channel_id=channel,
                log_time=20_000_000_000,
                publish_time=12_500_000_000,
                data=payload,
            )
        writer.finish()

    with opener(path) as store:
        imu = store.streams.imu.first().data
        assert isinstance(imu, Imu)
        assert imu.lcm_encode() == value.lcm_encode()
        telemetry = store.streams.telemetry.first()
        assert isinstance(telemetry.data, Telemetry)
        assert telemetry.data == expected_telemetry
        assert telemetry.data.battery == 0.785
        assert telemetry.ts == 20.0
        control = store.streams.control_log.first()
        assert isinstance(control.data, ControlEvent)
        assert control.data.lx == 0.5
        assert control.ts == 20.0


@pytest.mark.parametrize("topic", ["telemetry", "control_log"])
@pytest.mark.parametrize(
    ("encoding", "metadata", "expected_type"),
    [
        ("json", {"dimos.payload_type": "dimos.msgs.std_msgs.String.String"}, String),
        ("json", {"dimos.payload_type": "unknown.Payload"}, bytes),
        ("custom", {}, bytes),
    ],
)
def test_legacy_json_selection_respects_encoding_and_metadata(
    tmp_path, topic, encoding, metadata, expected_type
):
    path = tmp_path / "not-legacy.mcap"
    payload = b'{"event":"ready"}'
    with path.open("wb") as output:
        writer = Writer(output)
        writer.start(profile="dimos")
        channel = writer.register_channel(
            topic=topic, message_encoding=encoding, schema_id=0, metadata=metadata
        )
        writer.add_message(channel_id=channel, log_time=1, publish_time=1, data=payload)
        writer.finish()

    with open_dataset(path) as store:
        decoded = store.stream(topic).first().data
        assert isinstance(decoded, expected_type)
        if expected_type is String:
            assert decoded.data == payload.decode()
        else:
            assert decoded == payload


@pytest.fixture
def dataset_dirs(tmp_path, monkeypatch):
    project = tmp_path / "project"
    data_dir = project / "data"
    (data_dir / ".lfs").mkdir(parents=True)
    downloads = tmp_path / "downloads"
    recordings = tmp_path / "recordings"
    downloads.mkdir()
    recordings.mkdir()
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(data, "DIMOS_PROJECT_ROOT", project)
    monkeypatch.setattr(data, "get_project_root", lambda: project)
    monkeypatch.setattr(data, "get_data_dir", lambda: data_dir)
    monkeypatch.setattr(data, "_get_lfs_dir", lambda: data_dir / ".lfs")
    monkeypatch.setattr(constants, "DIMOS_PROJECT_ROOT", project)
    monkeypatch.setattr(constants, "DOWNLOADS_DIR", downloads)
    monkeypatch.setattr(constants, "RECORDINGS_DIR", recordings)
    return recordings, downloads


@pytest.mark.parametrize("resolver", [resolve_dataset, resolve_db_path, dataset_path])
def test_resolve_recording_name(dataset_dirs, resolver):
    recordings, _ = dataset_dirs
    run = recordings / "20260830-120000-go2" / "memory.db"
    run.parent.mkdir()
    run.touch()

    assert Path(resolver(run.parent.name)) == run


@pytest.mark.parametrize("resolver", [resolve_dataset, resolve_db_path])
@pytest.mark.parametrize("filename", ["memory.db", "memory.mcap"])
@pytest.mark.parametrize(
    "timestamp",
    [{"created_at": "2026-08-30T12:00:00+00:00"}, {}],
    ids=["timestamped", "no-timestamp"],
)
@pytest.mark.parametrize(
    "alias", ["name", "stem", "filename", "file_stem", "tag", "id", "short_id"]
)
def test_resolve_download_alias(dataset_dirs, resolver, filename, timestamp, alias):
    _, downloads = dataset_dirs
    upload = {"id": "abcdef1234567890", **timestamp}
    tag = _tag(upload)
    pulled = downloads / f"{tag}{filename}"
    pulled.touch()
    names = {
        "name": pulled.name,
        "stem": pulled.stem,
        "filename": filename,
        "file_stem": Path(filename).stem,
        "tag": tag.removesuffix("-"),
        "id": upload["id"][:8],
        "short_id": upload["id"][:4],
    }

    assert resolver(names[alias]) == pulled


@pytest.mark.parametrize("location", ["absolute", "relative", "data"])
def test_resolve_db_path_appends_db_to_dotted_stem(dataset_dirs, tmp_path, location):
    paths = {
        "absolute": (tmp_path / "session.v1", tmp_path / "session.v1.db"),
        "relative": (Path("session.v1"), tmp_path / "session.v1.db"),
        "data": (Path("session.v1"), data.get_data_dir() / "session.v1.db"),
    }
    stem, recording = paths[location]
    recording.touch()

    assert resolve_db_path(stem).resolve() == recording


def test_resolve_dataset_preserves_dotted_stem_extension(dataset_dirs):
    (data.get_data_dir() / "session.v1.db").touch()

    with pytest.raises(FileNotFoundError, match="session.v1"):
        resolve_dataset("session.v1")


@pytest.mark.parametrize("resolver", [resolve_dataset, resolve_db_path])
@pytest.mark.parametrize("name", ["session", "session.v1"])
@pytest.mark.parametrize("location", ["cwd", "project"])
def test_resolve_exact_input_before_suffixed_candidate(dataset_dirs, resolver, name, location):
    roots = {"cwd": Path.cwd(), "project": data.DIMOS_PROJECT_ROOT}
    exact = roots[location] / name
    exact.touch()
    (Path.cwd() / f"{name}.db").touch()

    assert resolver(name).resolve() == exact


@pytest.mark.parametrize("resolver", [resolve_dataset, resolve_db_path])
@pytest.mark.parametrize("location_index", [0, 1, 2], ids=["cwd", "project", "data"])
def test_resolve_existing_locations_before_recordings_and_downloads(
    dataset_dirs, resolver, location_index
):
    recordings, downloads = dataset_dirs
    run = recordings / "memory" / "memory.db"
    run.parent.mkdir()
    run.touch()
    (downloads / f"{_tag({'id': 'abcdef1234567890'})}memory.db").touch()
    roots = [Path.cwd(), data.DIMOS_PROJECT_ROOT, data.get_data_dir()]
    for root in roots[location_index:]:
        (root / "memory.db").touch()

    assert resolver("memory").resolve() == roots[location_index] / "memory.db"


def test_resolve_mcap_recording_by_folder_name(dataset_dirs):
    recordings, _ = dataset_dirs
    run = recordings / "2026-11-11_01-00-00pm-PST" / "memory.mcap"
    run.parent.mkdir()
    run.touch()

    assert resolve_dataset(run.parent.name) == run
    assert dataset_path(run.parent.name) == str(run)


@pytest.fixture(params=["downloads", "recording-and-download"])
def ambiguous_datasets(dataset_dirs, request):
    recordings, downloads = dataset_dirs
    locations = {"downloads": downloads, "recording-and-download": recordings / "memory.db"}
    first = locations[request.param] / f"{_tag({'id': 'abcdef1234567890'})}memory.db"
    first.parent.mkdir(exist_ok=True)
    second = downloads / f"{_tag({'id': 'fedcba0987654321'})}memory.db"
    first.touch()
    second.touch()
    return first, second


@pytest.mark.parametrize("resolver", [resolve_dataset, resolve_db_path, dataset_path])
def test_ambiguous_dataset_name_lists_candidates(ambiguous_datasets, resolver):
    first, second = ambiguous_datasets

    with pytest.raises(AmbiguousDatasetError, match="Ambiguous dataset") as error:
        resolver("memory.db")
    assert str(first) in str(error.value)
    assert str(second) in str(error.value)


@pytest.mark.parametrize("args", [["summary", "memory.db"], ["rerun", "memory.db", "--no-gui"]])
def test_cli_ambiguous_dataset_lists_candidates(ambiguous_datasets, args):
    first, second = ambiguous_datasets

    result = CliRunner().invoke(mem_app, args, env={"COLUMNS": "1000"})

    assert result.exit_code == 2
    assert "Ambiguous dataset" in result.output
    assert str(first) in result.output
    assert str(second) in result.output


def test_unknown_dataset_lists_search_locations(dataset_dirs):
    recordings, downloads = dataset_dirs

    with pytest.raises(FileNotFoundError, match="Dataset 'unknown' not found") as error:
        resolve_dataset("unknown")
    assert str(recordings) in str(error.value)
    assert str(downloads) in str(error.value)
    assert str(data.get_data_dir()) in str(error.value)
    assert "Test file" not in str(error.value)

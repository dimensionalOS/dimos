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
from typing import Any, TypeVar

import pytest

from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.experimental.memory.cdr_schema import recording_schema
from dimos.experimental.memory.rust_recorder import (
    RustMcapStoreConfig,
    RustRecorder,
    RustRecorderConfig,
    RustSqliteStoreConfig,
    RustStreamSpec,
)
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.type.recording import OnExisting
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos.messages.sensor_msgs.msg import Image
from dimos.messages.std_msgs.msg import Header


class SampleRustRecorder(RustRecorder):
    color_image: In[Image]
    odometry: In[PoseStamped]


class Unsupported:
    pass


class UnsupportedRustRecorder(RustRecorder):
    values: In[Unsupported]


class FakeTransport:
    def __init__(self, channel: str) -> None:
        self.channel = channel

    def stop(self) -> None:
        pass


TRecorder = TypeVar("TRecorder", bound=RustRecorder)


@pytest.fixture
def make_recorder() -> Any:
    recorders: list[RustRecorder] = []

    def make(recorder_type: type[TRecorder], **kwargs: Any) -> TRecorder:
        recorder = recorder_type(**kwargs)
        recorders.append(recorder)
        return recorder

    yield make
    for recorder in recorders:
        recorder.stop()


def connect(recorder: RustRecorder, **channels: str) -> None:
    for name, channel in channels.items():
        getattr(recorder, name).transport = FakeTransport(channel)  # type: ignore[assignment]


def test_specs_use_native_defaults_remapping_and_configured_workers(
    tmp_path: Path, make_recorder: Any
) -> None:
    recorder = make_recorder(
        SampleRustRecorder,
        store=RustSqliteStoreConfig(path=str(tmp_path / "recording.db")),
        record_tf=False,
        encoding_threads=7,
        stream_remapping={"odometry": "pose"},
        stream_codecs={"color_image": "lz4+cdr"},
    )
    connect(recorder, color_image="/camera", odometry="/odom")

    specs = recorder._stream_specs()
    recorder.config.streams = specs
    config = recorder.config.to_config_dict()

    assert config["encoding_threads"] == 7
    assert config["store"] == {
        "kind": "sqlite",
        "path": str(tmp_path / "recording.db"),
    }
    assert config["streams"] == [
        {
            "port": "color_image",
            "name": "color_image",
            "payload_type": "sensor_msgs/msg/Image",
            "codec": "lz4+cdr",
            "cdr_schema": recording_schema(Image),
            "source_timestamp": True,
        },
        {
            "port": "odometry",
            "name": "pose",
            "payload_type": "geometry_msgs/msg/PoseStamped",
            "codec": "cdr",
            "cdr_schema": recording_schema(PoseStamped),
            "source_timestamp": True,
        },
    ]
    assert set(config) == {"encoding_threads", "store", "streams"}


def test_json_options_cross_the_native_boundary_only_when_configured() -> None:
    schema = {"type": "object", "properties": {"sent": {"type": "number"}}}
    config = RustRecorderConfig(
        streams=[
            RustStreamSpec(
                port="events",
                name="events",
                payload_type="std_msgs/msg/String",
                codec="json",
                timestamp_field="sent",
                json_schema=schema,
            )
        ]
    )

    assert config.to_config_dict()["streams"] == [
        {
            "port": "events",
            "name": "events",
            "payload_type": "std_msgs/msg/String",
            "codec": "json",
            "timestamp_field": "sent",
            "json_schema": schema,
        }
    ]


@pytest.mark.parametrize("codec", ["cdr", "cdrjpeg", "lz4+cdr", "json"])
def test_unconfigured_json_options_are_omitted_from_native_streams(codec: str) -> None:
    config = RustRecorderConfig(
        streams=[
            RustStreamSpec(port="samples", name="samples", payload_type="test.Raw", codec=codec)
        ]
    )

    assert config.to_config_dict()["streams"] == [
        {"port": "samples", "name": "samples", "payload_type": "test.Raw", "codec": codec}
    ]


def test_store_preparation_creates_a_python_readable_registry(
    tmp_path: Path, make_recorder: Any
) -> None:
    path = tmp_path / "recording.db"
    recorder = make_recorder(
        SampleRustRecorder,
        store=RustSqliteStoreConfig(path=str(path)),
        record_tf=False,
    )
    connect(recorder, color_image="/camera", odometry="/odom")
    specs = recorder._stream_specs()

    assert [spec.codec for spec in specs] == ["cdrjpeg", "cdr"]

    recorder._prepare_store(specs)

    with SqliteStore(path=str(path), must_exist=True) as store:
        assert store.list_streams() == ["color_image", "odometry"]
        assert store.stream("color_image").count() == 0
        assert store.stream("odometry").count() == 0


def test_append_replaces_only_the_recorded_streams(tmp_path: Path, make_recorder: Any) -> None:
    path = tmp_path / "recording.db"
    with SqliteStore(path=str(path)) as store:
        store.stream("keep", PoseStamped).append(
            PoseStamped(
                header=Header(stamp=Time(0, 0), frame_id=""),
                pose=Pose(
                    position=Point(0.0, 0.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
                ),
            ),
            ts=1.0,
        )
        store.stream("odometry", PoseStamped).append(
            PoseStamped(
                header=Header(stamp=Time(0, 0), frame_id=""),
                pose=Pose(
                    position=Point(0.0, 0.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
                ),
            ),
            ts=2.0,
        )

    recorder = make_recorder(
        SampleRustRecorder,
        store=RustSqliteStoreConfig(path=str(path)),
        record_tf=False,
        on_existing=OnExisting.APPEND,
    )
    connect(recorder, odometry="/odom")
    recorder._prepare_store(recorder._stream_specs())

    with SqliteStore(path=str(path), must_exist=True) as store:
        assert store.stream("keep").count() == 1
        assert store.stream("odometry").count() == 0


def test_unsupported_python_payload_fails_before_native_process_starts(
    tmp_path: Path, make_recorder: Any
) -> None:
    recorder = make_recorder(
        UnsupportedRustRecorder,
        store=RustSqliteStoreConfig(path=str(tmp_path / "recording.db")),
        record_tf=False,
    )
    connect(recorder, values="/values")

    with pytest.raises(TypeError, match="only supports registered generated messages"):
        recorder._stream_specs()


@pytest.mark.parametrize("encoding_threads", [0, -1])
def test_encoding_threads_must_be_positive(encoding_threads: int) -> None:
    with pytest.raises(ValueError, match="encoding_threads"):
        RustRecorderConfig(encoding_threads=encoding_threads)


def test_default_store_path_is_resolved_from_the_project_root() -> None:
    config = RustRecorderConfig()

    assert Path(config.store.path).is_absolute()
    assert Path(config.store.path).name == "recording.db"


def test_native_recorder_is_built_and_run_from_the_nix_package() -> None:
    config = RustRecorderConfig()

    assert config.source_dir == "dimos/experimental/memory/rust"
    assert config.build_command == "nix build -L path:."
    assert config.executable == "result/bin/dimos-memory-recorder"


def test_invalid_codec_fails_during_preflight(tmp_path: Path, make_recorder: Any) -> None:
    recorder = make_recorder(
        SampleRustRecorder,
        store=RustSqliteStoreConfig(path=str(tmp_path / "recording.db")),
        record_tf=False,
        stream_codecs={"odometry": "pickle"},
    )
    connect(recorder, odometry="/odom")

    with pytest.raises(ValueError, match="Unsupported native codec"):
        recorder._stream_specs()


def test_jpeg_requires_an_image_stream(tmp_path: Path, make_recorder: Any) -> None:
    recorder = make_recorder(
        SampleRustRecorder,
        store=RustSqliteStoreConfig(path=str(tmp_path / "recording.db")),
        record_tf=False,
        stream_codecs={"odometry": "cdrjpeg"},
    )
    connect(recorder, odometry="/odom")

    with pytest.raises(TypeError, match="JPEG codec requires Image"):
        recorder._stream_specs()


def test_unconnected_recorder_does_not_spawn_or_touch_the_store(
    tmp_path: Path, make_recorder: Any
) -> None:
    path = tmp_path / "recording.db"
    recorder = make_recorder(SampleRustRecorder, store=RustSqliteStoreConfig(path=str(path)))

    recorder.start()

    assert recorder._process is None
    assert not path.exists()


def test_mcap_store_uses_python_codec_defaults_and_does_not_precreate_the_artifact(
    tmp_path: Path, make_recorder: Any
) -> None:
    path = tmp_path / "recording.mcap"
    recorder = make_recorder(
        SampleRustRecorder,
        store=RustMcapStoreConfig(path=str(path)),
        record_tf=False,
    )
    connect(recorder, color_image="/camera", odometry="/odom")

    specs = recorder._stream_specs()
    recorder._prepare_store(specs)

    assert [spec.codec for spec in specs] == ["cdrjpeg", "cdr"]
    assert not path.exists()
    recorder.config.streams = specs
    assert recorder.config.to_config_dict()["store"] == {
        "kind": "mcap",
        "path": str(path),
    }


def test_mcap_accepts_storage_codecs_and_rejects_append(tmp_path: Path, make_recorder: Any) -> None:
    path = tmp_path / "recording.mcap"
    recorder = make_recorder(
        SampleRustRecorder,
        store=RustMcapStoreConfig(path=str(path)),
        record_tf=False,
        stream_codecs={"color_image": "cdrjpeg"},
    )
    connect(recorder, color_image="/camera")

    assert [spec.codec for spec in recorder._stream_specs()] == ["cdrjpeg"]

    append_recorder = make_recorder(
        SampleRustRecorder,
        store=RustMcapStoreConfig(path=str(path)),
        record_tf=False,
        on_existing=OnExisting.APPEND,
    )
    connect(append_recorder, odometry="/odom")
    with pytest.raises(ValueError, match="MCAP append is unsupported"):
        append_recorder._prepare_store(append_recorder._stream_specs())


def test_recorder_launches_without_duplicate_topic_cli_args(
    tmp_path: Path, make_recorder: Any
) -> None:
    recorder = make_recorder(
        SampleRustRecorder,
        store=RustSqliteStoreConfig(path=str(tmp_path / "recording.db")),
    )

    assert recorder._argv({"odometry": "/odom"}) == [recorder._executable]


def test_recorder_rejects_native_cli_arguments() -> None:
    with pytest.raises(ValueError, match="stdin-only"):
        RustRecorderConfig(extra_args=["--diagnostic"])


def test_duplicate_remapped_stream_names_fail_before_launch(
    tmp_path: Path, make_recorder: Any
) -> None:
    recorder = make_recorder(
        SampleRustRecorder,
        store=RustSqliteStoreConfig(path=str(tmp_path / "recording.db")),
        record_tf=False,
        stream_remapping={"color_image": "samples", "odometry": "samples"},
    )
    connect(recorder, color_image="/camera", odometry="/odom")

    with pytest.raises(ValueError, match="Duplicate recorded stream names"):
        recorder._stream_specs()


def test_replay_never_prepares_native_sources_or_process(tmp_path, make_recorder, mocker):
    fetch = mocker.patch(
        "dimos.core.native_module.get_project_root", side_effect=AssertionError("fetch")
    )
    spawn = mocker.patch(
        "dimos.core.native_module.subprocess.Popen", side_effect=AssertionError("spawn")
    )
    start = mocker.patch.object(Module, "start")
    path = tmp_path / "replay.db"
    recorder = make_recorder(
        SampleRustRecorder,
        g=GlobalConfig(replay=True),
        store=RustSqliteStoreConfig(path=str(path)),
    )
    recorder.build()
    recorder.start()
    fetch.assert_not_called()
    spawn.assert_not_called()
    start.assert_called_once_with(recorder)
    assert not path.exists()

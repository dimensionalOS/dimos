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

from dimos.core.stream import In
from dimos.experimental.memory.rust_recorder import (
    RustMcapStoreConfig,
    RustRecorder,
    RustRecorderConfig,
    RustSqliteStoreConfig,
)
from dimos.memory.module import OnExisting
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image


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
        stream_codecs={"color_image": "lz4+lcm"},
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
            "payload_type": "dimos.msgs.sensor_msgs.Image.Image",
            "codec": "lz4+lcm",
        },
        {
            "port": "odometry",
            "name": "pose",
            "payload_type": "dimos.msgs.geometry_msgs.PoseStamped.PoseStamped",
            "codec": "lcm",
        },
    ]
    assert set(config) == {"encoding_threads", "store", "streams"}


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

    assert [spec.codec for spec in specs] == ["jpeg", "lcm"]

    recorder._prepare_store(specs)

    with SqliteStore(path=str(path), must_exist=True) as store:
        assert store.list_streams() == ["color_image", "odometry"]
        assert store.stream("color_image").count() == 0
        assert store.stream("odometry").count() == 0


def test_append_replaces_only_the_recorded_streams(tmp_path: Path, make_recorder: Any) -> None:
    path = tmp_path / "recording.db"
    with SqliteStore(path=str(path)) as store:
        store.stream("keep", PoseStamped).append(PoseStamped(), ts=1.0)
        store.stream("odometry", PoseStamped).append(PoseStamped(), ts=2.0)

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

    with pytest.raises(TypeError, match="only supports LCM-backed messages"):
        recorder._stream_specs()


@pytest.mark.parametrize("encoding_threads", [0, -1])
def test_encoding_threads_must_be_positive(encoding_threads: int) -> None:
    with pytest.raises(ValueError, match="encoding_threads"):
        RustRecorderConfig(encoding_threads=encoding_threads)


def test_default_store_path_is_resolved_from_the_project_root() -> None:
    config = RustRecorderConfig()

    assert Path(config.store.path).is_absolute()
    assert Path(config.store.path).name == "recording.db"


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


def test_unconnected_recorder_does_not_spawn_or_touch_the_store(
    tmp_path: Path, make_recorder: Any
) -> None:
    path = tmp_path / "recording.db"
    recorder = make_recorder(SampleRustRecorder, store=RustSqliteStoreConfig(path=str(path)))

    recorder.start()

    assert recorder._process is None
    assert not path.exists()


def test_mcap_store_uses_wire_codecs_and_does_not_precreate_the_artifact(
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

    assert [spec.codec for spec in specs] == ["lcm", "lcm"]
    assert not path.exists()
    recorder.config.streams = specs
    assert recorder.config.to_config_dict()["store"] == {
        "kind": "mcap",
        "path": str(path),
    }


def test_mcap_rejects_storage_codecs_and_append(tmp_path: Path, make_recorder: Any) -> None:
    path = tmp_path / "recording.mcap"
    recorder = make_recorder(
        SampleRustRecorder,
        store=RustMcapStoreConfig(path=str(path)),
        record_tf=False,
        stream_codecs={"color_image": "jpeg"},
    )
    connect(recorder, color_image="/camera")

    with pytest.raises(ValueError, match="original wire packets"):
        recorder._stream_specs()

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

    assert recorder._argv({"odometry": "/odom"}) == [recorder.config.executable]


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

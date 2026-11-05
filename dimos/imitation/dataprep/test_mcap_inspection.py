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

"""MCAP-first recording inspection readiness (DIM-481).

Fixtures are bounded synthetic recordings written with the same channel
metadata the native CollectionModule recorder emits (``dimos.payload_type``
and ``dimos.observation_time=publish_time``), so they exercise the real
schema.json + recording.mcap read path. They are synthetic stand-ins: no
claim is made about real robot manipulation recordings.
"""

from __future__ import annotations

import builtins
import errno
import hashlib
from pathlib import Path
from typing import Any

from mcap.writer import Writer as McapWriter
import numpy as np
import pytest

from dimos.imitation.collection.episode import EpisodeEvent, EpisodeStatus
from dimos.imitation.dataprep.build import _open_recording, inspect_recording
from dimos.imitation.dataprep.core import inspect_episodes
from dimos.imitation.dataprep.schema import (
    DataPrepConfig,
    EpisodeExtractor,
    FeatureSpec,
    RecordingSchema,
    SyncConfig,
)
from dimos.memory.codecs.jpeg import JpegCodec
from dimos.memory.store.mcap import McapStore
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.String import String


def _register_channel(
    writer: McapWriter,
    topic: str,
    payload_type: type[Any],
    message_encoding: str = "lcm",
    *,
    observation_time: str | None = "publish_time",
) -> int:
    metadata = {"dimos.payload_type": f"{payload_type.__module__}.{payload_type.__qualname__}"}
    if observation_time is not None:
        metadata["dimos.observation_time"] = observation_time
    return writer.register_channel(
        topic=topic,
        message_encoding=message_encoding,
        schema_id=0,
        metadata=metadata,
    )


def _status_json(ts: float, event: EpisodeEvent, label: str | None = "pick") -> bytes:
    return (
        EpisodeStatus(
            ts=ts,
            state="recording" if event == "start" else "idle",
            last_event=event,
            episodes_saved=1,
            episodes_discarded=0,
            task_label=label,
        )
        .to_json()
        .encode()
    )


def _add(
    writer: McapWriter,
    channel_id: int,
    log_ts: float,
    publish_ts: float,
    data: bytes,
) -> None:
    writer.add_message(
        channel_id=channel_id,
        log_time=round(log_ts * 1_000_000_000),
        publish_time=round(publish_ts * 1_000_000_000),
        data=data,
    )


def _write_recording(path: Path, *, status_topic: str = "status") -> None:
    """One saved episode (10.0s-10.1s): 30Hz RGB camera + joint state."""
    with path.open("wb") as output:
        writer = McapWriter(output)
        writer.start(profile="dimos", library="test")
        channels = {
            "camera": _register_channel(writer, "color_image", Image, "jpeg"),
            "state": _register_channel(writer, "joint_state", JointState),
            "status": _register_channel(writer, status_topic, String, "json"),
        }
        _add(writer, channels["status"], 10.0, 10.0, _status_json(10.0, "start"))
        for index in range(4):
            ts = 10.0 + index / 30.0
            _add(
                writer,
                channels["camera"],
                ts,
                ts,
                JpegCodec().encode(
                    Image(
                        ts=ts,
                        frame_id="wrist_camera_link",
                        format=ImageFormat.RGB,
                        data=np.full((8, 8, 3), index, dtype=np.uint8),
                    )
                ),
            )
            _add(
                writer,
                channels["state"],
                ts,
                ts,
                JointState(
                    ts=ts,
                    name=["shoulder", "wrist"],
                    position=[float(index), float(index + 1)],
                ).lcm_encode(),
            )
        end = 10.0 + 3 / 30.0
        _add(writer, channels["status"], end, end, _status_json(end, "save"))
        writer.finish()


def _schema(**episodes: Any) -> RecordingSchema:
    return RecordingSchema(
        episodes=EpisodeExtractor(**episodes),
        observation={
            "image": FeatureSpec(
                stream="color_image",
                field="data",
                dtype="video",
                shape=(8, 8, 3),
                names=["height", "width", "channels"],
            ),
            "state": FeatureSpec(
                stream="joint_state",
                field="position",
                dtype="float32",
                shape=(2,),
                names=["shoulder", "wrist"],
            ),
        },
        sync=SyncConfig(anchor="image", rate_hz=30.0, tolerance_ms=20.0),
    )


def _config(source: Path, **episodes: Any) -> DataPrepConfig:
    return DataPrepConfig(recording_schema=_schema(**episodes), source=str(source))


def test_inspection_uses_schema_declared_status_stream(tmp_path: Path) -> None:
    """The recorder remaps the status stream in schema.json; inspection must follow it."""
    source = tmp_path / "recording.mcap"
    _write_recording(source, status_topic="recorder_episode_status")

    info = inspect_recording(
        source, config=_config(source, status_stream="recorder_episode_status")
    )

    assert info["status_stream"] == "recorder_episode_status"
    assert info["episodes"] == 1
    assert info["saved_episodes"] == 1
    assert info["quality"][0]["valid"] is True


def test_inspection_uses_schema_ranges_extractor(tmp_path: Path) -> None:
    """Ranges-configured recordings need no status stream at all."""
    source = tmp_path / "recording.mcap"
    _write_recording(source)

    config = _config(source, extractor="ranges", ranges=[(10.0, 10.0 + 3 / 30.0)])
    info = inspect_recording(source, config=config)

    assert info["status_stream"] is None
    assert info["episodes"] == 1
    assert info["saved_episodes"] == 1
    assert info["quality"][0]["emitted_frames"] == 4


def test_inspection_without_config_keeps_standalone_status_stream(tmp_path: Path) -> None:
    source = tmp_path / "recording.mcap"
    _write_recording(source, status_topic="recorder_episode_status")

    default = inspect_recording(source)
    assert default["status_stream"] is None  # no stream literally named "status"
    assert default["episodes"] == 0

    explicit = inspect_recording(source, status_stream="recorder_episode_status")
    assert explicit["status_stream"] == "recorder_episode_status"
    assert explicit["saved_episodes"] == 1


def test_inspection_leaves_recording_source_immutable(tmp_path: Path) -> None:
    source = tmp_path / "recording.mcap"
    _write_recording(source)
    before_bytes = hashlib.sha256(source.read_bytes()).hexdigest()
    before_stat = source.stat()
    before_listing = sorted(p.name for p in tmp_path.iterdir())

    info = inspect_recording(source, config=_config(source))
    assert info["saved_episodes"] == 1

    assert hashlib.sha256(source.read_bytes()).hexdigest() == before_bytes
    after_stat = source.stat()
    assert (after_stat.st_mtime_ns, after_stat.st_size) == (
        before_stat.st_mtime_ns,
        before_stat.st_size,
    )
    assert sorted(p.name for p in tmp_path.iterdir()) == before_listing  # no sidecars


def test_inspection_rejects_writes_on_the_read_path(tmp_path: Path) -> None:
    source = tmp_path / "recording.mcap"
    _write_recording(source)
    with _open_recording(source) as store:
        with pytest.raises(NotImplementedError, match="read-only"):
            store.stream("joint_state").append(
                JointState(ts=99.0, name=["shoulder"], position=[0.0]), ts=99.0
            )


def test_episode_bounds_come_from_the_status_document_not_reception_time(
    tmp_path: Path,
) -> None:
    """Reception (log) time must never shift an episode boundary."""
    source = tmp_path / "recording.mcap"
    with source.open("wb") as output:
        writer = McapWriter(output)
        writer.start(profile="dimos", library="test")
        status = _register_channel(writer, "status", String, "json")
        # Button press at 10.0/11.0 received 2s later.
        _add(writer, status, 12.0, 10.0, _status_json(10.0, "start", label="place"))
        _add(writer, status, 13.0, 11.0, _status_json(11.0, "save", label="place"))
        # A second episode left open at EOF.
        _add(writer, status, 14.0, 12.0, _status_json(12.0, "start", label="pick"))
        writer.finish()

    info = inspect_recording(source)

    assert info["saved_episodes"] == 1
    assert info["incomplete_episodes"] == [{"start_ts": 12.0, "task_label": "pick"}]
    with _open_recording(source) as store:
        episodes = inspect_episodes(store, EpisodeExtractor(status_stream="status")).episodes
    assert [(e.start_ts, e.end_ts, e.task_label) for e in episodes] == [(10.0, 11.0, "place")]


def test_observation_time_metadata_selects_publish_over_log_time(tmp_path: Path) -> None:
    source = tmp_path / "recording.mcap"
    with source.open("wb") as output:
        writer = McapWriter(output)
        writer.start(profile="dimos", library="test")
        publish_basis = _register_channel(writer, "source_stamped", JointState)
        log_basis = _register_channel(
            writer, "reception_stamped", JointState, observation_time=None
        )
        for channel in (publish_basis, log_basis):
            _add(
                writer,
                channel,
                log_ts=20.0,
                publish_ts=15.0,
                data=JointState(ts=15.0, name=["j"], position=[0.0]).lcm_encode(),
            )
        writer.finish()

    with _open_recording(source) as store:
        assert store.stream("source_stamped").first().ts == 15.0  # publish_time
        assert store.stream("reception_stamped").first().ts == 20.0  # log_time fallback


def test_jpeg_video_decodes_to_rgb_without_channel_swap(tmp_path: Path) -> None:
    source = tmp_path / "recording.mcap"
    with source.open("wb") as output:
        writer = McapWriter(output)
        writer.start(profile="dimos", library="test")
        channel = _register_channel(writer, "color_image", Image, "jpeg")
        pixels = np.zeros((4, 4, 3), dtype=np.uint8)
        pixels[..., 0] = 200  # red-dominant
        pixels[..., 1] = 20
        pixels[..., 2] = 40
        _add(
            writer,
            channel,
            10.0,
            10.0,
            JpegCodec(quality=95).encode(
                Image(ts=10.0, frame_id="cam", format=ImageFormat.RGB, data=pixels)
            ),
        )
        writer.finish()

    with _open_recording(source) as store:
        decoded = store.stream("color_image").first().data

    assert decoded.format is ImageFormat.RGB
    assert decoded.data.dtype == np.uint8
    assert decoded.data.shape == (4, 4, 3)
    mean = decoded.data.reshape(-1, 3).mean(axis=0)
    assert mean[0] > mean[2] > mean[1]  # R=200, B=40, G=20 survive lossy decode in order
    assert mean[0] - mean[2] > 50  # a BGR swap would flip the dominant channel


def test_corrupt_video_payload_rejects_episode_with_decode_diagnostic(tmp_path: Path) -> None:
    source = tmp_path / "recording.mcap"
    with source.open("wb") as output:
        writer = McapWriter(output)
        writer.start(profile="dimos", library="test")
        camera = _register_channel(writer, "color_image", Image, "jpeg")
        status = _register_channel(writer, "status", String, "json")
        _add(writer, status, 10.0, 10.0, _status_json(10.0, "start"))
        _add(writer, camera, 10.0, 10.0, b"not-a-jpeg-payload")
        _add(writer, status, 10.1, 10.1, _status_json(10.1, "save"))
        writer.finish()

    info = inspect_recording(source, config=_config(source))

    assert info["saved_episodes"] == 1
    [report] = info["quality"]
    assert report["valid"] is False
    assert any("Decode error" in reason for reason in report["rejection_reasons"])


def test_unsupported_episode_status_version_is_an_explicit_error(tmp_path: Path) -> None:
    source = tmp_path / "recording.mcap"
    with source.open("wb") as output:
        writer = McapWriter(output)
        writer.start(profile="dimos", library="test")
        status = _register_channel(writer, "status", String, "json")
        document = _status_json(10.0, "start").replace(
            b'"schema_version": 1', b'"schema_version": 2'
        )
        _add(writer, status, 10.0, 10.0, document)
        writer.finish()

    with pytest.raises(ValueError, match="Unsupported EpisodeStatus schema_version"):
        inspect_recording(source)


def test_unfinalized_recording_reports_missing_summary_index(tmp_path: Path) -> None:
    """An interrupted recording has no MCAP summary index; say so explicitly."""
    source = tmp_path / "recording.mcap"
    with source.open("wb") as output:
        writer = McapWriter(output)
        writer.start(profile="dimos", library="test")
        status = _register_channel(writer, "status", String, "json")
        _add(writer, status, 10.0, 10.0, _status_json(10.0, "start"))
        # No finish(): the footer/summary section is never written.
    before_bytes = hashlib.sha256(source.read_bytes()).hexdigest()

    with pytest.raises(ValueError, match="no readable summary index"):
        inspect_recording(source)
    with pytest.raises(ValueError, match="no readable summary index"):
        McapStore(path=str(source))

    # The failed read must not modify or "repair" the source in place.
    assert hashlib.sha256(source.read_bytes()).hexdigest() == before_bytes


def test_missing_recording_is_a_plain_missing_file_error(tmp_path: Path) -> None:
    absent = tmp_path / "absent.mcap"
    with pytest.raises(FileNotFoundError):
        inspect_recording(absent)
    assert not absent.exists()  # never created as a side effect


@pytest.mark.parametrize("code", [errno.EACCES, errno.EIO, errno.EISDIR])
def test_filesystem_failure_keeps_native_error(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, code: int
) -> None:
    """EACCES/EIO/EISDIR at open time are not MCAP corruption: the native
    OSError subclass and errno must survive, not a truncation ValueError."""

    def fail(*args: Any, **kwargs: Any) -> None:
        raise OSError(code, "simulated filesystem failure")

    monkeypatch.setattr(builtins, "open", fail)
    with pytest.raises(OSError) as caught:
        McapStore(path=str(tmp_path / "source.mcap"))
    assert caught.value.errno == code
    assert not isinstance(caught.value, ValueError)


def test_directory_path_is_not_reported_as_corruption(tmp_path: Path) -> None:
    with pytest.raises(IsADirectoryError):
        McapStore(path=str(tmp_path))


def test_default_finalized_empty_recording_is_supported(tmp_path: Path) -> None:
    """A finalized empty MCAP (default writer settings) opens with no streams."""
    source = tmp_path / "empty.mcap"
    with source.open("wb") as output:
        writer = McapWriter(output)
        writer.start(profile="dimos", library="test")
        writer.finish()

    with McapStore(path=str(source)) as store:
        assert store.list_streams() == []


def test_finalized_recording_without_statistics_names_indexing_policy(
    tmp_path: Path,
) -> None:
    """A valid finalized MCAP written without statistics is well-formed input
    the read-only store does not support: the error must name the missing
    indexing, not claim the file is truncated or corrupt."""
    source = tmp_path / "unindexed.mcap"
    with source.open("wb") as output:
        writer = McapWriter(output, use_statistics=False)
        writer.start(profile="dimos", library="test")
        writer.finish()

    with pytest.raises(ValueError, match="no summary statistics") as caught:
        McapStore(path=str(source))
    message = str(caught.value)
    assert "indexed" in message
    assert "truncated" not in message
    assert "corrupt" not in message

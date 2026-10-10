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

"""DataPrep interoperability with self-describing native MCAP recordings."""

from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path
from typing import Any
import weakref

from dimos_generated.sensor_msgs.msg import Image, JointState
from dimos_generated.std_msgs.msg import Header, String
from dimos_message_build.registry import encode as cdr_encode, schema as message_schema
from mcap.writer import Writer as McapWriter
import numpy as np
import pytest

from dimos.imitation.collection.episode import EpisodeStatus
from dimos.imitation.dataprep.build import _open_recording, inspect_recording, run_dataprep
from dimos.imitation.dataprep.core import extract_episodes, iter_episode_samples
from dimos.imitation.dataprep.schema import (
    DataPrepConfig,
    FeatureSpec,
    OutputConfig,
    Sample,
    SyncConfig,
)
from dimos.memory.codecs.cdr import CdrCodec
from dimos.memory.codecs.lz4 import Lz4Codec
from dimos.msgs.image import image_from_array
from dimos.msgs.time import time_from_seconds


def _register_channel(
    writer: McapWriter,
    name: str,
    payload_type: type[Any],
    message_encoding: str = "cdr",
) -> int:
    schema_id = 0
    if message_encoding in {"cdr", "lz4+cdr"}:
        schema_id = writer.register_schema(
            name=payload_type.__msgtype__,
            encoding="ros2msg",
            data=message_schema(payload_type.__msgtype__).encode(),
        )
    return writer.register_channel(
        topic=name,
        message_encoding=message_encoding,
        schema_id=schema_id,
        metadata={
            "dimos.payload_type": f"{payload_type.__module__}.{payload_type.__qualname__}",
            "dimos.observation_time": "publish_time",
        },
    )


def _write_message(writer: McapWriter, channel_id: int, ts: float, message: Any) -> None:
    timestamp_ns = round(ts * 1_000_000_000)
    writer.add_message(
        channel_id=channel_id,
        log_time=timestamp_ns,
        publish_time=timestamp_ns,
        data=message.to_json().encode("utf-8")
        if isinstance(message, EpisodeStatus)
        else (message if isinstance(message, bytes) else cdr_encode(message)),
    )


def _write_collection(path: Path) -> None:
    with path.open("wb") as output:
        writer = McapWriter(output)
        writer.start(profile="dimos", library="test")
        channels = {
            "color_image": _register_channel(writer, "color_image", Image),
            "coordinator_joint_state": _register_channel(
                writer, "coordinator_joint_state", JointState
            ),
            "applied_joint_position_command": _register_channel(
                writer, "applied_joint_position_command", JointState
            ),
            "status": _register_channel(writer, "status", String, "json"),
        }
        _write_message(
            writer,
            channels["status"],
            10.0,
            EpisodeStatus(
                ts=10.0,
                state="recording",
                last_event="start",
                episodes_saved=0,
                episodes_discarded=0,
                task_label="pick",
            ),
        )
        for index in range(3):
            ts = 10.0 + index / 30.0
            _write_message(
                writer,
                channels["color_image"],
                ts,
                image_from_array(
                    np.full((8, 8, 3), index, dtype=np.uint8),
                    encoding="rgb8",
                    header=Header(stamp=time_from_seconds(ts), frame_id="wrist_camera_link"),
                ),
            )
            _write_message(
                writer,
                channels["coordinator_joint_state"],
                ts,
                JointState(
                    header=Header(stamp=time_from_seconds(ts), frame_id="coordinator"),
                    name=["shoulder", "wrist"],
                    position=np.array([float(index), float(index + 1)]),
                    velocity=np.zeros(2),
                    effort=np.zeros(2),
                ),
            )
            _write_message(
                writer,
                channels["applied_joint_position_command"],
                ts,
                JointState(
                    header=Header(stamp=time_from_seconds(ts), frame_id="coordinator"),
                    name=["shoulder", "wrist"],
                    position=np.array([float(index) + 0.25, float(index) + 1.25]),
                    velocity=np.array([], dtype=np.float64),
                    effort=np.array([], dtype=np.float64),
                ),
            )
        _write_message(
            writer,
            channels["status"],
            10.0 + 2 / 30.0,
            EpisodeStatus(
                ts=10.0 + 2 / 30.0,
                state="idle",
                last_event="save",
                episodes_saved=1,
                episodes_discarded=0,
                task_label="pick",
            ),
        )
        writer.finish()


def _config(source: Path, output: Path) -> DataPrepConfig:
    names = ["shoulder", "wrist"]
    return DataPrepConfig(
        source=str(source),
        observation={
            "observation.images.wrist": FeatureSpec(
                stream="color_image",
                field=None,
                dtype="video",
                shape=(8, 8, 3),
                names=["height", "width", "channels"],
            ),
            "observation.state": FeatureSpec(
                stream="coordinator_joint_state",
                field="position",
                dtype="float32",
                shape=(2,),
                names=names,
            ),
        },
        action={
            "action": FeatureSpec(
                stream="applied_joint_position_command",
                field="position",
                dtype="float32",
                shape=(2,),
                names=names,
            )
        },
        sync=SyncConfig(
            anchor="observation.images.wrist",
            rate_hz=30.0,
            tolerance_ms=20.0,
        ),
        output=OutputConfig(format="hdf5", path=output),
    )


def test_mcap_recording_inspects_and_produces_valid_samples(tmp_path: Path) -> None:
    source = tmp_path / "session.mcap"
    output = tmp_path / "dataset"
    output.mkdir()
    _write_collection(source)
    config = _config(source, output)
    received: list[Sample] = []

    def writer(samples: Iterator[Sample], selected_output: OutputConfig) -> Path:
        received.extend(samples)
        return selected_output.path

    info = inspect_recording(source, config=config)
    dataset_path = run_dataprep(config, writer=writer)

    assert info["streams"] == {
        "applied_joint_position_command": 3,
        "color_image": 3,
        "coordinator_joint_state": 3,
        "status": 2,
    }
    assert info["saved_episodes"] == 1
    assert info["quality"][0]["valid"] is True
    assert dataset_path == output
    assert len(received) == 3
    np.testing.assert_array_equal(received[0].observation["observation.state"], [0.0, 1.0])
    np.testing.assert_array_equal(received[0].action["action"], [0.25, 1.25])


@pytest.mark.parametrize("encoding", ["cdr", "lz4+cdr"])
def test_recording_metadata_decodes_typed_messages(tmp_path: Path, encoding: str) -> None:
    path = tmp_path / "recording.mcap"
    message = JointState(
        header=Header(stamp=time_from_seconds(12.5), frame_id="arm"),
        name=["arm/joint1"],
        position=np.array([0.25]),
        velocity=np.array([], dtype=np.float64),
        effort=np.array([], dtype=np.float64),
    )
    codec = CdrCodec(JointState)
    payload = Lz4Codec(codec).encode(message) if encoding == "lz4+cdr" else codec.encode(message)
    with path.open("wb") as file:
        writer = McapWriter(file)
        writer.start()
        channel = _register_channel(writer, "measured", JointState, encoding)
        _write_message(writer, channel, 11.5, payload)
        writer.finish()

    with _open_recording(path) as store:
        observation = store.stream("measured").first()
        assert observation.ts == 11.5
        assert cdr_encode(observation.data) == cdr_encode(message)


def test_missing_message_package_reports_the_recorded_type(tmp_path: Path) -> None:
    path = tmp_path / "recording.mcap"
    with path.open("wb") as file:
        writer = McapWriter(file)
        writer.start()
        channel = writer.register_channel(
            topic="custom_state",
            message_encoding="cdr",
            schema_id=0,
            metadata={"dimos.payload_type": "missing_recording_package.State"},
        )
        _write_message(writer, channel, 1.0, b"unused")
        writer.finish()

    with pytest.raises(ImportError, match="custom_state.*missing_recording_package.State"):
        _open_recording(path)


def test_sample_emission_does_not_keep_every_decoded_camera_frame(tmp_path, mocker):
    recording = tmp_path / "session.mcap"
    _write_collection(recording)
    config = _config(recording, tmp_path / "dataset")
    config.observation["camera_alias"] = config.observation["observation.images.wrist"]
    decoded = []
    decode = CdrCodec.decode

    def track_decode(codec, payload):
        image = decode(codec, payload)
        if isinstance(image, Image):
            decoded.append(weakref.ref(image.data))
        return image

    mocker.patch.object(CdrCodec, "decode", autospec=True, side_effect=track_decode)
    with _open_recording(recording) as store:
        episode = extract_episodes(store, config.episodes)[0]
        samples = iter_episode_samples(
            store,
            episode,
            {**config.observation, **config.action},
            config.sync,
            config.quality,
            obs_keys=set(config.observation),
            action_keys=set(config.action),
        )
        first = next(samples)

        assert sum(reference() is not None for reference in decoded) <= 2
        np.testing.assert_array_equal(
            first.observation["observation.images.wrist"], np.zeros((8, 8, 3), dtype=np.uint8)
        )
        np.testing.assert_array_equal(
            first.observation["camera_alias"], first.observation["observation.images.wrist"]
        )
        samples.close()

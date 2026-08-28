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

from mcap.writer import Writer as McapWriter
import numpy as np

from dimos.imitation.dataprep.build import inspect_recording, run_dataprep
from dimos.imitation.dataprep.core import (
    DataPrepConfig,
    FeatureSpec,
    OutputConfig,
    Sample,
    SyncConfig,
)
from dimos.msgs.imitation_msgs.EpisodeStatus import EpisodeStatus
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState


def _register_channel(writer: McapWriter, name: str, payload_type: type[Any]) -> int:
    return writer.register_channel(
        topic=name,
        message_encoding="lcm",
        schema_id=0,
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
        data=message.lcm_encode(),
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
            "status": _register_channel(writer, "status", EpisodeStatus),
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
                Image(
                    ts=ts,
                    frame_id="wrist_camera_link",
                    format=ImageFormat.RGB,
                    data=np.full((8, 8, 3), index, dtype=np.uint8),
                ),
            )
            _write_message(
                writer,
                channels["coordinator_joint_state"],
                ts,
                JointState(
                    ts=ts,
                    frame_id="coordinator",
                    name=["shoulder", "wrist"],
                    position=[float(index), float(index + 1)],
                    velocity=[0.0, 0.0],
                    effort=[0.0, 0.0],
                ),
            )
            _write_message(
                writer,
                channels["applied_joint_position_command"],
                ts,
                JointState(
                    ts=ts,
                    frame_id="coordinator",
                    name=["shoulder", "wrist"],
                    position=[float(index) + 0.25, float(index) + 1.25],
                    velocity=[],
                    effort=[],
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
                field="data",
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

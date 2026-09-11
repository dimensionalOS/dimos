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

"""Full native-MCAP to LeRobot conversion in the locked policy environment."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Literal

from dimos_lerobot.dataprep import write
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from mcap.writer import Writer as McapWriter
import numpy as np
import pytest

from dimos.imitation.collection.profile import CollectionFeature, CollectionProfile
from dimos.imitation.collection.recording import RecordingSchema
from dimos.imitation.dataprep.build import run_dataprep
from dimos.imitation.dataprep.core import (
    OutputConfig,
    SyncConfig,
)
from dimos.memory.codecs.jpeg import JpegCodec
from dimos.msgs.imitation_msgs.EpisodeStatus import EpisodeStatus
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState

JOINTS = [f"joint_{i}" for i in range(14)]


def _channel(
    writer: McapWriter,
    name: str,
    payload_type: type[Any],
    message_encoding: str = "lcm",
) -> int:
    return int(
        writer.register_channel(
            topic=name,
            message_encoding=message_encoding,
            schema_id=0,
            metadata={
                "dimos.payload_type": f"{payload_type.__module__}.{payload_type.__qualname__}",
                "dimos.observation_time": "publish_time",
            },
        )
    )


def _add(writer: McapWriter, channel_id: int, ts: float, message: Any) -> None:
    timestamp_ns = round(ts * 1_000_000_000)
    writer.add_message(
        channel_id=channel_id,
        log_time=timestamp_ns,
        publish_time=timestamp_ns,
        data=message if isinstance(message, bytes) else message.lcm_encode(),
    )


def _record(path: Path, camera_count: int) -> None:
    with path.open("wb") as output:
        writer = McapWriter(output)
        writer.start(profile="dimos", library="test")
        image_channels = [_channel(writer, f"view_{i}", Image, "jpeg") for i in range(camera_count)]
        state_channel = _channel(writer, "measured", JointState)
        action_channel = _channel(writer, "commanded", JointState)
        status_channel = _channel(writer, "status", EpisodeStatus)
        events: tuple[Literal["save", "discard"], ...] = ("save", "discard")
        for episode, event in enumerate(events):
            start_time = 20.0 + episode
            _add(
                writer,
                status_channel,
                start_time,
                EpisodeStatus(
                    ts=start_time,
                    state="recording",
                    last_event="start",
                    episodes_saved=episode,
                    episodes_discarded=0,
                    task_label="pick",
                ),
            )
            for frame in range(3):
                ts = start_time + frame / 30.0
                for image_channel in image_channels:
                    _add(
                        writer,
                        image_channel,
                        ts,
                        JpegCodec().encode(
                            Image(
                                ts=ts,
                                frame_id="wrist_camera_link",
                                format=ImageFormat.RGB,
                                data=np.full((64, 64, 3), frame, dtype=np.uint8),
                            )
                        ),
                    )
                _add(
                    writer,
                    state_channel,
                    ts,
                    JointState(
                        ts=ts,
                        frame_id="coordinator",
                        name=list(reversed(JOINTS)),
                        position=[float(frame + i) for i in reversed(range(len(JOINTS)))],
                        velocity=[0.0] * len(JOINTS),
                        effort=[0.0] * len(JOINTS),
                    ),
                )
                _add(
                    writer,
                    action_channel,
                    ts,
                    JointState(
                        ts=ts,
                        frame_id="coordinator",
                        name=list(reversed(JOINTS)),
                        position=[float(frame + i) + 0.5 for i in reversed(range(len(JOINTS)))],
                        velocity=[],
                        effort=[],
                    ),
                )
            end = start_time + 2 / 30.0
            _add(
                writer,
                status_channel,
                end,
                EpisodeStatus(
                    ts=end,
                    state="idle",
                    last_event=event,
                    episodes_saved=1,
                    episodes_discarded=0,
                    task_label="pick",
                ),
            )
        writer.finish()


@pytest.mark.parametrize("camera_count", [1, 2, 4])
def test_mcap_converts_to_lerobot_dataset(tmp_path: Path, camera_count: int) -> None:
    source = tmp_path / "session.mcap"
    destination = tmp_path / "dataset"
    _record(source, camera_count)
    profile = CollectionProfile(
        name="dual-test",
        robot_type="dual_openyam",
        observations={
            **{
                f"observation.images.view_{i}": CollectionFeature(
                    stream=f"view_{i}",
                    message_type=Image,
                    field="data",
                    dtype="video",
                    shape=(64, 64, 3),
                    names=["height", "width", "channels"],
                )
                for i in range(camera_count)
            },
            "observation.state": CollectionFeature(
                stream="measured",
                message_type=JointState,
                field="position",
                dtype="float32",
                shape=(len(JOINTS),),
                names=JOINTS,
            ),
            "observation.velocity": CollectionFeature(
                stream="measured",
                message_type=JointState,
                field="velocity",
                dtype="float32",
                shape=(len(JOINTS),),
                names=JOINTS,
            ),
        },
        actions={
            "action": CollectionFeature(
                stream="commanded",
                message_type=JointState,
                field="position",
                dtype="float32",
                shape=(len(JOINTS),),
                names=JOINTS,
            ),
        },
        sync=SyncConfig(anchor="observation.images.view_0", rate_hz=30, tolerance_ms=20),
    )
    directory = tmp_path / "session"
    directory.mkdir()
    source.rename(directory / "recording.mcap")
    (directory / "schema.json").write_text(profile.to_schema().model_dump_json())
    moved = tmp_path / "moved"
    directory.rename(moved)
    config = RecordingSchema.read(moved).dataprep_config(
        moved,
        OutputConfig(
            format="lerobot",
            path=destination,
            metadata={"repo_id": "local/openyam-mcap"},
        ),
    )
    # Exercise the exact JSON boundary used by the isolated converter.
    config = type(config).model_validate_json(config.model_dump_json())
    root = run_dataprep(config, writer=write)

    info = json.loads((root / "meta" / "info.json").read_text())
    assert info["total_episodes"] == 1
    assert info["total_frames"] == 3
    assert info["fps"] == 30
    assert info["features"]["action"]["names"] == JOINTS

    dataset = LeRobotDataset("local/openyam-mcap", root=root)
    frame = dataset[0]
    for i in range(camera_count):
        assert tuple(frame[f"observation.images.view_{i}"].shape) == (3, 64, 64)
    np.testing.assert_allclose(frame["observation.state"].numpy(), np.arange(len(JOINTS)))
    np.testing.assert_allclose(frame["action"].numpy(), np.arange(len(JOINTS)) + 0.5)
    np.testing.assert_allclose(frame["observation.velocity"].numpy(), np.zeros(len(JOINTS)))

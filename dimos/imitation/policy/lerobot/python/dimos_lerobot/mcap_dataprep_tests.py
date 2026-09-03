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
from typing import Any

from dimos_lerobot.dataprep import write
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from mcap.writer import Writer as McapWriter
import numpy as np

from dimos.imitation.dataprep.build import run_dataprep
from dimos.imitation.dataprep.core import (
    DataPrepConfig,
    FeatureSpec,
    OutputConfig,
    SyncConfig,
)
from dimos.memory.codecs.jpeg import JpegCodec
from dimos.msgs.imitation_msgs.EpisodeStatus import EpisodeStatus
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState

JOINTS = [f"arm/joint{index}" for index in range(1, 7)] + ["arm/gripper"]


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


def _record(path: Path) -> None:
    with path.open("wb") as output:
        writer = McapWriter(output)
        writer.start(profile="dimos", library="test")
        image_channel = _channel(writer, "color_image", Image, "jpeg")
        state_channel = _channel(writer, "coordinator_joint_state", JointState)
        action_channel = _channel(writer, "applied_joint_position_command", JointState)
        status_channel = _channel(writer, "status", EpisodeStatus)
        _add(
            writer,
            status_channel,
            20.0,
            EpisodeStatus(
                ts=20.0,
                state="recording",
                last_event="start",
                episodes_saved=0,
                episodes_discarded=0,
                task_label="pick",
            ),
        )
        for frame in range(3):
            ts = 20.0 + frame / 30.0
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
                    name=JOINTS,
                    position=[float(frame)] * len(JOINTS),
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
                    name=JOINTS,
                    position=[float(frame) + 0.5] * len(JOINTS),
                    velocity=[],
                    effort=[],
                ),
            )
        end = 20.0 + 2 / 30.0
        _add(
            writer,
            status_channel,
            end,
            EpisodeStatus(
                ts=end,
                state="idle",
                last_event="save",
                episodes_saved=1,
                episodes_discarded=0,
                task_label="pick",
            ),
        )
        writer.finish()


def test_mcap_converts_to_lerobot_dataset(tmp_path: Path) -> None:
    source = tmp_path / "session.mcap"
    destination = tmp_path / "dataset"
    _record(source)
    config = DataPrepConfig(
        source=str(source),
        observation={
            "observation.images.wrist": FeatureSpec(
                stream="color_image",
                field="data",
                dtype="video",
                shape=(64, 64, 3),
                names=["height", "width", "channels"],
            ),
            "observation.state": FeatureSpec(
                stream="coordinator_joint_state",
                field="position",
                dtype="float32",
                shape=(len(JOINTS),),
                names=JOINTS,
            ),
        },
        action={
            "action": FeatureSpec(
                stream="applied_joint_position_command",
                field="position",
                dtype="float32",
                shape=(len(JOINTS),),
                names=JOINTS,
            )
        },
        sync=SyncConfig(
            anchor="observation.images.wrist",
            rate_hz=30.0,
            tolerance_ms=20.0,
        ),
        output=OutputConfig(
            format="lerobot",
            path=destination,
            metadata={"repo_id": "local/openyam-mcap", "robot_type": "openyam"},
        ),
    )

    root = run_dataprep(config, writer=write)

    info = json.loads((root / "meta" / "info.json").read_text())
    assert info["total_episodes"] == 1
    assert info["total_frames"] == 3
    assert info["fps"] == 30
    assert info["features"]["action"]["names"] == JOINTS

    dataset = LeRobotDataset("local/openyam-mcap", root=root)
    frame = dataset[0]
    assert tuple(frame["observation.images.wrist"].shape) == (3, 64, 64)

# Copyright 2025-2026 Dimensional Inc.
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

"""Synthetic R1 capture in the same MCAP/LCM contract as the native recorder."""

from pathlib import Path
from typing import Any

from mcap.writer import Writer
import numpy as np

from dimos.imitation.collection.episode import EpisodeEvent, EpisodeStatus, RecordingState
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.String import String
from dimos.robot.galaxea.r1pro.collection import (
    CAMERAS,
    POSITION_JOINTS,
    collection_profile,
    collection_schema,
)
from dimos.robot.galaxea.r1pro.config import R1PRO_BODY_AXES


def write_collection(
    directory: Path, *, missing_camera: str | None = None, stale_commands: bool = False
) -> Path:
    directory.mkdir()
    profile = collection_profile()
    schema = collection_schema(profile)
    (directory / "schema.json").write_text(schema.model_dump_json())
    with (directory / schema.payload).open("wb") as output:
        writer = Writer(output)
        writer.start(profile="dimos")
        channels = {
            name: writer.register_channel(
                topic=name,
                message_encoding="lcm",
                schema_id=0,
                metadata={
                    "dimos.payload_type": f"{kind.__module__}.{kind.__qualname__}",
                    "dimos.observation_time": "publish_time",
                },
            )
            for name, kind in {**profile.input_types(), "status": String}.items()
        }

        def emit(name: str, ts: float, message: Any) -> None:
            writer.add_message(
                channel_id=channels[name],
                log_time=round(ts * 1e9),
                publish_time=round(ts * 1e9),
                data=message.lcm_encode(),
            )

        def event(
            ts: float, name: EpisodeEvent, state: RecordingState, saved: int, discarded: int
        ) -> None:
            emit(
                "status",
                ts,
                String(
                    EpisodeStatus(
                        ts=ts,
                        state=state,
                        last_event=name,
                        episodes_saved=saved,
                        episodes_discarded=discarded,
                        task_label="pick bottle",
                    ).to_json()
                ),
            )

        event(10, "start", "recording", 0, 0)
        for i in range(81):
            ts = 10 + i / 100
            # Reverse wire order to verify projection uses names, not array indices.
            emit(
                "coordinator_joint_state",
                ts,
                JointState(
                    ts=ts,
                    name=list(reversed(POSITION_JOINTS)),
                    position=list(reversed([j / 100 for j in range(20)])),
                ),
            )
            if not stale_commands or i < 10:
                emit(
                    "coordinator_joint_command",
                    ts,
                    JointState(ts=ts, name=list(POSITION_JOINTS), position=[0.5] * 20),
                )
                emit(
                    "coordinator_joint_velocity",
                    ts,
                    JointState(
                        ts=ts,
                        name=list(R1PRO_BODY_AXES),
                        velocity=[0.2 if i < 9 else 0.4, -0.1, 0.3],
                    ),
                )
            if i % 20 == 0:
                for camera, (stream, shape) in CAMERAS.items():
                    if camera != missing_camera:
                        image = Image(
                            ts=ts,
                            format=ImageFormat.RGB,
                            data=np.full(shape, 80 + i, dtype=np.uint8),
                        )
                        emit(stream, ts, CompressedImage.from_image(image))
        event(10.81, "save", "idle", 1, 0)
        event(11, "start", "recording", 1, 0)
        event(11.1, "discard", "idle", 1, 1)
        event(12, "start", "recording", 1, 1)
        writer.finish()  # type: ignore[no-untyped-call]
    return directory

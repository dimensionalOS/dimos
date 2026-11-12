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

from collections.abc import Iterator
from pathlib import Path

import pytest

from dimos.memory.store.base import Store
from dimos.memory.store.mcap import McapStore
from dimos.memory.store.memory import MemoryStore
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


@pytest.fixture(params=["memory", "sqlite", "mcap"])
def recording(request: pytest.FixtureRequest, tmp_path: Path) -> Iterator[Store]:
    streams: dict[str, list[float]] = {
        "empty": [],
        "odom": [10.02, 10.00, 10.01],
        "camera": [10.025, 10.006, 10.014],
    }
    store: Store
    if request.param == "mcap":
        mcap_writer = pytest.importorskip("mcap.writer")
        path = tmp_path / "recording.mcap"
        with path.open("wb") as output:
            writer = mcap_writer.Writer(output)
            writer.start(profile="dimos")
            for name, timestamps in streams.items():
                channel = writer.register_channel(
                    topic=name,
                    message_encoding="lcm",
                    schema_id=0,
                    metadata={
                        "dimos.payload_type": "dimos.msgs.geometry_msgs.PoseStamped.PoseStamped",
                        "dimos.observation_time": "publish_time",
                    },
                )
                for i, ts in enumerate(timestamps):
                    writer.add_message(
                        channel_id=channel,
                        log_time=(20 + i) * 1_000_000_000,
                        publish_time=round(ts * 1_000_000_000),
                        data=PoseStamped(ts=ts, frame_id=str(i)).lcm_encode(),
                    )
            writer.finish()
        with McapStore(path=str(path)) as store:
            yield store
    else:
        store = (
            SqliteStore(path=str(tmp_path / "recording.db"))
            if request.param == "sqlite"
            else MemoryStore()
        )
        with store:
            for name, timestamps in streams.items():
                stream = store.stream(name, PoseStamped)
                for i, ts in enumerate(timestamps):
                    stream.append(PoseStamped(ts=ts, frame_id=str(i)), ts=ts)
            yield store


def test_first_ts_uses_earliest_source_timestamp(recording: Store) -> None:
    assert recording.replay().first_ts() == 10.0


def test_seek_uses_earliest_source_timestamp(recording: Store) -> None:
    replay = recording.replay(seek=0.005)

    assert [(ts, pose.frame_id) for ts, pose in replay.stream("odom").iterate_ts()] == [
        (10.01, "2"),
        (10.02, "0"),
    ]


def test_seek_uses_shared_recording_start(recording: Store) -> None:
    replay = recording.replay(seek=0.005)

    assert [ts for ts, _ in replay.stream("camera").iterate_ts()] == [10.006, 10.014, 10.025]


def test_duration_uses_earliest_source_timestamp(recording: Store) -> None:
    replay = recording.replay(duration=0.015)

    assert [ts for ts, _ in replay.stream("odom").iterate_ts()] == [10.0, 10.01]


def test_pin_anchor_uses_earliest_source_timestamp(recording: Store) -> None:
    replay = recording.replay()

    replay.pin_anchor()

    assert replay._anchor is not None
    assert replay._anchor[1] == 10.0

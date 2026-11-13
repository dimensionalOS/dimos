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

from pathlib import Path
import threading

from mcap.writer import Writer
import pytest

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.robot.unitree.g1.g1_replay import G1Replay


@pytest.mark.parametrize("suffix", [".db", ".mcap"])
def test_replays_odometry_from_sqlite_and_mcap(tmp_path: Path, suffix: str) -> None:
    recorded = Odometry(ts=12.5, frame_id="odom")
    path = tmp_path / f"recording{suffix}"
    if suffix == ".mcap":
        with path.open("wb") as output:
            writer = Writer(output)
            writer.start(profile="dimos")
            channel = writer.register_channel(
                topic="pointlio_odometry",
                message_encoding="lcm",
                schema_id=0,
                metadata={
                    "dimos.payload_type": "dimos.msgs.nav_msgs.Odometry.Odometry",
                    "dimos.observation_time": "publish_time",
                },
            )
            writer.add_message(
                channel_id=channel,
                log_time=13_000_000_000,
                publish_time=12_500_000_000,
                data=recorded.lcm_encode(),
            )
            writer.finish()
    else:
        with SqliteStore(path=str(path)) as store:
            store.stream("pointlio_odometry", Odometry).append(recorded, ts=recorded.ts)

    replay = G1Replay(dataset=str(path))
    received: list[Odometry] = []
    published = threading.Event()

    def on_odometry(message: Odometry) -> None:
        received.append(message)
        published.set()

    replay.pointlio_odometry.subscribe(on_odometry)
    try:
        replay.start()
        assert published.wait(5.0)
    finally:
        replay.stop()
    assert [message.lcm_encode() for message in received] == [recorded.lcm_encode()]

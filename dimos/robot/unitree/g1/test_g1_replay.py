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

_TIMEOUT = 5.0  # seconds a test waits on the replay before failing


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
        replay.on_system_modules_ready([])
        assert published.wait(5.0)
    finally:
        replay.stop()
    assert [message.lcm_encode() for message in received] == [recorded.lcm_encode()]


def test_replay_logs_source_duration_and_completion(tmp_path, mocker):
    path = tmp_path / "g1.db"
    with SqliteStore(path=str(path)) as store:
        odom = store.stream("pointlio_odometry", Odometry)
        for ts in (1.0, 2.0):
            odom.append(Odometry(ts=ts), ts=ts)
    log = mocker.patch("dimos.memory.replay_progress.logger")
    finished = threading.Event()
    log.info.side_effect = lambda message, **_fields: (
        finished.set() if message == "Replay finished" else None
    )
    with G1Replay(dataset=str(path)) as replay:
        log.info.assert_called_once_with(
            "Replay started",
            source=str(path),
            path=str(path),
            duration_s=1.0,
            speed=1.0,
            loop=False,
        )
        replay.on_system_modules_ready([])
        assert finished.wait(_TIMEOUT)
        assert log.info.call_args_list[1:] == [mocker.call("Replay finished", source=str(path))]

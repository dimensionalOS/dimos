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

from contextlib import closing
from pathlib import Path
import sqlite3
import threading

from mcap.writer import Writer
import pytest
from reactivex.testing import TestScheduler

from dimos.core.coordination.blueprints import autoconnect
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.get_all_blueprints import get_blueprint_by_name
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


def test_replay_logs_source_duration_and_completion(tmp_path, mocker):
    path = tmp_path / "g1.db"
    with SqliteStore(path=str(path)) as store:
        odom = store.stream("pointlio_odometry", Odometry)
        for ts in (1.0, 2.0):
            odom.append(Odometry(ts=ts), ts=ts)
    scheduler = TestScheduler()
    mocker.patch("dimos.memory.replay.TimeoutScheduler", return_value=scheduler)
    log = mocker.patch("dimos.memory.replay_progress.logger")
    with G1Replay(dataset=str(path)):
        log.info.assert_called_once_with(
            "Replay started",
            source=str(path),
            path=str(path),
            duration_s=1.0,
            speed=1.0,
            loop=False,
        )
        scheduler.advance_by(5)
        assert log.info.call_args_list[1:] == [mocker.call("Replay finished", source=str(path))]


@pytest.mark.parametrize(
    ("name", "error"),
    [("pointlio_lidar", ValueError), ("pointlio_odometry", TypeError)],
)
def test_rejects_unusable_robot_output(tmp_path, name, error):
    path = tmp_path / "input.db"
    with SqliteStore(path=path) as store:
        store.stream(name, int).append(1, ts=1.0)
        if name == "pointlio_odometry":
            store.stream("tf", TFMessage).append(TFMessage(), ts=1.0)
    module = G1Replay(dataset=str(path))
    try:
        with pytest.raises(error, match=name):
            module.start()
        assert module.rpc is not None  # The coordinator still needs the startup error reply.
        assert module._loop_thread is None
    finally:
        module.stop()


def test_failed_camera_decode_closes_started_replay(tmp_path, mocker):
    path = tmp_path / "broken-camera.db"
    with SqliteStore(path=path) as store:
        store.stream("realsense_camera_info", CameraInfo).append(CameraInfo(), ts=1.0)
    with closing(sqlite3.connect(path)) as connection, connection:
        connection.execute("UPDATE realsense_camera_info_blob SET data = X'00'")

    module = G1Replay(dataset=str(path))
    stop_store = mocker.spy(SqliteStore, "stop")
    loop_thread = module._loop_thread
    assert loop_thread is not None
    try:
        with pytest.raises(ValueError, match="Decode error"):
            module.start()
        assert not loop_thread.is_alive()
        stop_store.assert_called_once()
    finally:
        module.stop()


def test_mesh_replay_warns_about_missing_inputs_and_delivers_odometry(tmp_path, mocker):
    path = tmp_path / "odom.db"
    with SqliteStore(path=path) as store:
        store.stream("pointlio_odometry", Odometry).append(Odometry(ts=1.0), ts=1.0)
    blueprint = autoconnect(get_blueprint_by_name("unitree-g1-replay-mesh")).solidify()
    source = next(atom for atom in blueprint.active_blueprints if atom.module is G1Replay)
    replay = G1Replay(dataset=str(path), **source.kwargs)
    seen = threading.Event()
    warning = mocker.patch("dimos.robot.unitree.g1.g1_replay.logger.warning")
    replay.pointlio_odometry.subscribe(lambda _: seen.set())
    with replay:
        assert seen.wait(5)
        warning.assert_called_once()
        assert set(warning.call_args.kwargs["missing"]) == {
            "color_image",
            "realsense_camera_info",
            "realsense_depth_image",
            "tf",
        }


def test_empty_recorded_outputs_differ_from_an_empty_playback_window(tmp_path: Path) -> None:
    path = tmp_path / "recording.db"
    with SqliteStore(path=path) as store:
        store.stream("pointlio_odometry", Odometry)
        store.stream("tf", TFMessage)
        store.stream("temperature", int).append(20, ts=0.0)

    replay = G1Replay(dataset=str(path))
    try:
        with pytest.raises(ValueError, match="pointlio_odometry"):
            replay.start()
    finally:
        replay.stop()

    with SqliteStore(path=path) as store:
        store.stream("pointlio_odometry").append(Odometry(ts=0.0), ts=0.0)
    with G1Replay(dataset=str(path), seek=10.0):
        pass

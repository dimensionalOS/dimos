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

from contextlib import ExitStack
from functools import partial
from types import SimpleNamespace

import numpy as np
import pytest
from reactivex import interval
from reactivex.disposable import Disposable
from reactivex.testing import TestScheduler

from dimos.experimental.robot.bosdyn.spot.replay import SpotReplay
from dimos.mapping.relocalization.blueprints import RecordingPlayer
from dimos.memory import replay_progress
from dimos.memory.replay import ReplayStream
from dimos.memory.replay_module import ReplayModule
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.perception.localize.demo_blueprints.xarm_feed import DepthCameraFeed
from dimos.robot.unitree.g1.g1_replay import G1Replay


@pytest.mark.parametrize("player", [SpotReplay, DepthCameraFeed, G1Replay])
def test_robot_players_keep_tf_before_other_sensors(tmp_path, mocker, player):
    path = tmp_path / "sensors.db"
    with SqliteStore(path=str(path)) as store:
        for ts in (10.0, 10.75):
            store.stream("tf", TFMessage).append(TFMessage(Transform(ts=ts)), ts=ts)
        for name in ("odometry", "pointlio_odometry"):
            store.stream(name, Odometry).append(Odometry(ts=11.0), ts=11.0)
        store.stream("color_image", Image).append(
            Image.from_numpy(np.zeros((2, 2, 3), dtype=np.uint8)), ts=11.0
        )
        store.stream("depth_image", Image)
        store.stream("camera_info", CameraInfo)

    scheduler = TestScheduler()
    mocker.patch("dimos.memory.replay.time", SimpleNamespace(time=lambda: scheduler.clock))
    mocker.patch("dimos.memory.replay.TimeoutScheduler", return_value=scheduler)
    observable = ReplayStream.observable

    def open_stream(stream, **kwargs):
        if stream.name == "tf":
            scheduler.sleep(0.2)
        return observable(stream, **kwargs)

    mocker.patch.object(ReplayStream, "observable", open_stream)
    settings = (
        {"db_path": str(path), "loop": False} if player is SpotReplay else {"dataset": str(path)}
    )
    with ExitStack() as cleanup:
        module = player(**settings)
        cleanup.callback(module.stop)
        received = []
        cleanup.enter_context(Disposable(module.tf.subscribe(received.append)))
        module.start()
        scheduler.advance_to(2)
    assert [batch.transforms[0].ts for batch in received] == [10.0, 10.75]


@pytest.mark.parametrize("player", [RecordingPlayer, ReplayModule])
def test_player_starts_clock_after_opening_both_sensor_cursors(tmp_path, mocker, player):
    path = tmp_path / "sensors.db"
    tf_name = "tf" if player is RecordingPlayer else "transforms"
    with SqliteStore(path=str(path)) as store:
        for ts in (5.1, 5.2, 5.4):
            cloud = PointCloud2.from_numpy(np.zeros((1, 3)), timestamp=ts)
            store.stream("pointlio_lidar", PointCloud2).append(cloud, ts=ts)
        for ts in (5.0, 5.15, 5.6):
            store.stream(tf_name, TFMessage).append(TFMessage(Transform(ts=ts)), ts=ts)

    scheduler = TestScheduler()
    mocker.patch("dimos.memory.replay.time", SimpleNamespace(time=lambda: scheduler.clock))
    mocker.patch("dimos.memory.replay.TimeoutScheduler", return_value=scheduler)
    base_stream = ReplayStream._base_stream
    events = []

    def open_cursor(stream):
        if stream.name == tf_name:
            scheduler.advance_by(0.3)
        return base_stream(stream)

    mocker.patch.object(ReplayStream, "_base_stream", open_cursor)
    with ExitStack() as cleanup:
        module = player(dataset=str(path))
        lidar = module.lidar if player is RecordingPlayer else module.outputs["pointlio_lidar"]
        transforms = module.tf if player is RecordingPlayer else module.outputs[tf_name]
        cleanup.callback(module.stop)
        cleanup.enter_context(
            Disposable(lidar.subscribe(lambda m: events.append((scheduler.clock, m.ts))))
        )
        cleanup.enter_context(
            Disposable(
                transforms.subscribe(lambda m: events.append((scheduler.clock, m.transforms[0].ts)))
            )
        )
        module.start()
        start = scheduler.clock
        assert events == []
        scheduler.advance_by(1.0)
    assert [stamp for _, stamp in events] == pytest.approx([5.0, 5.1, 5.15, 5.2, 5.4, 5.6])
    assert [wall - start for wall, _ in events] == pytest.approx([0, 0.1, 0.15, 0.2, 0.4, 0.6])


@pytest.fixture
def calibrated_recording(tmp_path):
    """Mount frames and intrinsics keep one old stamp; the sensors start 990 s later."""
    path = tmp_path / "calibrated.db"
    with SqliteStore(path=str(path)) as store:
        frames = store.stream("tf", TFMessage)
        for _ in range(2):
            frames.append(
                TFMessage(Transform(frame_id="base", child_frame_id="camera", ts=10.0)), ts=10.0
            )
        for ts in (1000.0, 1000.5):
            frames.append(
                TFMessage(Transform(frame_id="world", child_frame_id="base", ts=ts)), ts=ts
            )
        # An edge or stream sampled once is data, not calibration: it plays at its own time.
        frames.append(
            TFMessage(Transform(frame_id="base", child_frame_id="tool", ts=1000.25)), ts=1000.25
        )
        # Except on the static channel of a tf stream, where ROS 2 latches a mount once.
        store.stream("tf_static", TFMessage).append(
            TFMessage(Transform(frame_id="base", child_frame_id="lidar", ts=10.0)), ts=10.0
        )
        for name in ("grayscale_info", "camera_info", "realsense_camera_info"):
            for _ in range(2):
                store.stream(name, CameraInfo).append(CameraInfo(ts=10.0), ts=10.0)
        for ts in (1000.0, 1000.25, 1000.5):
            for name in ("odometry", "pointlio_odometry"):
                store.stream(name, Odometry).append(Odometry(ts=ts), ts=ts)
            image = Image.from_numpy(np.zeros((2, 2, 3), dtype=np.uint8), ts=ts)
            store.stream("color_image", Image).append(image, ts=ts)
            cloud = PointCloud2.from_numpy(np.zeros((1, 3)), timestamp=ts)
            store.stream("pointlio_lidar", PointCloud2).append(cloud, ts=ts)
        store.stream("depth_image", Image).append(image, ts=1000.5)
    return path


@pytest.fixture
def scheduler(mocker):
    scheduler = TestScheduler()
    mocker.patch("dimos.memory.replay.time", SimpleNamespace(time=lambda: scheduler.clock))
    mocker.patch("dimos.memory.replay.TimeoutScheduler", return_value=scheduler)
    return scheduler


@pytest.fixture
def play(calibrated_recording, scheduler):
    """Run a player for some scheduler seconds; gives (wall offset, stamp) per port."""
    with ExitStack() as cleanup:

        def run(player, seconds, *ports, **settings):
            dataset = "db_path" if player is SpotReplay else "dataset"
            module = player(**{dataset: str(calibrated_recording)}, **settings)
            cleanup.callback(module.stop)
            received = {name: [] for name in ("tf", *ports)}

            def record(name, message):
                if isinstance(message, TFMessage):
                    received[name].extend(
                        (scheduler.clock, t.child_frame_id, t.ts) for t in message.transforms
                    )
                else:
                    received[name].append((scheduler.clock, message.ts))

            for name in received:
                cleanup.enter_context(
                    Disposable(getattr(module, name).subscribe(partial(record, name)))
                )
            module.start()
            scheduler.advance_by(seconds)
            return received

        yield run


@pytest.mark.parametrize(
    ("player", "sensor", "settings"),
    [
        (SpotReplay, "odometry", {"loop": False}),
        (DepthCameraFeed, "color_image", {}),
        (G1Replay, "pointlio_odometry", {}),
        (RecordingPlayer, "lidar", {}),
        (ReplayModule, "odometry", {}),
    ],
)
def test_static_calibration_does_not_delay_sensor_playback(play, mocker, player, sensor, settings):
    log = mocker.patch("dimos.memory.replay_progress.logger")
    received = play(player, 1, sensor, **settings)
    assert received[sensor] == [(0, 1000.0), (0.25, 1000.25), (0.5, 1000.5)]
    # The mount frame is published once, ahead of the edges that move.
    assert received["tf"] == [
        (0, "camera", 10.0),
        (0, "base", 1000.0),
        (0.25, "tool", 1000.25),
        (0.5, "base", 1000.5),
    ]
    assert [call.args[0] for call in log.info.call_args_list] == [
        "Replay started",
        "Replay finished",
    ]
    assert log.info.call_args_list[0].kwargs["duration_s"] == 0.5


def test_only_the_static_tf_channel_makes_one_row_calibration(play):
    received = play(ReplayModule, 1, "odometry", "depth_image", "tf_static")
    assert received["tf_static"] == [(0, "lidar", 10.0)]
    assert received["odometry"][0] == (0, 1000.0)
    # Elsewhere one row is a sample: an event stream and an ordinary tf edge keep their time.
    assert received["depth_image"] == [(0.5, 1000.5)]
    assert (0.25, "tool", 1000.25) in received["tf"]


def test_looped_spot_replay_repeats_without_the_static_lead(play):
    received = play(SpotReplay, 0.6, "odometry")
    assert received["odometry"] == [(0, 1000.0), (0.25, 1000.25), (0.5, 1000.5), (0.55, 1000.0)]
    assert received["tf"] == [
        (0, "camera", 10.0),
        (0, "base", 1000.0),
        (0.25, "tool", 1000.25),
        (0.5, "base", 1000.5),
        (0.55, "base", 1000.0),
    ]


def test_clip_is_measured_from_the_first_sensor_row(play, scheduler, mocker):
    mocker.patch.object(
        replay_progress, "interval", side_effect=lambda period: interval(period, scheduler)
    )
    received = play(
        DepthCameraFeed, 2, "color_image", "depth_image", "camera_info", seek=0.25, duration=0.5
    )
    assert received["color_image"] == [(0, 1000.25), (0.25, 1000.5)]
    assert received["depth_image"] == [(0.25, 1000.5)]
    # Calibration is outside the clip and still reaches every consumer, once a second.
    assert received["camera_info"] == [(1, 10.0), (2, 10.0)]
    assert received["tf"] == [
        (0, "camera", 10.0),
        (0, "tool", 1000.25),
        (0.25, "base", 1000.5),
        (1, "camera", 10.0),
        (2, "camera", 10.0),
    ]

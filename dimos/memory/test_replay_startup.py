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
from types import SimpleNamespace

import numpy as np
import pytest
from reactivex.disposable import Disposable
from reactivex.testing import TestScheduler

from dimos.experimental.robot.bosdyn.spot.replay import SpotReplay
from dimos.mapping.relocalization.blueprints import RecordingPlayer
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


@pytest.mark.parametrize("player", [SpotReplay, RecordingPlayer, DepthCameraFeed, G1Replay])
def test_robot_players_keep_tf_before_other_sensors(tmp_path, mocker, player):
    path = tmp_path / "sensors.db"
    with SqliteStore(path=str(path)) as store:
        for ts in (10.0, 10.75):
            store.stream("tf", TFMessage).append(TFMessage(Transform(ts=ts)), ts=ts)
        for name in ("odometry", "pointlio_odometry"):
            store.stream(name, Odometry).append(Odometry(ts=11.0), ts=11.0)
        store.stream("pointlio_lidar", PointCloud2).append(
            PointCloud2.from_numpy(np.zeros((1, 3)), timestamp=11.0), ts=11.0
        )
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

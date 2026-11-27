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

import numpy as np
import pytest
from reactivex.testing import TestScheduler

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.perception.localize.demo_blueprints.xarm_feed import DepthCameraFeed


def test_feed_logs_source_duration_and_completion(tmp_path, mocker):
    path = tmp_path / "xarm.db"
    with SqliteStore(path=str(path)) as store:
        for ts in (1.0, 2.0):
            for name in ("color_image", "depth_image"):
                store.stream(name, Image).append(
                    Image.from_numpy(np.zeros((2, 2, 3), dtype=np.uint8)), ts=ts
                )
            store.stream("camera_info", CameraInfo).append(CameraInfo(), ts=ts)
            store.stream("tf", TFMessage).append(TFMessage(), ts=ts)
    scheduler = TestScheduler()
    mocker.patch("dimos.memory.replay.TimeoutScheduler", return_value=scheduler)
    log = mocker.patch("dimos.memory.replay_progress.logger")
    with DepthCameraFeed(dataset=str(path)):
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
    ("color_type", "error"),
    [(CameraInfo, TypeError), (Image, ValueError)],
    ids=["wrong-type", "no-frames"],
)
def test_feed_rejects_a_recording_it_cannot_play(tmp_path, color_type, error):
    path = tmp_path / "xarm.db"
    with SqliteStore(path=str(path)) as store:
        color = store.stream("color_image", color_type)
        if color_type is CameraInfo:
            color.append(CameraInfo(), ts=1.0)
        store.stream("depth_image", Image)
        store.stream("camera_info", CameraInfo)
        store.stream("tf", TFMessage)
    module = DepthCameraFeed(dataset=str(path))
    try:
        with pytest.raises(error, match="color_image"):
            module.start()
        assert module._loop_thread is None
    finally:
        module.stop()

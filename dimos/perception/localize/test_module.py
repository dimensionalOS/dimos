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

import threading

import numpy as np
import pytest
from reactivex.disposable import Disposable
from reactivex.scheduler import ThreadPoolScheduler

from dimos.memory.store.memory import MemoryStore
from dimos.models.embedding.base import Embedding
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.perception.localize.memento import Memento
from dimos.perception.localize.module import PerceptionLocalize
from dimos.utils.testing.waiting import wait_until


@pytest.fixture
def localizer(mocker):
    module = PerceptionLocalize()
    for name in ("tf", "color_image", "depth_image", "lidar", "camera_info"):
        mocker.patch.object(
            getattr(module, name), "subscribe", return_value=mocker.Mock(name=f"unsubscribe_{name}")
        )
    mocker.patch("dimos.perception.localize.module.DanDetector", autospec=True)
    yield module
    module.stop()


def test_stop_without_start_disposes_registered_resources(localizer, mocker):
    cleanup = mocker.Mock()
    localizer.register_disposable(Disposable(cleanup))

    localizer.stop()

    cleanup.assert_called_once_with()


@pytest.mark.parametrize("geometry", ["depth", "lidar"])
def test_stop_after_subscription_failure_releases_earlier_resources(localizer, mocker, geometry):
    localizer.config.geometry = geometry
    close = mocker.spy(Memento, "close")
    stop_memory = mocker.spy(MemoryStore, "stop")
    localizer.camera_info.subscribe.side_effect = RuntimeError("subscription failed")

    with pytest.raises(RuntimeError, match="subscription failed"):
        localizer.start()
    localizer.stop()
    localizer.stop()

    localizer.tf.subscribe.return_value.assert_called_once_with()
    localizer.color_image.subscribe.return_value.assert_called_once_with()
    geometry_stream = localizer.depth_image if geometry == "depth" else localizer.lidar
    geometry_stream.subscribe.return_value.assert_called_once_with()
    close.assert_called_once_with(localizer._memento)
    stop_memory.assert_called_once_with(localizer._memory)


def test_stop_after_memory_setup_failure_closes_database(localizer, mocker):
    close = mocker.spy(Memento, "close")
    mocker.patch.object(MemoryStore, "stream", side_effect=RuntimeError("memory setup failed"))

    with pytest.raises(RuntimeError, match="memory setup failed"):
        localizer.start()
    localizer.stop()

    close.assert_called_once_with(localizer._memento)


def test_stop_after_thread_start_failure_releases_resources(localizer, mocker):
    close = mocker.spy(Memento, "close")
    start_thread = mocker.patch.object(
        threading.Thread, "start", side_effect=RuntimeError("thread start failed")
    )

    with pytest.raises(RuntimeError, match="thread start failed"):
        localizer.start()
    mocker.stop(start_thread)
    localizer.stop()

    localizer.camera_info.subscribe.return_value.assert_called_once_with()
    close.assert_called_once_with(localizer._memento)


def test_stop_after_start_releases_resources_once(localizer, mocker):
    close = mocker.spy(Memento, "close")

    localizer.start()
    localizer.stop()
    localizer.stop()

    localizer.tf.subscribe.return_value.assert_called_once_with()
    localizer.color_image.subscribe.return_value.assert_called_once_with()
    localizer.depth_image.subscribe.return_value.assert_called_once_with()
    localizer.camera_info.subscribe.return_value.assert_called_once_with()
    close.assert_called_once_with(localizer._memento)
    assert not localizer._thread.is_alive()


@pytest.fixture
def warmed_localizer(mocker, monkeypatch):
    pytest.importorskip("turbojpeg")
    scheduler = ThreadPoolScheduler(max_workers=2)
    monkeypatch.setattr("dimos.utils.threadpool.scheduler", scheduler)
    module = PerceptionLocalize()
    callbacks = {}

    def make_subscribe(name):
        unsubscribe = mocker.Mock(name=f"unsubscribe_{name}")

        def subscribe(cb):
            callbacks[name] = cb
            return unsubscribe

        return subscribe

    for name in ("tf", "color_image", "depth_image", "lidar", "camera_info"):
        mocker.patch.object(getattr(module, name), "subscribe", side_effect=make_subscribe(name))
    detector_cls = mocker.patch("dimos.perception.localize.module.DanDetector", autospec=True)
    detector_cls.return_value.localize.return_value = [[]]
    module.start()
    try:
        callbacks["camera_info"](CameraInfo(frame_id="cam", height=8, width=8))
        wait_until(
            lambda: module._stage == "waiting for the first posed frame of the feed",
            timeout=5.0,
            interval=0.01,
        )
        yield module, callbacks
    finally:
        module.stop()
        scheduler.executor.shutdown(wait=True)


def test_frame_embeddings_publishes_every_indexed_frame_in_order(warmed_localizer):
    module, _ = warmed_localizer
    received = []
    unsubscribe = module.frame_embeddings.subscribe(received.append)
    image = Image.from_numpy(
        np.zeros((8, 8, 3), dtype=np.uint8), format=ImageFormat.RGB, frame_id="cam", ts=1000.0
    )
    try:
        for i in range(20):
            vec = np.array([float(i), float(i + 1), -0.5], dtype=np.float32)
            module.index.append(
                image,
                ts=1000.0 + i,
                tags={"camera": "cam", "sharpness": 10.0 + 0.5 * i},
                embedding=Embedding(vec),
            )
        wait_until(lambda: len(received) == 20, timeout=5.0, interval=0.01)
    finally:
        unsubscribe()

    assert [msg.ts for msg in received] == [1000.0 + i for i in range(20)]
    for i, msg in enumerate(received):
        assert msg.frame_id == "cam"
        assert msg.sharpness == 10.0 + 0.5 * i
        assert msg.embedder == module.config.embedder
        assert msg.embed_hz == module.rig.embed_hz
        assert np.array_equal(
            msg.embedding, np.array([float(i), float(i + 1), -0.5], dtype=np.float32)
        )


def test_window_at_start_retains_first_frame_pose_and_depth_past_ten_minutes(warmed_localizer):
    module, callbacks = warmed_localizer
    first_ts = 1000.0
    color = Image.from_numpy(
        np.zeros((8, 8, 3), dtype=np.uint8), format=ImageFormat.RGB, frame_id="cam", ts=first_ts
    )
    depth = Image.from_numpy(
        np.full((8, 8), 500, dtype=np.uint16),
        format=ImageFormat.DEPTH16,
        frame_id="cam",
        ts=first_ts,
    )
    embedding = Embedding(np.array([1.0, 0.0], dtype=np.float32))

    callbacks["tf"](TFMessage(Transform(frame_id="world", child_frame_id="cam", ts=first_ts)))
    callbacks["depth_image"](depth)
    module.index.append(
        color, ts=first_ts, tags={"camera": "cam", "sharpness": 1.0}, embedding=embedding
    )
    wait_until(
        lambda: module._ready.is_set() and module.rig.depth_at(first_ts, "cam") is not None,
        timeout=5.0,
        interval=0.01,
    )

    for i in range(1, 701):
        ts = first_ts + i
        callbacks["tf"](TFMessage(Transform(frame_id="world", child_frame_id="cam", ts=ts)))
        module.index.append(
            color, ts=ts, tags={"camera": "cam", "sharpness": 1.0}, embedding=embedding
        )

    answer = module.localize("mug", 0.0, 1.0)
    assert answer.splitlines()[0] == "window 0.0s to 1.0s of 700.0s of feed, 2 frames"
    assert module.rig.world_to_optical(first_ts, "cam") is not None
    assert module.rig.depth_at(first_ts, "cam") is not None

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
from typing import Any

import numpy as np
import pytest
from reactivex.disposable import Disposable

from dimos.memory.store.memory import MemoryStore
from dimos.memory.store.sqlite import SqliteStore
from dimos.models.embedding.base import Embedding
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.perception.localize import dandetect
from dimos.perception.localize.dandetect import NoSegmenter, Option
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


class _SameEmbedder:
    def embed(self, *images: Image) -> Embedding | list[Embedding]:
        vecs = [Embedding(np.array([1.0, 0.0], dtype=np.float32)) for _ in images]
        return vecs[0] if len(vecs) == 1 else vecs

    def embed_text(self, *texts: str) -> Embedding | list[Embedding]:
        vecs = [Embedding(np.array([1.0, 0.0], dtype=np.float32)) for _ in texts]
        return vecs[0] if len(vecs) == 1 else vecs

    def stop(self) -> None:
        pass


class _BoxDetector:
    """Finds the box in every frame it is shown, and remembers when each frame was taken."""

    def __init__(self) -> None:
        self.seen: list[float] = []

    def query_score_rows(
        self, image: Image, queries: list[str], threshold: float
    ) -> tuple[np.ndarray, np.ndarray]:
        self.seen.append(image.ts)
        boxes = np.array([[10.0, 10.0, 30.0, 30.0]], dtype=np.float32)
        return boxes, np.full((1, len(queries)), 0.6, dtype=np.float32)

    def stop(self) -> None:
        pass


def test_localize_answers_from_a_recording_window(mocker, monkeypatch, tmp_path):
    # six frames, 1 s apart, of a box 0.45 m from a camera sliding 2 cm per frame
    path = tmp_path / "recording.db"
    with SqliteStore(path=str(path)) as store:
        info = CameraInfo(
            frame_id="cam", height=40, width=40, K=[40.0, 0.0, 20.0, 0.0, 40.0, 20.0, 0.0, 0.0, 1.0]
        )
        store.stream("camera_info", CameraInfo).append(info, ts=1.0)
        color = store.stream("color_image", Image)
        depth = store.stream("depth_image", Image, codec="lz4+lcm")
        tf = store.stream("tf", TFMessage)
        for k in range(6):
            ts = 1.0 + k
            rgb = np.full((40, 40, 3), 120 + k, dtype=np.uint8)
            rgb[10:30, 10:30] = 220
            color.append(
                Image.from_numpy(rgb, format=ImageFormat.RGB, frame_id="cam", ts=ts), ts=ts
            )
            depth_mm = np.full((40, 40), 500, dtype=np.uint16)
            depth_mm[10:30, 10:30] = 450
            depth.append(
                Image.from_numpy(depth_mm, format=ImageFormat.DEPTH16, frame_id="cam", ts=ts), ts=ts
            )
            pose = Transform(
                translation=Vector3(0.02 * k, 0.0, 0.0),
                frame_id="world",
                child_frame_id="cam",
                ts=ts,
            )
            tf.append(TFMessage(pose), ts=ts)

    detector = _BoxDetector()

    def fake_build(option: Option) -> Any:
        if option is dandetect.EMBEDDERS["siglip"]:
            return _SameEmbedder()
        if option is dandetect.SEGMENTERS["none"]:
            return NoSegmenter()
        return detector

    monkeypatch.setattr(dandetect, "_build", fake_build)
    module = PerceptionLocalize(segmenter="none")
    for name in ("tf", "color_image", "depth_image", "lidar", "camera_info"):
        mocker.patch.object(getattr(module, name), "subscribe", return_value=mocker.Mock())
    module.start()
    try:
        wait_until(module._loaded.is_set, timeout=10.0)
        answer = module.localize("box", start=-2.0, duration=2.0, recording=str(path))
    finally:
        module.stop()

    lines = answer.splitlines()
    assert lines[0] == "window 3.0s to 5.0s of 5.0s of feed, 3 frames"
    assert lines[1].startswith("'box' at (")
    assert detector.seen and min(detector.seen) >= 4.0

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

from dimos.memory.store.memory import MemoryStore
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.perception.localize.localize import _finalize
from dimos.perception.localize.memento import Memento, Sighting
from dimos.perception.localize.module import PerceptionLocalize, footprint
from dimos.perception.localize.types import LocalizeParams


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


def _sighting(ts: float, score: float, at: tuple[float, float, float], view: float) -> Sighting:
    return Sighting(
        ts=ts,
        camera="camera",
        score=score,
        points=np.asarray(at) + np.zeros((5, 3)),
        viewpoint=Vector3(view, 0.0, 1.0),
    )


def test_a_query_with_no_answer_reports_its_best_rejected_group() -> None:
    sightings = [
        _sighting(10.0, 0.21, (0.5, 0.1, 0.2), 0.0),
        _sighting(12.0, 0.18, (0.5, 0.1, 0.2), 0.1),
        _sighting(11.0, 0.15, (-0.5, 0.0, 0.2), 0.0),
    ]

    instances, near_miss = _finalize("mug", sightings, LocalizeParams(), "world", None)

    assert instances == []
    assert near_miss is not None
    score, views, position = near_miss
    assert score == pytest.approx(0.21)
    assert views == 2
    assert position == pytest.approx((0.5, 0.1, 0.2))


def _box_cloud(
    length: float, width: float, yaw_deg: float, centre: tuple[float, float]
) -> np.ndarray:
    """A dense top face, z 0.95, over a short side wall down to 0.91."""
    u, v = np.meshgrid(
        np.linspace(-length / 2, length / 2, 60), np.linspace(-width / 2, width / 2, 30)
    )
    yaw = np.radians(yaw_deg)
    x = centre[0] + u * np.cos(yaw) - v * np.sin(yaw)
    y = centre[1] + u * np.sin(yaw) + v * np.cos(yaw)
    top = np.stack([x.ravel(), y.ravel(), np.full(x.size, 0.95)], axis=1)
    wall = top[: len(top) // 3].copy()
    wall[:, 2] = np.linspace(0.91, 0.95, len(wall))
    return np.concatenate([top, wall])


@pytest.mark.parametrize("yaw", [30.0, -60.0, 0.0])
def test_footprint_measures_a_rotated_box(yaw: float) -> None:
    box = footprint(_box_cloud(0.30, 0.10, yaw, (0.5, -0.1)))
    assert box is not None
    (cx, cy), (length, width), measured, (bottom, top) = box
    assert (cx, cy) == pytest.approx((0.5, -0.1), abs=0.01)
    assert (length, width) == pytest.approx((0.30, 0.10), abs=0.02)
    assert measured == pytest.approx(yaw, abs=2.0)
    assert (bottom, top) == pytest.approx((0.91, 0.95), abs=0.005)

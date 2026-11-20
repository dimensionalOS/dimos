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

import pytest
from reactivex.disposable import Disposable

from dimos.memory.store.memory import MemoryStore
from dimos.perception.localize.memento import Memento
from dimos.perception.localize.module import PerceptionLocalize


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

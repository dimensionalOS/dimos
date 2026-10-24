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

from enum import Enum
import importlib.util
from pathlib import Path
import sys
import threading

import pytest

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT


@pytest.fixture
def camera(mocker, monkeypatch):
    # Load an isolated copy with the optional native SDK replaced at its boundary.
    sdk = mocker.Mock()
    sdk.DEPTH_MODE = Enum("DEPTH_MODE", ["NEURAL"])
    monkeypatch.setitem(sys.modules, "pyzed", mocker.Mock(sl=sdk))
    monkeypatch.setitem(sys.modules, "pyzed.sl", sdk)
    spec = importlib.util.spec_from_file_location(
        "dimos.hardware.sensors.camera.zed._camera_shutdown_test",
        Path(__file__).with_name("camera.py"),
    )
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)

    camera = module.ZEDCamera()
    camera._zed = mocker.Mock()
    camera._thread = mocker.Mock(spec=threading.Thread)
    camera._thread.is_alive.return_value = True
    camera._image_left = mocker.sentinel.image_left
    camera._depth_map = mocker.sentinel.depth_map
    camera._tracking_enabled = True
    camera._running = True
    yield camera
    if camera._thread is not None:
        camera._thread.is_alive.return_value = False
    camera.stop()


def test_stop_keeps_camera_and_buffers_alive_until_capture_finishes(camera):
    sdk_camera = camera._zed
    capture_thread = camera._thread
    depth_map = camera._depth_map

    def finish_frame(*, timeout):
        assert timeout == DEFAULT_THREAD_JOIN_TIMEOUT
        assert not camera._running
        assert camera._zed is sdk_camera
        assert camera._depth_map is depth_map
        sdk_camera.close.assert_not_called()
        sdk_camera.disable_positional_tracking.assert_not_called()
        capture_thread.is_alive.return_value = False

    capture_thread.join.side_effect = finish_frame

    camera.stop()

    capture_thread.join.assert_called_once_with(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
    sdk_camera.close.assert_called_once_with()
    sdk_camera.disable_positional_tracking.assert_called_once_with()
    assert camera._zed is None
    assert camera._thread is None
    assert camera._depth_map is None


def test_stop_timeout_preserves_resources_until_a_later_stop(camera):
    sdk_camera = camera._zed
    capture_thread = camera._thread
    image_left, depth_map = camera._image_left, camera._depth_map

    with pytest.raises(TimeoutError, match="ZED capture thread did not stop"):
        camera.stop()

    assert not camera._running
    assert camera._zed is sdk_camera
    assert camera._thread is capture_thread
    assert camera._image_left is image_left
    assert camera._depth_map is depth_map
    sdk_camera.close.assert_not_called()
    sdk_camera.disable_positional_tracking.assert_not_called()

    capture_thread.is_alive.return_value = False
    camera.stop()

    sdk_camera.close.assert_called_once_with()
    assert camera._zed is None
    assert camera._thread is None

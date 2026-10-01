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

import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock

from dimos_generated.sensor_msgs.msg import Image
import numpy as np
import pytest

from dimos.msgs.image import image_view
from dimos.msgs.time import to_nanoseconds


@pytest.fixture
def camera_module(monkeypatch):
    gi = ModuleType("gi")
    gi.require_version = Mock()
    repository = ModuleType("gi.repository")
    repository.GLib = Mock()
    repository.Gst = SimpleNamespace(
        init=Mock(),
        FlowReturn=SimpleNamespace(OK=0, ERROR=-1),
        MapFlags=SimpleNamespace(READ=1),
        CLOCK_TIME_NONE=-1,
    )
    monkeypatch.setitem(sys.modules, "gi", gi)
    monkeypatch.setitem(sys.modules, "gi.repository", repository)
    spec = importlib.util.spec_from_file_location(
        "_gstreamer_generated_test", Path(__file__).with_name("gstreamer_camera.py")
    )
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    return module


def test_generated_bgr_frame_owns_pixels_after_sdk_buffer_unmap(camera_module, monkeypatch):
    camera = camera_module.GstreamerCameraModule(frame_id="camera_test")
    camera.running = True
    pixels = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
    raw = bytearray(pixels.tobytes())
    mapped = SimpleNamespace(data=raw)
    buffer = Mock(pts=1700000000125000000)
    buffer.map.return_value = (True, mapped)
    buffer.unmap.side_effect = lambda info: raw.__setitem__(slice(None), bytes(18))
    caps = Mock()
    caps.get_structure.return_value.get_value.side_effect = {"width": 3, "height": 2}.__getitem__
    sample = Mock()
    sample.get_buffer.return_value = buffer
    sample.get_caps.return_value = caps
    sink = Mock()
    sink.emit.return_value = sample
    received = []
    monkeypatch.setattr(camera.video, "publish", received.append)
    try:
        assert camera._on_new_sample(sink) == 0
        assert len(received) == 1 and type(received[0]) is Image
        value = Image.decode(received[0].encode())
        assert value.header.frame_id == "camera_test"
        assert value.encoding == "bgr8" and value.step == 9
        assert to_nanoseconds(value.header.stamp) == 1700000000125000000
        np.testing.assert_array_equal(image_view(value), pixels)
        buffer.unmap.assert_called_once_with(mapped)
        # Existing invalid/pre-2000 timestamp rejection occurs before mapping.
        buffer.pts = 1
        assert camera._on_new_sample(sink) == 0 and len(received) == 1
        assert buffer.map.call_count == 1
    finally:
        camera.stop()

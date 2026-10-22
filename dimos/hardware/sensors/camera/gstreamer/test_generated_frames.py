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

"""Actual in-memory GStreamer buffers/pipelines; no camera, TCP stream or fake SDK."""

import gc
import threading
from unittest.mock import Mock

from dimos_cdrgen.registry import decode, encode
import numpy as np
import pytest

from dimos.message_utils.image import image_view
from dimos.message_utils.time import to_nanoseconds
from dimos.messages.sensor_msgs.msg import Image
from dimos.protocol.rpc.zenohrpc import ZenohRPC

gi = pytest.importorskip("gi", reason="GStreamer acceptance requires the optional GI binding")
gi.require_version("Gst", "1.0")
from gi.repository import Gst

from dimos.hardware.sensors.camera.gstreamer.gstreamer_camera import GstreamerCameraModule


@pytest.fixture
def camera(mocker):
    mocker.patch.object(ZenohRPC, "start")
    mocker.patch.object(ZenohRPC, "serve_module_rpc")
    instance = GstreamerCameraModule(frame_id="camera_test")
    yield instance
    instance.stop()


@pytest.mark.parametrize("offset,delta_ns", [(0.0, 0), (1e-9, 1), (-1e-9, -1)])
def test_actual_pipeline_copies_padded_bgr_and_exact_stamp(
    camera, mocker, offset, delta_ns
) -> None:
    camera.config.timestamp_offset = offset
    camera.running = True
    pipeline = Gst.parse_launch(
        "appsrc name=source format=time ! video/x-raw,format=BGR,width=3,height=2,framerate=30/1 "
        "! appsink name=sink emit-signals=true sync=false"
    )
    sink, source = pipeline.get_by_name("sink"), pipeline.get_by_name("source")
    sink.connect("new-sample", camera._on_new_sample)
    received = threading.Event()
    published = []

    def publish(value):
        published.append(value)
        received.set()

    mocker.patch.object(camera.video, "publish", side_effect=publish)
    pixels = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
    raw = bytearray([99] * 24)
    np.ndarray((2, 3, 3), dtype=np.uint8, buffer=raw, strides=(12, 3, 1))[:] = pixels
    buffer = Gst.Buffer.new_allocate(None, len(raw), None)
    buffer.fill(0, raw)
    buffer.pts = 1700000000123456789
    try:
        pipeline.set_state(Gst.State.PLAYING)
        assert source.emit("push-buffer", buffer) == Gst.FlowReturn.OK
        source.emit("end-of-stream")
        assert received.wait(timeout=5)
    finally:
        pipeline.set_state(Gst.State.NULL)
    del buffer, raw, pipeline
    gc.collect()
    assert len(published) == 1
    assert type(published[0]) is Image
    message = decode(encode(published[0]), Image)
    assert message.encoding == "bgr8" and message.step == 9
    assert to_nanoseconds(message.header.stamp) == 1700000000123456789 + delta_ns
    assert message.header.frame_id == "camera_test"
    np.testing.assert_array_equal(image_view(message), pixels)


def test_invalid_source_stamp_keeps_existing_skip_behavior(camera, mocker) -> None:
    buffer = Gst.Buffer.new_allocate(None, 24, None)
    buffer.pts = 0
    caps = Gst.Caps.from_string("video/x-raw,format=BGR,width=3,height=2,framerate=30/1")
    sample = Gst.Sample.new(buffer, caps, None, None)
    sink = Mock()
    sink.emit.return_value = sample
    publish = mocker.patch.object(camera.video, "publish")
    assert camera._on_new_sample(sink) == Gst.FlowReturn.OK
    publish.assert_not_called()


def test_stop_closes_module_loop_when_pipeline_was_never_started(camera):
    loop_thread = camera._loop_thread
    camera.stop()
    assert not loop_thread.is_alive()

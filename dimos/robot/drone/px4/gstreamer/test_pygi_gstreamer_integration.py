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

from __future__ import annotations

from threading import Event

import pytest

from dimos.robot.drone.px4.gstreamer import pygi_gstreamer
from dimos.robot.drone.px4.gstreamer.gstreamer_api import (
    GstBusEvent,
    GstBusMessage,
    GstEncoder,
    GstSource,
    GstSourceConfig,
    GstTeePipelineSpec,
)


def _test_pipeline() -> pygi_gstreamer.PyGstPipeline:
    return pygi_gstreamer.PyGstPipeline(
        GstTeePipelineSpec(
            source=GstSourceConfig(
                source=GstSource.VIDEOTEST,
                encoder=GstEncoder.X264,
                width=16,
                height=16,
                fps=1,
            )
        )
    )


def test_bus_eos_is_dispatched_by_the_owned_glib_context() -> None:
    gi = pytest.importorskip("gi")
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst

    received_eos = Event()
    adapter = _test_pipeline()
    adapter.set_callbacks(
        lambda _sample: None,
        lambda _sample: None,
        lambda message: received_eos.set() if message.event is GstBusEvent.EOS else None,
    )
    adapter.start()
    try:
        assert adapter._pipeline.send_event(Gst.Event.new_eos())
        assert received_eos.wait(1.0), "EOS was not dispatched without caller GLib iteration"
    finally:
        adapter.stop()


def test_bus_callback_can_stop_pipeline_without_joining_its_own_thread() -> None:
    gi = pytest.importorskip("gi")
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst

    stopped = Event()
    adapter = _test_pipeline()

    def stop_on_eos(message: GstBusMessage) -> None:
        if message.event is GstBusEvent.EOS:
            adapter.stop()
            stopped.set()

    adapter.set_callbacks(lambda _sample: None, lambda _sample: None, stop_on_eos)
    adapter.start()
    try:
        assert adapter._pipeline.send_event(Gst.Event.new_eos())
        assert stopped.wait(1.0), "EOS callback deadlocked while stopping its own main loop"
    finally:
        adapter.stop()

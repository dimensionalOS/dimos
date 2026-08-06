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

"""Optional PyGObject implementation of the typed PX4 GStreamer boundary."""

from __future__ import annotations

from collections.abc import Callable
from importlib import import_module
from threading import Event, Thread, current_thread
from typing import Protocol

from dimos.robot.drone.px4.gstreamer.gstreamer_api import (
    BusCallback,
    GstBusEvent,
    GstBusMessage,
    GstPipelineError,
    GstTeePipelineSpec,
    H264AccessUnit,
    H264SampleCallback,
    RawBgrSample,
    RawSampleCallback,
)


class GstMapInfo(Protocol):
    data: bytes | memoryview


class GstBuffer(Protocol):
    pts: int

    def map(self, flags: int) -> tuple[bool, GstMapInfo]: ...

    def unmap(self, mapping: GstMapInfo) -> None: ...

    def has_flags(self, flags: int) -> bool: ...


class GstCapsStructure(Protocol):
    def get_value(self, name: str) -> int: ...


class GstCaps(Protocol):
    def get_structure(self, index: int) -> GstCapsStructure: ...


class GstSample(Protocol):
    def get_buffer(self) -> GstBuffer: ...

    def get_caps(self) -> GstCaps: ...


class GstAppSink(Protocol):
    def emit(self, signal: str) -> GstSample | None: ...

    def connect(self, signal: str, callback: Callable[[GstAppSink], int]) -> int: ...


class GstBus(Protocol):
    def add_signal_watch(self) -> None: ...

    def remove_signal_watch(self) -> None: ...

    def connect(self, signal: str, callback: Callable[[GstBus, GstMessage], None]) -> int: ...


class GstMessage(Protocol):
    type: int

    def parse_error(self) -> tuple[Exception, str]: ...


class GlibMainContext(Protocol):
    def push_thread_default(self) -> None: ...

    def pop_thread_default(self) -> None: ...

    def iteration(self, may_block: bool) -> bool: ...

    def wakeup(self) -> None: ...


class PyGstApi:
    """Create a real local pipeline while preserving the small typed camera seam."""

    def create_pipeline(self, spec: GstTeePipelineSpec) -> PyGstPipeline:
        return PyGstPipeline(spec)


class PyGstPipeline:
    """Adapt PyGObject callbacks to copied, typed sample values."""

    def __init__(self, spec: GstTeePipelineSpec) -> None:
        try:
            gi = import_module("gi")
        except ModuleNotFoundError as error:
            raise GstPipelineError(detail="PyGObject gi is unavailable") from error
        try:
            gi.require_version("Gst", "1.0")
        except ValueError as error:
            raise GstPipelineError(detail="PyGObject cannot require Gst 1.0") from error
        try:
            gst = import_module("gi.repository.Gst")
        except ModuleNotFoundError as error:
            raise GstPipelineError(detail="PyGObject Gst namespace is unavailable") from error
        try:
            glib = import_module("gi.repository.GLib")
        except ModuleNotFoundError as error:
            raise GstPipelineError(detail="PyGObject GLib namespace is unavailable") from error
        gst.init(None)
        self._gst = gst
        self._main_context: GlibMainContext = glib.MainContext.new()
        self._main_context_stop = Event()
        self._main_context_ready = Event()
        self._main_context_thread: Thread | None = None
        self._pipeline = gst.parse_launch(spec.render())
        self._raw_sink = self._pipeline.get_by_name("raw_sink")
        self._h264_sink = self._pipeline.get_by_name("h264_sink")
        if self._raw_sink is None or self._h264_sink is None:
            raise GstPipelineError(detail="required appsinks are missing")
        self._raw_callback: RawSampleCallback | None = None
        self._h264_callback: H264SampleCallback | None = None
        self._bus_callback: BusCallback | None = None
        self._bus: GstBus | None = None
        self._bus_watch_added = False

    def set_callbacks(
        self,
        raw_callback: RawSampleCallback,
        h264_callback: H264SampleCallback,
        bus_callback: BusCallback,
    ) -> None:
        self._raw_callback = raw_callback
        self._h264_callback = h264_callback
        self._bus_callback = bus_callback
        self._raw_sink.connect("new-sample", self._on_raw_sample)
        self._h264_sink.connect("new-sample", self._on_h264_sample)
        bus = self._pipeline.get_bus()
        self._bus = bus
        if not self._bus_watch_added:
            self._main_context.push_thread_default()
            try:
                bus.add_signal_watch()
            finally:
                self._main_context.pop_thread_default()
            self._bus_watch_added = True
        bus.connect("message", self._on_bus_message)

    def start(self) -> None:
        self._main_context_stop.clear()
        self._main_context_ready.clear()
        self._main_context_thread = Thread(
            target=self._run_main_context,
            name="px4-gstreamer-bus",
            daemon=True,
        )
        self._main_context_thread.start()
        if not self._main_context_ready.wait(5.0):
            self._stop_main_context()
            raise GstPipelineError(detail="GLib main context failed to start")
        state = self._pipeline.set_state(self._gst.State.PLAYING)
        if state == self._gst.StateChangeReturn.FAILURE:
            self._stop_main_context()
            raise GstPipelineError(detail="pipeline failed to enter PLAYING")

    def stop(self) -> None:
        bus = self._bus
        if bus is not None and self._bus_watch_added:
            bus.remove_signal_watch()
            self._bus_watch_added = False
        self._pipeline.set_state(self._gst.State.NULL)
        self._stop_main_context()

    def _run_main_context(self) -> None:
        _ = self._main_context.iteration(False)
        self._main_context_ready.set()
        while not self._main_context_stop.is_set():
            _ = self._main_context.iteration(True)

    def _stop_main_context(self) -> None:
        thread = self._main_context_thread
        if thread is None:
            return
        self._main_context_thread = None
        self._main_context_stop.set()
        self._main_context.wakeup()
        if thread is not current_thread():
            thread.join()

    def _on_raw_sample(self, sink: GstAppSink) -> int:
        sample = sink.emit("pull-sample")
        if sample is None:
            return self._gst.FlowReturn.ERROR
        buffer = sample.get_buffer()
        caps = sample.get_caps().get_structure(0)
        mapped, mapping = buffer.map(self._gst.MapFlags.READ)
        if not mapped:
            return self._gst.FlowReturn.ERROR
        try:
            callback = self._raw_callback
            if callback is not None:
                callback(
                    RawBgrSample(
                        data=bytes(mapping.data),
                        width=int(caps.get_value("width")),
                        height=int(caps.get_value("height")),
                        pts_ns=self._pts_ns(buffer.pts),
                    )
                )
        finally:
            buffer.unmap(mapping)
        return self._gst.FlowReturn.OK

    def _on_h264_sample(self, sink: GstAppSink) -> int:
        sample = sink.emit("pull-sample")
        if sample is None:
            return self._gst.FlowReturn.ERROR
        buffer = sample.get_buffer()
        mapped, mapping = buffer.map(self._gst.MapFlags.READ)
        if not mapped:
            return self._gst.FlowReturn.ERROR
        try:
            callback = self._h264_callback
            if callback is not None:
                callback(
                    H264AccessUnit(
                        data=bytes(mapping.data),
                        pts_ns=self._pts_ns(buffer.pts),
                        delta_unit=buffer.has_flags(self._gst.BufferFlags.DELTA_UNIT),
                    )
                )
        finally:
            buffer.unmap(mapping)
        return self._gst.FlowReturn.OK

    def _on_bus_message(self, _bus: GstBus, message: GstMessage) -> None:
        callback = self._bus_callback
        if callback is None:
            return
        if message.type == self._gst.MessageType.ERROR:
            error, debug = message.parse_error()
            callback(GstBusMessage(event=GstBusEvent.ERROR, detail=f"{error}: {debug}"))
        if message.type == self._gst.MessageType.EOS:
            callback(GstBusMessage(event=GstBusEvent.EOS, detail="end of stream"))

    def _pts_ns(self, pts: int) -> int:
        if pts == self._gst.CLOCK_TIME_NONE:
            return 0
        return int(pts)

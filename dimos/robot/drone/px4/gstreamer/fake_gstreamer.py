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

"""Deterministic in-memory Gst implementation for local PX4 camera tests."""

from dimos.robot.drone.px4.gstreamer.gstreamer_api import (
    BusCallback,
    GstBusEvent,
    GstBusMessage,
    GstPipeline,
    GstPipelineError,
    GstTeePipelineSpec,
    H264AccessUnit,
    H264SampleCallback,
    RawBgrSample,
    RawSampleCallback,
)


class FakeGstPipeline:
    """Mutable because tests deterministically drive pipeline callbacks and lifecycle."""

    def __init__(self, spec: GstTeePipelineSpec, start_error: GstPipelineError | None) -> None:
        self.spec: GstTeePipelineSpec = spec
        self._start_error: GstPipelineError | None = start_error
        self._raw_callback: RawSampleCallback | None = None
        self._h264_callback: H264SampleCallback | None = None
        self._bus_callback: BusCallback | None = None
        self.started: bool = False
        self.stop_count: int = 0

    def set_callbacks(
        self,
        raw_callback: RawSampleCallback,
        h264_callback: H264SampleCallback,
        bus_callback: BusCallback,
    ) -> None:
        self._raw_callback = raw_callback
        self._h264_callback = h264_callback
        self._bus_callback = bus_callback

    def start(self) -> None:
        if self._start_error is not None:
            raise self._start_error
        self.started = True

    def stop(self) -> None:
        self.stop_count += 1
        self.started = False

    def emit_raw(self, sample: RawBgrSample) -> None:
        callback = self._raw_callback
        if callback is None:
            raise GstPipelineError(detail="raw callback is not configured")
        callback(sample)

    def emit_h264(self, sample: H264AccessUnit) -> None:
        callback = self._h264_callback
        if callback is None:
            raise GstPipelineError(detail="H.264 callback is not configured")
        callback(sample)

    def emit_bus(self, event: GstBusEvent, detail: str) -> None:
        callback = self._bus_callback
        if callback is None:
            raise GstPipelineError(detail="bus callback is not configured")
        callback(GstBusMessage(event=event, detail=detail))


class FakeGstApi:
    """Records one pipeline specification and exposes its callbacks to tests."""

    def __init__(self, start_error: GstPipelineError | None = None) -> None:
        self._start_error: GstPipelineError | None = start_error
        self.pipeline: FakeGstPipeline | None = None

    @property
    def pipeline_description(self) -> str:
        pipeline = self._pipeline()
        return pipeline.spec.render()

    @property
    def stop_count(self) -> int:
        pipeline = self._pipeline()
        return pipeline.stop_count

    def create_pipeline(self, spec: GstTeePipelineSpec) -> GstPipeline:
        if self.pipeline is not None:
            raise GstPipelineError(detail="fake API owns exactly one pipeline")
        self.pipeline = FakeGstPipeline(spec, self._start_error)
        return self.pipeline

    def emit_raw(self, sample: RawBgrSample) -> None:
        self._pipeline().emit_raw(sample)

    def emit_h264(self, sample: H264AccessUnit) -> None:
        self._pipeline().emit_h264(sample)

    def emit_bus(self, event: GstBusEvent, detail: str) -> None:
        self._pipeline().emit_bus(event, detail)

    def _pipeline(self) -> FakeGstPipeline:
        if self.pipeline is None:
            raise GstPipelineError(detail="pipeline has not been created")
        return self.pipeline

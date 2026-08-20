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

"""GStreamer tee camera with raw BGR and Annex-B H.264 outputs."""

from enum import Enum
import logging
from threading import Thread, current_thread
from typing import Any, ClassVar

import gi  # type: ignore[import-not-found,import-untyped]
import numpy as np
from pydantic import Field, model_validator
from typing_extensions import Self

gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
from gi.repository import GLib, Gst  # type: ignore[import-not-found,import-untyped]

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.utils.logging_config import setup_logger

Gst.init(None)
logger = setup_logger(level=logging.INFO)


class GstInputFormat(str, Enum):
    RAW = "raw"
    H264 = "h264"


class GstEncoder(str, Enum):
    NVV4L2 = "nvv4l2h264enc"
    X264 = "x264enc"


DEFAULT_INPUT_PIPELINE = (
    "v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480,framerate=30/1"
)


class GsTeeCameraConfig(ModuleConfig):
    input_pipeline: str = DEFAULT_INPUT_PIPELINE
    input_format: GstInputFormat = GstInputFormat.RAW
    encoder: GstEncoder = GstEncoder.NVV4L2
    bitrate: int = Field(default=4_000_000, ge=1_000)
    gop: int = Field(default=30, gt=0)

    @model_validator(mode="after")
    def reject_unused_encoder_options(self) -> Self:
        unused = {"encoder", "bitrate", "gop"} & self.model_fields_set
        if self.input_format is GstInputFormat.H264 and unused:
            names = ", ".join(sorted(unused))
            raise ValueError(f"{names} cannot be set when input_format is h264")
        return self


class GsTeeCamera(Module):
    """Own one live source pipeline and publish its raw and encoded tee branches."""

    config: GsTeeCameraConfig
    dedicated_worker: ClassVar[bool] = True

    color_image: Out[Image]
    video_h264: Out[CompressedVideo]

    def __init__(self, **kwargs: Any) -> None:
        kwargs.setdefault("frame_id", "camera_optical")
        super().__init__(**kwargs)
        self._pipeline: Any = None
        self._bus: Any = None
        self._main_loop: Any = None
        self._main_loop_thread: Thread | None = None

    @rpc
    def start(self) -> None:
        """Create, bind, and start the one owned tee pipeline."""
        if self._pipeline is not None:
            raise RuntimeError("camera is already running")
        super().start()

        if self.config.input_format is GstInputFormat.H264:
            raw_branch = (
                "t. ! queue max-size-buffers=2 leaky=downstream ! avdec_h264 ! "
                "videoconvert ! video/x-raw,format=BGR ! appsink name=raw_sink "
                "emit-signals=true sync=false max-buffers=1 drop=true"
            )
            h264_branch = (
                "t. ! queue max-size-buffers=2 ! "
                "video/x-h264,stream-format=byte-stream,alignment=au ! appsink "
                "name=h264_sink emit-signals=true sync=false max-buffers=1 drop=false"
            )
        else:
            raw_branch = (
                "t. ! queue max-size-buffers=2 leaky=downstream ! videoconvert ! "
                "video/x-raw,format=BGR ! appsink name=raw_sink emit-signals=true "
                "sync=false max-buffers=1 drop=true"
            )
            match self.config.encoder:
                case GstEncoder.NVV4L2:
                    encoder_input = "nvvidconv ! video/x-raw(memory:NVMM),format=NV12"
                    encoder = (
                        f"nvv4l2h264enc bitrate={self.config.bitrate} "
                        f"iframeinterval={self.config.gop} idrinterval={self.config.gop} "
                        "insert-sps-pps=true maxperf-enable=true"
                    )
                case GstEncoder.X264:
                    encoder_input = "videoconvert"
                    encoder = (
                        f"x264enc bitrate={self.config.bitrate // 1000} "
                        f"key-int-max={self.config.gop} bframes=0 tune=zerolatency"
                    )
            h264_branch = (
                f"t. ! queue max-size-buffers=2 ! {encoder_input} ! {encoder} ! "
                "h264parse config-interval=-1 ! "
                "video/x-h264,stream-format=byte-stream,alignment=au ! appsink "
                "name=h264_sink emit-signals=true sync=false max-buffers=1 drop=false"
            )

        pipeline = Gst.parse_launch(
            f"{self.config.input_pipeline} ! tee name=t {raw_branch} {h264_branch}"
        )
        raw_sink = pipeline.get_by_name("raw_sink")
        h264_sink = pipeline.get_by_name("h264_sink")
        if raw_sink is None or h264_sink is None:
            pipeline.set_state(Gst.State.NULL)
            raise RuntimeError("required appsinks are missing")
        raw_sink.connect("new-sample", self._on_raw_sample)
        h264_sink.connect("new-sample", self._on_h264_sample)
        bus = pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)
        self._pipeline = pipeline
        self._bus = bus
        self._main_loop = GLib.MainLoop()
        self._main_loop_thread = Thread(
            target=self._main_loop.run,
            name="px4-gstreamer-bus",
            daemon=True,
        )
        self._main_loop_thread.start()
        if pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            self._release_pipeline()
            raise RuntimeError("pipeline failed to enter PLAYING")

    @rpc
    def stop(self) -> None:
        """Release the owned Gst pipeline before closing Module resources."""
        self._release_pipeline()
        super().stop()

    def _on_raw_sample(self, sink: Any) -> Any:
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR
        buffer = sample.get_buffer()
        mapped, mapping = buffer.map(Gst.MapFlags.READ)
        if not mapped:
            return Gst.FlowReturn.ERROR
        try:
            caps = sample.get_caps().get_structure(0)
            width = int(caps.get_value("width"))
            height = int(caps.get_value("height"))
            pts = 0 if buffer.pts == Gst.CLOCK_TIME_NONE else int(buffer.pts)
            self.color_image.publish(
                Image(
                    data=np.frombuffer(mapping.data, dtype=np.uint8)
                    .reshape((height, width, 3))
                    .copy(),
                    format=ImageFormat.BGR,
                    frame_id=self.frame_id,
                    ts=pts / 1_000_000_000,
                )
            )
        finally:
            buffer.unmap(mapping)
        return Gst.FlowReturn.OK

    def _on_h264_sample(self, sink: Any) -> Any:
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR
        buffer = sample.get_buffer()
        mapped, mapping = buffer.map(Gst.MapFlags.READ)
        if not mapped:
            return Gst.FlowReturn.ERROR
        try:
            pts = 0 if buffer.pts == Gst.CLOCK_TIME_NONE else int(buffer.pts)
            self.video_h264.publish(
                CompressedVideo(
                    data=bytes(mapping.data),
                    format="h264",
                    frame_id=self.frame_id,
                    ts=pts / 1_000_000_000,
                )
            )
        finally:
            buffer.unmap(mapping)
        return Gst.FlowReturn.OK

    def _on_bus_message(self, _bus: Any, message: Any) -> None:
        if message.type == Gst.MessageType.ERROR:
            error, debug = message.parse_error()
            logger.error(f"GStreamer pipeline error: {error}: {debug}")
        elif message.type == Gst.MessageType.EOS:
            logger.info("GStreamer pipeline reached end of stream")
        else:
            return
        self._release_pipeline()

    def _release_pipeline(self) -> None:
        pipeline = self._pipeline
        self._pipeline = None
        bus = self._bus
        self._bus = None
        if bus is not None:
            bus.remove_signal_watch()
        if pipeline is not None:
            pipeline.set_state(Gst.State.NULL)
        main_loop = self._main_loop
        self._main_loop = None
        if main_loop is not None:
            main_loop.quit()
        thread = self._main_loop_thread
        self._main_loop_thread = None
        if thread is not None and thread is not current_thread():
            thread.join()

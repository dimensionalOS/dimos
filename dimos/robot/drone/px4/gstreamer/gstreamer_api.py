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

"""Typed boundary between the camera module and optional PyGObject Gst."""

from collections.abc import Callable, Mapping
from dataclasses import dataclass
from enum import Enum
import os
import re
from typing import Final, Protocol

SAFE_V4L2_DEVICE: Final = re.compile(r"/dev/(?:[A-Za-z0-9._+:\-]+/)*[A-Za-z0-9._+:\-]+")


class GstSource(str, Enum):
    """The only source elements accepted by the local camera pipeline."""

    V4L2 = "v4l2src"
    VIDEOTEST = "videotestsrc"
    UDP_RTP_H264 = "udp-rtp-h264"


class GstBusEvent(str, Enum):
    """Terminal events delivered by a pipeline bus."""

    ERROR = "error"
    EOS = "eos"


class GstEncoder(str, Enum):
    """The supported local H.264 encoders with no implicit fallback."""

    NVV4L2 = "nvv4l2h264enc"
    X264 = "x264enc"


@dataclass(frozen=True, slots=True)
class GstPipelineError(RuntimeError):
    """A pipeline could not be created or transitioned."""

    detail: str

    def __str__(self) -> str:
        return self.detail


@dataclass(frozen=True, slots=True)
class RawBgrSample:
    """One raw BGR frame copied from the raw appsink."""

    data: bytes
    width: int
    height: int
    pts_ns: int


@dataclass(frozen=True, slots=True)
class H264AccessUnit:
    """One Annex-B H.264 access unit copied from the encoded appsink."""

    data: bytes
    pts_ns: int
    delta_unit: bool


@dataclass(frozen=True, slots=True)
class GstBusMessage:
    """A terminal pipeline-bus diagnostic."""

    event: GstBusEvent
    detail: str


RawSampleCallback = Callable[[RawBgrSample], None]
H264SampleCallback = Callable[[H264AccessUnit], None]
BusCallback = Callable[[GstBusMessage], None]


@dataclass(frozen=True, slots=True)
class GstSourceConfig:
    """Capture settings shared before the tee splits the source stream."""

    source: GstSource = GstSource.V4L2
    device: str = "/dev/video0"
    udp_port: int = 5600
    width: int = 640
    height: int = 480
    fps: int = 30
    encoder: GstEncoder = GstEncoder.NVV4L2
    bitrate: int = 4_000_000
    gop: int = 30

    def __post_init__(self) -> None:
        match self.source:
            case GstSource.V4L2:
                _validate_v4l2_device(self.device)
            case GstSource.VIDEOTEST:
                return
            case GstSource.UDP_RTP_H264:
                _validate_udp_port(self.udp_port)

    def render(self) -> str:
        """Render exactly one live source element and its negotiated raw caps."""
        element: str
        match self.source:
            case GstSource.V4L2:
                element = f"v4l2src device={self.device}"
            case GstSource.VIDEOTEST:
                element = "videotestsrc is-live=true"
            case GstSource.UDP_RTP_H264:
                return (
                    f"udpsrc port={self.udp_port} caps=application/x-rtp,media=video,"
                    "encoding-name=H264,payload=96 ! rtph264depay ! h264parse config-interval=-1"
                )
        return f"{element} ! video/x-raw,width={self.width},height={self.height},framerate={self.fps}/1"


def source_config_from_environment(
    environment: Mapping[str, str] | None = None,
) -> GstSourceConfig:
    """Parse strictly typed PX4 source and encoder settings from the environment."""
    values = os.environ if environment is None else environment
    source = _source_from_value(values.get("DIMOS_PX4_CAMERA_SOURCE"))
    return GstSourceConfig(
        source=source,
        device=values.get("DIMOS_PX4_CAMERA_DEVICE", "/dev/video0"),
        udp_port=_udp_port_from_environment(values) if source is GstSource.UDP_RTP_H264 else 5600,
        width=_positive_integer(values, "DIMOS_PX4_CAMERA_WIDTH", 640),
        height=_positive_integer(values, "DIMOS_PX4_CAMERA_HEIGHT", 480),
        fps=_positive_integer(values, "DIMOS_PX4_CAMERA_FPS", 30),
        encoder=_encoder_from_value(values.get("DIMOS_PX4_CAMERA_ENCODER")),
        bitrate=_positive_integer(values, "DIMOS_PX4_CAMERA_BITRATE", 4_000_000),
        gop=_positive_integer(values, "DIMOS_PX4_CAMERA_GOP", 30),
    )


def _source_from_value(value: str | None) -> GstSource:
    match value:
        case None | "v4l2src":
            return GstSource.V4L2
        case "videotestsrc":
            return GstSource.VIDEOTEST
        case "udp-rtp-h264":
            return GstSource.UDP_RTP_H264
        case unexpected:
            raise GstPipelineError(
                detail=(
                    "DIMOS_PX4_CAMERA_SOURCE must be v4l2src, videotestsrc, or udp-rtp-h264, "
                    f"got {unexpected!r}"
                )
            )


def _validate_v4l2_device(device: str) -> None:
    if SAFE_V4L2_DEVICE.fullmatch(device) and ".." not in device.split("/"):
        return
    raise GstPipelineError(
        detail=f"DIMOS_PX4_CAMERA_DEVICE must be a safe absolute /dev path, got {device!r}"
    )


def _validate_udp_port(port: int) -> None:
    if 1 <= port <= 65_535:
        return
    raise GstPipelineError(
        detail=f"DIMOS_PX4_CAMERA_UDP_PORT must be between 1 and 65535, got {port!r}"
    )


def _udp_port_from_environment(values: Mapping[str, str]) -> int:
    port = _positive_integer(values, "DIMOS_PX4_CAMERA_UDP_PORT", 5600)
    _validate_udp_port(port)
    return port


def _encoder_from_value(value: str | None) -> GstEncoder:
    match value:
        case None | "nvv4l2h264enc":
            return GstEncoder.NVV4L2
        case "x264enc":
            return GstEncoder.X264
        case unexpected:
            raise GstPipelineError(
                detail=f"DIMOS_PX4_CAMERA_ENCODER must be nvv4l2h264enc or x264enc, got {unexpected!r}"
            )


def _positive_integer(values: Mapping[str, str], name: str, default: int) -> int:
    value = values.get(name)
    if value is None:
        return default
    try:
        parsed = int(value)
    except ValueError as error:
        raise GstPipelineError(
            detail=f"{name} must be a positive integer, got {value!r}"
        ) from error
    if parsed < 1:
        raise GstPipelineError(detail=f"{name} must be a positive integer, got {value!r}")
    return parsed


@dataclass(frozen=True, slots=True)
class GstTeePipelineSpec:
    """The single-source, independently queued raw and H.264 pipeline contract."""

    source: GstSourceConfig

    def render(self) -> str:
        """Render the GStreamer launch description consumed by the optional adapter."""
        if self.source.source is GstSource.UDP_RTP_H264:
            return self._render_udp_rtp_h264()
        raw_branch = (
            "t. ! queue max-size-buffers=2 leaky=downstream ! videoconvert ! "
            "video/x-raw,format=BGR ! appsink name=raw_sink emit-signals=true "
            "sync=false max-buffers=1 drop=true"
        )
        encoder = self._render_encoder()
        h264_branch = (
            "t. ! queue max-size-buffers=2 ! videoconvert ! "
            f"{encoder} ! h264parse config-interval=-1 ! "
            "video/x-h264,stream-format=byte-stream,alignment=au ! appsink "
            "name=h264_sink emit-signals=true sync=false max-buffers=1 drop=false"
        )
        return f"{self.source.render()} ! tee name=t {raw_branch} {h264_branch}"

    def _render_udp_rtp_h264(self) -> str:
        raw_branch = (
            "t. ! queue max-size-buffers=2 leaky=downstream ! avdec_h264 ! videoconvert ! "
            "video/x-raw,format=BGR ! appsink name=raw_sink emit-signals=true "
            "sync=false max-buffers=1 drop=true"
        )
        h264_branch = (
            "t. ! queue max-size-buffers=2 ! "
            "video/x-h264,stream-format=byte-stream,alignment=au ! appsink "
            "name=h264_sink emit-signals=true sync=false max-buffers=1 drop=false"
        )
        return f"{self.source.render()} ! tee name=t {raw_branch} {h264_branch}"

    def _render_encoder(self) -> str:
        match self.source.encoder:
            case GstEncoder.NVV4L2:
                return (
                    f"nvv4l2h264enc bitrate={self.source.bitrate} iframeinterval={self.source.gop} "
                    f"idrinterval={self.source.gop} insert-sps-pps=true maxperf-enable=true"
                )
            case GstEncoder.X264:
                return "x264enc bframes=0 tune=zerolatency"


class GstPipeline(Protocol):
    """Minimal lifecycle and callback surface shared by real and fake pipelines."""

    def set_callbacks(
        self,
        raw_callback: RawSampleCallback,
        h264_callback: H264SampleCallback,
        bus_callback: BusCallback,
    ) -> None: ...

    def start(self) -> None: ...

    def stop(self) -> None: ...


class GstApi(Protocol):
    """Factory for an owned local GStreamer pipeline."""

    def create_pipeline(self, spec: GstTeePipelineSpec) -> GstPipeline: ...

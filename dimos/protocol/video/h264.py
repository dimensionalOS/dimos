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

from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from fractions import Fraction
from typing import TYPE_CHECKING, Protocol

import numpy as np

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.VideoPacket import VideoPacket

if TYPE_CHECKING:
    import av


class MissingVideoDependencyError(ImportError):
    """Raised when H.264 support is selected without required video packages."""


class UnsupportedVideoImageError(ValueError):
    """Raised when an image cannot be represented by the H.264 adapter."""


class VideoDecodeGapError(RuntimeError):
    """Raised when a decoder cannot safely decode because GOP state is invalid."""


@dataclass(frozen=True)
class H264Config:
    """Configuration for opt-in H.264 image encoding."""

    bitrate: int = 2_000_000
    target_fps: int = 30
    keyframe_interval: int = 30
    profile: str = "baseline"
    preset: str = "veryfast"
    tune: str = "zerolatency"
    max_gop_frames: int = 30
    pixel_format: str = "yuv420p"
    supported_formats: tuple[ImageFormat, ...] = field(
        default_factory=lambda: (ImageFormat.RGB, ImageFormat.BGR, ImageFormat.GRAY)
    )

    def __post_init__(self) -> None:
        if self.bitrate <= 0:
            raise ValueError("bitrate must be positive")
        if self.target_fps <= 0:
            raise ValueError("target_fps must be positive")
        if self.keyframe_interval <= 0:
            raise ValueError("keyframe_interval must be positive")
        if self.max_gop_frames <= 0:
            raise ValueError("max_gop_frames must be positive")


class H264CodecAdapter(Protocol):
    """DimOS-facing codec adapter; hides aiortc/RTP details from public APIs."""

    def encode_image(self, image: Image, *, force_keyframe: bool) -> tuple[bytes, int]: ...

    def decode_packet(self, packet: VideoPacket) -> Image: ...


@dataclass(frozen=True)
class H264AccessUnit:
    """Complete Annex B access unit for one source frame."""

    data: bytes

    @classmethod
    def from_rtp_payloads(
        cls,
        payloads: Sequence[bytes],
        depayload: Callable[[bytes], bytes],
    ) -> H264AccessUnit:
        """Assemble RTP-sized H.264 payloads into one Annex B access unit."""

        if not payloads:
            raise ValueError("H.264 encoder returned no payloads")
        data = b"".join(depayload(payload) for payload in payloads)
        if not data.startswith((b"\x00\x00\x01", b"\x00\x00\x00\x01")):
            raise ValueError("H.264 access unit is not Annex B byte-stream data")
        return cls(data=data)


def ensure_supported_image(image: Image, config: H264Config) -> None:
    """Validate the first-version H.264 image input contract."""

    if image.format not in config.supported_formats:
        supported = ", ".join(fmt.value for fmt in config.supported_formats)
        raise UnsupportedVideoImageError(
            f"H.264 image encoding supports {supported}; got {image.format.value}"
        )
    if image.dtype != np.dtype(np.uint8):
        raise UnsupportedVideoImageError(
            f"H.264 image encoding requires uint8 data; got {image.dtype}"
        )
    if image.channels not in (1, 3):
        raise UnsupportedVideoImageError(
            f"H.264 image encoding requires 1 or 3 channels; got {image.channels}"
        )


class AiortcH264Codec:
    """Small adapter around aiortc's H.264 encoder/decoder internals."""

    def __init__(self, config: H264Config | None = None) -> None:
        self.config = config or H264Config()
        try:
            from aiortc.codecs.h264 import (
                H264Decoder as AiortcDecoder,
                H264Encoder as AiortcEncoder,
                h264_depayload,
            )
            from aiortc.jitterbuffer import JitterFrame
            import av
        except ImportError as exc:
            raise MissingVideoDependencyError(
                "H.264 image mode requires aiortc, PyAV, FFmpeg, and H.264 codec support"
            ) from exc

        self._av = av
        self._jitter_frame_type = JitterFrame
        self._depayload = h264_depayload
        self._encoder = AiortcEncoder()
        self._decoder = AiortcDecoder()
        self._frame_index = 0
        self._time_base = Fraction(1, self.config.target_fps)
        if hasattr(self._encoder, "target_bitrate"):
            self._encoder.target_bitrate = self.config.bitrate

    def encode_image(self, image: Image, *, force_keyframe: bool) -> tuple[bytes, int]:
        ensure_supported_image(image, self.config)
        frame = self._to_video_frame(image)
        payloads, pts = self._encoder.encode(frame, force_keyframe=force_keyframe)
        access_unit = H264AccessUnit.from_rtp_payloads(payloads, self._depayload)
        return access_unit.data, int(pts)

    def decode_packet(self, packet: VideoPacket) -> Image:
        frame = self._jitter_frame_type(data=packet.data, timestamp=packet.pts)
        decoded_frames = self._decoder.decode(frame)
        if not decoded_frames:
            raise VideoDecodeGapError("H.264 decoder produced no frame")
        return self._from_video_frame(decoded_frames[0], packet)

    def _to_video_frame(self, image: Image) -> av.VideoFrame:
        fmt = _av_input_format(image.format)
        frame = self._av.VideoFrame.from_ndarray(np.ascontiguousarray(image.data), format=fmt)
        frame.pts = self._frame_index
        frame.time_base = self._time_base
        self._frame_index += 1
        return frame

    @staticmethod
    def _from_video_frame(frame: av.VideoFrame, packet: VideoPacket) -> Image:
        image_format = ImageFormat(packet.format)
        arr = frame.to_ndarray(format=_av_input_format(image_format))
        return Image(data=arr, format=image_format, frame_id=packet.frame_id, ts=packet.ts)


class H264Encoder:
    """Encode a normal DimOS Image stream into per-frame H.264 packets."""

    def __init__(
        self,
        config: H264Config | None = None,
        *,
        codec: H264CodecAdapter | None = None,
    ) -> None:
        self.config = config or H264Config()
        self._codec = codec or AiortcH264Codec(self.config)
        self._seq = 0
        self._keyframe_seq = -1

    def encode(self, image: Image, *, force_keyframe: bool = False) -> VideoPacket:
        ensure_supported_image(image, self.config)
        is_keyframe = self._should_force_keyframe(force_keyframe)
        access_unit, pts = self._codec.encode_image(image, force_keyframe=is_keyframe)
        if is_keyframe:
            self._keyframe_seq = self._seq
        packet = VideoPacket(
            seq=self._seq,
            ts=image.ts,
            frame_id=image.frame_id,
            width=image.width,
            height=image.height,
            format=image.format.value,
            codec="h264",
            bitstream="annex_b",
            is_keyframe=is_keyframe,
            keyframe_seq=self._keyframe_seq,
            pts=pts,
            data=access_unit,
        )
        self._seq += 1
        return packet

    def _should_force_keyframe(self, requested: bool) -> bool:
        if requested or self._seq == 0 or self._keyframe_seq < 0:
            return True
        since_keyframe = self._seq - self._keyframe_seq
        return since_keyframe >= min(self.config.keyframe_interval, self.config.max_gop_frames)


class GopBuffer:
    """Track H.264 GOP validity across a packet stream."""

    def __init__(self) -> None:
        self.expected_seq: int | None = None
        self.keyframe_seq: int | None = None
        self.valid = False

    def accept(self, packet: VideoPacket) -> bool:
        """Return True when the packet can be safely decoded."""

        if self.expected_seq is not None and packet.seq != self.expected_seq:
            self.valid = False
        self.expected_seq = packet.seq + 1

        if packet.is_keyframe:
            self.keyframe_seq = packet.seq
            self.valid = True
            return True

        if not self.valid:
            return False
        if self.keyframe_seq is None or packet.keyframe_seq != self.keyframe_seq:
            self.valid = False
            return False
        return True


class H264Decoder:
    """Decode per-frame H.264 packets into normal DimOS Images."""

    def __init__(
        self,
        config: H264Config | None = None,
        *,
        codec: H264CodecAdapter | None = None,
        gop_buffer: GopBuffer | None = None,
    ) -> None:
        self.config = config or H264Config()
        self._codec = codec or AiortcH264Codec(self.config)
        self._gop_buffer = gop_buffer or GopBuffer()

    def decode(self, packet: VideoPacket) -> Image:
        if not self._gop_buffer.accept(packet):
            raise VideoDecodeGapError(
                f"Cannot decode H.264 packet seq={packet.seq}; waiting for next keyframe"
            )
        return self._codec.decode_packet(packet)


def _av_input_format(format: ImageFormat) -> str:
    match format:
        case ImageFormat.RGB:
            return "rgb24"
        case ImageFormat.BGR:
            return "bgr24"
        case ImageFormat.GRAY:
            return "gray"
        case _:
            raise UnsupportedVideoImageError(f"Unsupported H.264 image format: {format.value}")


__all__ = [
    "AiortcH264Codec",
    "GopBuffer",
    "H264AccessUnit",
    "H264CodecAdapter",
    "H264Config",
    "H264Decoder",
    "H264Encoder",
    "MissingVideoDependencyError",
    "UnsupportedVideoImageError",
    "VideoDecodeGapError",
    "ensure_supported_image",
]

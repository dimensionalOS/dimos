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

from collections.abc import Callable, Iterator, Sequence
from dataclasses import dataclass, field
from fractions import Fraction
from typing import TYPE_CHECKING, Any, Protocol, cast

import numpy as np

from dimos.msgs.sensor_msgs.Image import H264_IMAGE_ENCODING, Image, ImageFormat

if TYPE_CHECKING:
    import av


H264_CODEC = "h264"
H264_BITSTREAM = "annex_b"


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

    def decode_image(self, image: Image) -> Image: ...


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

    if image.encoding != "raw":
        raise UnsupportedVideoImageError(
            f"H.264 encoding expects raw Image data; got encoding={image.encoding!r}"
        )
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


def h264_metadata(image: Image) -> dict[str, Any]:
    """Return validated H.264 metadata from an encoded Image."""

    if image.encoding != H264_IMAGE_ENCODING:
        raise ValueError(f"Expected H.264 encoded Image, got encoding={image.encoding!r}")
    metadata = image.codec_metadata
    if metadata.get("codec", H264_CODEC) != H264_CODEC:
        raise ValueError(f"Expected codec={H264_CODEC!r}, got {metadata.get('codec')!r}")
    if metadata.get("bitstream", H264_BITSTREAM) != H264_BITSTREAM:
        raise ValueError(
            f"Expected bitstream={H264_BITSTREAM!r}, got {metadata.get('bitstream')!r}"
        )
    for key in ("seq", "is_keyframe", "keyframe_seq", "pts", "width", "height"):
        if key not in metadata:
            raise ValueError(f"H.264 encoded Image missing metadata field {key!r}")
    if not isinstance(image.data, bytes):
        raise ValueError("H.264 encoded Image payload must be bytes")
    return metadata


class AiortcH264Codec:
    """Small adapter around aiortc's H.264 encoder/decoder internals."""

    def __init__(self, config: H264Config | None = None) -> None:
        self.config = config or H264Config()
        try:
            from aiortc.codecs.h264 import (
                MAX_FRAME_RATE,
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

        class ConfiguredAiortcEncoder(AiortcEncoder):
            def __init__(self, h264_config: H264Config) -> None:
                super().__init__()
                self._dimos_config = h264_config

            def _encode_frame(self, frame: av.VideoFrame, force_keyframe: bool) -> Iterator[bytes]:
                configured_bitrate = self.codec.bit_rate if self.codec else None
                if self.codec and (
                    frame.width != self.codec.width
                    or frame.height != self.codec.height
                    or configured_bitrate is None
                    or abs(self.target_bitrate - configured_bitrate) / configured_bitrate > 0.1
                ):
                    self.buffer_data = b""
                    self.buffer_pts = None
                    self.codec = None

                if force_keyframe:
                    frame.pict_type = av.video.frame.PictureType.I
                else:
                    frame.pict_type = av.video.frame.PictureType.NONE

                if self.codec is None:
                    self.codec = av.CodecContext.create("libx264", "w")
                    self.codec.width = frame.width
                    self.codec.height = frame.height
                    self.codec.bit_rate = self.target_bitrate
                    self.codec.pix_fmt = self._dimos_config.pixel_format
                    self.codec.framerate = Fraction(MAX_FRAME_RATE, 1)
                    self.codec.time_base = Fraction(1, MAX_FRAME_RATE)
                    self.codec.options = {
                        "level": "31",
                        "preset": self._dimos_config.preset,
                        "tune": self._dimos_config.tune,
                    }
                    self.codec.profile = _av_h264_profile(self._dimos_config.profile)

                data_to_send = b""
                for package in self.codec.encode(frame):
                    data_to_send += bytes(package)

                if data_to_send:
                    yield from self._split_bitstream(data_to_send)

        self._encoder = ConfiguredAiortcEncoder(self.config)
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

    def decode_image(self, image: Image) -> Image:
        metadata = h264_metadata(image)
        assert isinstance(image.data, bytes)
        frame = self._jitter_frame_type(data=image.data, timestamp=int(metadata["pts"]))
        decoded_frames = self._decoder.decode(frame)
        if not decoded_frames:
            raise VideoDecodeGapError("H.264 decoder produced no frame")
        return self._from_video_frame(cast("av.VideoFrame", decoded_frames[0]), image)

    def _to_video_frame(self, image: Image) -> av.VideoFrame:
        fmt = _av_input_format(image.format)
        frame = self._av.VideoFrame.from_ndarray(
            np.ascontiguousarray(image.require_raw("h264 encode")), format=fmt
        )
        frame.pts = self._frame_index
        frame.time_base = self._time_base
        self._frame_index += 1
        return frame

    @staticmethod
    def _from_video_frame(frame: av.VideoFrame, image: Image) -> Image:
        image_format = image.format
        arr = frame.to_ndarray(format=_av_input_format(image_format))
        return Image(data=arr, format=image_format, frame_id=image.frame_id, ts=image.ts)


class H264Encoder:
    """Encode a normal DimOS Image stream into per-frame H.264 Images."""

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

    def encode(self, image: Image, *, force_keyframe: bool = False) -> Image:
        ensure_supported_image(image, self.config)
        is_keyframe = self._should_force_keyframe(force_keyframe)
        access_unit, pts = self._codec.encode_image(image, force_keyframe=is_keyframe)
        if is_keyframe:
            self._keyframe_seq = self._seq
        metadata: dict[str, Any] = {
            "seq": self._seq,
            "codec": H264_CODEC,
            "bitstream": H264_BITSTREAM,
            "is_keyframe": is_keyframe,
            "keyframe_seq": self._keyframe_seq,
            "pts": pts,
            "width": image.width,
            "height": image.height,
            "channels": image.channels,
            "dtype": str(image.dtype),
        }
        self._seq += 1
        return Image.encoded(
            data=access_unit,
            encoding=H264_IMAGE_ENCODING,
            format=image.format,
            frame_id=image.frame_id,
            ts=image.ts,
            codec_metadata=metadata,
        )

    def _should_force_keyframe(self, requested: bool) -> bool:
        if requested or self._seq == 0 or self._keyframe_seq < 0:
            return True
        since_keyframe = self._seq - self._keyframe_seq
        return since_keyframe >= min(self.config.keyframe_interval, self.config.max_gop_frames)


class GopBuffer:
    """Track H.264 GOP validity across an encoded Image stream."""

    def __init__(self) -> None:
        self.expected_seq: int | None = None
        self.keyframe_seq: int | None = None
        self.valid = False

    def accept(self, image: Image) -> bool:
        """Return True when the encoded Image can be safely decoded."""

        metadata = h264_metadata(image)
        seq = int(metadata["seq"])
        keyframe_seq = int(metadata["keyframe_seq"])
        is_keyframe = bool(metadata["is_keyframe"])

        if self.expected_seq is not None and seq != self.expected_seq:
            self.valid = False
        self.expected_seq = seq + 1

        if is_keyframe:
            self.keyframe_seq = seq
            self.valid = True
            return True

        if not self.valid:
            return False
        if self.keyframe_seq is None or keyframe_seq != self.keyframe_seq:
            self.valid = False
            return False
        return True


class H264Decoder:
    """Decode H.264 encoded Images into normal raw DimOS Images."""

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

    def decode(self, image: Image) -> Image:
        metadata = h264_metadata(image)
        if not self._gop_buffer.accept(image):
            raise VideoDecodeGapError(
                f"Cannot decode H.264 image seq={metadata['seq']}; waiting for next keyframe"
            )
        return self._codec.decode_image(image)


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


def _av_h264_profile(profile: str) -> str:
    match profile.lower():
        case "baseline":
            return "Baseline"
        case "main":
            return "Main"
        case "high":
            return "High"
        case _:
            return profile


__all__ = [
    "H264_BITSTREAM",
    "H264_CODEC",
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
    "h264_metadata",
]

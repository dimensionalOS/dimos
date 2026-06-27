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

from collections.abc import Callable, Iterator, Mapping, Sequence
from dataclasses import dataclass, field
from fractions import Fraction
import json
import struct
from typing import TYPE_CHECKING, Any, Protocol, cast

import numpy as np

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat

if TYPE_CHECKING:
    import av


H264_CODEC = "h264"
H264_BITSTREAM = "annex_b"
_H264_PACKET_MAGIC = b"DIMH2641"


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

    def decode_packet(self, packet: H264Packet) -> Image: ...


@dataclass(frozen=True)
class H264Packet:
    """Internal encoded H.264 access unit for one source ``Image``.

    This is deliberately not a public module-facing message type. It is used by
    H.264 transports/storage backends as a physical representation while public
    stream APIs continue to expose decoded :class:`Image` values.
    """

    data: bytes
    format: ImageFormat
    frame_id: str
    ts: float
    seq: int
    pts: int
    is_keyframe: bool
    keyframe_seq: int
    width: int
    height: int
    channels: int
    dtype: str
    codec: str = H264_CODEC
    bitstream: str = H264_BITSTREAM

    def metadata(self) -> dict[str, Any]:
        return {
            "codec": self.codec,
            "bitstream": self.bitstream,
            "format": self.format.value,
            "frame_id": self.frame_id,
            "ts": self.ts,
            "seq": self.seq,
            "pts": self.pts,
            "is_keyframe": self.is_keyframe,
            "keyframe_seq": self.keyframe_seq,
            "width": self.width,
            "height": self.height,
            "channels": self.channels,
            "dtype": self.dtype,
        }

    def to_bytes(self) -> bytes:
        header = json.dumps(self.metadata(), sort_keys=True, separators=(",", ":")).encode("utf-8")
        return _H264_PACKET_MAGIC + struct.pack(">I", len(header)) + header + self.data

    @classmethod
    def from_bytes(cls, payload: bytes) -> H264Packet:
        if not payload.startswith(_H264_PACKET_MAGIC):
            raise ValueError("H.264 packet is missing DimOS packet envelope")
        offset = len(_H264_PACKET_MAGIC)
        if len(payload) < offset + 4:
            raise ValueError("H.264 packet envelope is truncated")
        header_len = struct.unpack(">I", payload[offset : offset + 4])[0]
        header_start = offset + 4
        header_end = header_start + header_len
        if header_end > len(payload):
            raise ValueError("H.264 packet metadata header is truncated")
        try:
            metadata = json.loads(payload[header_start:header_end].decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValueError("H.264 packet metadata header is invalid") from exc
        if not isinstance(metadata, dict):
            raise ValueError("H.264 packet metadata header must be a JSON object")
        data = payload[header_end:]
        return cls.from_parts(data=data, metadata=metadata)

    @classmethod
    def from_parts(cls, *, data: bytes, metadata: Mapping[str, Any]) -> H264Packet:
        if not isinstance(data, bytes):
            raise ValueError("H.264 packet payload must be bytes")
        codec = _metadata_str(metadata, "codec", H264_CODEC)
        bitstream = _metadata_str(metadata, "bitstream", H264_BITSTREAM)
        if codec != H264_CODEC:
            raise ValueError(f"Expected codec={H264_CODEC!r}, got {metadata.get('codec')!r}")
        if bitstream != H264_BITSTREAM:
            raise ValueError(
                f"Expected bitstream={H264_BITSTREAM!r}, got {metadata.get('bitstream')!r}"
            )
        for key in ("seq", "is_keyframe", "keyframe_seq", "pts", "width", "height"):
            if key not in metadata:
                raise ValueError(f"H.264 packet missing metadata field {key!r}")
        if not data:
            raise ValueError("H.264 packet payload cannot be empty")
        seq = _metadata_int(metadata, "seq", minimum=0)
        pts = _metadata_int(metadata, "pts", minimum=0)
        keyframe_seq = _metadata_int(metadata, "keyframe_seq", minimum=0)
        width = _metadata_int(metadata, "width", minimum=1)
        height = _metadata_int(metadata, "height", minimum=1)
        channels = _metadata_int(metadata, "channels", default=3, minimum=1)
        is_keyframe = _metadata_bool(metadata, "is_keyframe")
        try:
            image_format = ImageFormat(_metadata_str(metadata, "format", ImageFormat.RGB.value))
        except ValueError as exc:
            raise ValueError(
                f"Invalid H.264 packet image format: {metadata.get('format')!r}"
            ) from exc
        return cls(
            data=data,
            format=image_format,
            frame_id=_metadata_str(metadata, "frame_id", ""),
            ts=_metadata_float(metadata, "ts", default=0.0),
            seq=seq,
            pts=pts,
            is_keyframe=is_keyframe,
            keyframe_seq=keyframe_seq,
            width=width,
            height=height,
            channels=channels,
            dtype=_metadata_str(metadata, "dtype", "uint8"),
            codec=codec,
            bitstream=bitstream,
        )


def _metadata_bool(metadata: Mapping[str, Any], key: str) -> bool:
    value = metadata[key]
    if not isinstance(value, bool):
        raise ValueError(f"H.264 packet metadata field {key!r} must be a boolean")
    return value


def _metadata_int(
    metadata: Mapping[str, Any],
    key: str,
    *,
    default: int | None = None,
    minimum: int | None = None,
) -> int:
    value = metadata.get(key, default)
    if not isinstance(value, int) or isinstance(value, bool):
        raise ValueError(f"H.264 packet metadata field {key!r} must be an integer")
    if minimum is not None and value < minimum:
        raise ValueError(f"H.264 packet metadata field {key!r} must be >= {minimum}")
    return value


def _metadata_float(metadata: Mapping[str, Any], key: str, *, default: float) -> float:
    value = metadata.get(key, default)
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise ValueError(f"H.264 packet metadata field {key!r} must be numeric")
    return float(value)


def _metadata_str(metadata: Mapping[str, Any], key: str, default: str) -> str:
    value = metadata.get(key, default)
    if not isinstance(value, str):
        raise ValueError(f"H.264 packet metadata field {key!r} must be a string")
    return value


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


def h264_metadata(packet: H264Packet) -> dict[str, Any]:
    """Return validated metadata from an internal H.264 packet."""

    metadata = packet.metadata()
    H264Packet.from_parts(data=packet.data, metadata=metadata)
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

    def decode_packet(self, packet: H264Packet) -> Image:
        metadata = h264_metadata(packet)
        frame = self._jitter_frame_type(data=packet.data, timestamp=int(metadata["pts"]))
        decoded_frames = self._decoder.decode(frame)
        if not decoded_frames:
            raise VideoDecodeGapError("H.264 decoder produced no frame")
        return self._from_video_frame(cast("av.VideoFrame", decoded_frames[0]), packet)

    def _to_video_frame(self, image: Image) -> av.VideoFrame:
        fmt = _av_input_format(image.format)
        frame = self._av.VideoFrame.from_ndarray(np.ascontiguousarray(image.data), format=fmt)
        frame.pts = self._frame_index
        frame.time_base = self._time_base
        self._frame_index += 1
        return frame

    @staticmethod
    def _from_video_frame(frame: av.VideoFrame, packet: H264Packet) -> Image:
        image_format = packet.format
        arr = frame.to_ndarray(format=_av_input_format(image_format))
        return Image(data=arr, format=image_format, frame_id=packet.frame_id, ts=packet.ts)


class H264Encoder:
    """Encode a normal DimOS Image stream into internal H.264 packets."""

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
        self._last_source_signature: tuple[ImageFormat, int, int, int, str] | None = None

    def encode(self, image: Image, *, force_keyframe: bool = False) -> H264Packet:
        ensure_supported_image(image, self.config)
        source_signature = self._source_signature(image)
        is_keyframe = self._should_force_keyframe(source_signature, force_keyframe)
        access_unit, pts = self._codec.encode_image(image, force_keyframe=is_keyframe)
        if is_keyframe:
            self._keyframe_seq = self._seq
        packet = H264Packet(
            data=access_unit,
            format=image.format,
            frame_id=image.frame_id,
            ts=image.ts,
            seq=self._seq,
            pts=pts,
            is_keyframe=is_keyframe,
            keyframe_seq=self._keyframe_seq,
            width=image.width,
            height=image.height,
            channels=image.channels,
            dtype=str(image.dtype),
        )
        self._last_source_signature = source_signature
        self._seq += 1
        return packet

    @staticmethod
    def _source_signature(image: Image) -> tuple[ImageFormat, int, int, int, str]:
        return (image.format, image.width, image.height, image.channels, str(image.dtype))

    def _should_force_keyframe(
        self,
        source_signature: tuple[ImageFormat, int, int, int, str],
        requested: bool,
    ) -> bool:
        if requested or self._seq == 0 or self._keyframe_seq < 0:
            return True
        if (
            self._last_source_signature is not None
            and source_signature != self._last_source_signature
        ):
            return True
        since_keyframe = self._seq - self._keyframe_seq
        return since_keyframe >= min(self.config.keyframe_interval, self.config.max_gop_frames)


class GopBuffer:
    """Track H.264 GOP validity across an encoded packet stream."""

    def __init__(self) -> None:
        self.expected_seq: int | None = None
        self.keyframe_seq: int | None = None
        self.valid = False

    def accept(self, packet: H264Packet) -> bool:
        """Return True when the encoded packet can be safely decoded."""

        seq = packet.seq
        keyframe_seq = packet.keyframe_seq
        is_keyframe = packet.is_keyframe

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
    """Decode internal H.264 packets into normal raw DimOS Images."""

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

    def decode(self, packet: H264Packet) -> Image:
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
    "H264Packet",
    "MissingVideoDependencyError",
    "UnsupportedVideoImageError",
    "VideoDecodeGapError",
    "ensure_supported_image",
    "h264_metadata",
]

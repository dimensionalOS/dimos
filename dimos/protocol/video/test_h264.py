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

import builtins
from dataclasses import dataclass
import json
import struct

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.protocol.video import h264 as h264_module
from dimos.protocol.video.h264 import (
    AiortcH264Codec,
    GopBuffer,
    H264AccessUnit,
    H264Config,
    H264Decoder,
    H264Encoder,
    H264Packet,
    MissingVideoDependencyError,
    UnsupportedVideoImageError,
    VideoDecodeGapError,
    h264_metadata,
)


@dataclass
class FakeCodec:
    encoded_force_keyframes: list[bool]
    decoded_sequences: list[int]

    def encode_image(self, image: Image, *, force_keyframe: bool) -> tuple[bytes, int]:
        self.encoded_force_keyframes.append(force_keyframe)
        if force_keyframe:
            return b"\x00\x00\x00\x01\x67sps\x00\x00\x00\x01\x68pps\x00\x00\x00\x01\x65idr", 90
        return b"\x00\x00\x00\x01\x41delta", 180

    def decode_packet(self, packet: H264Packet) -> Image:
        metadata = h264_metadata(packet)
        self.decoded_sequences.append(int(metadata["seq"]))
        return Image(
            data=np.zeros((packet.height, packet.width, 3), dtype=np.uint8),
            format=packet.format,
            frame_id=packet.frame_id,
            ts=packet.ts,
        )


def _image(format: ImageFormat = ImageFormat.RGB, dtype: np.dtype = np.dtype(np.uint8)) -> Image:
    return Image(
        data=np.zeros((4, 6, 3), dtype=dtype),
        format=format,
        frame_id="cam",
        ts=123.0,
    )


def _packet(seq: int, *, key: bool, keyframe_seq: int | None = None) -> H264Packet:
    return H264Packet(
        data=b"\x00\x00\x00\x01\x65" if key else b"\x00\x00\x00\x01\x41",
        format=ImageFormat.RGB,
        frame_id="cam",
        ts=123.0 + seq,
        seq=seq,
        is_keyframe=key,
        keyframe_seq=seq if key else (0 if keyframe_seq is None else keyframe_seq),
        pts=seq * 90,
        width=6,
        height=4,
        channels=3,
        dtype="uint8",
    )


def test_h264_packet_roundtrips_metadata_and_access_unit() -> None:
    packet = _packet(0, key=True)

    decoded = H264Packet.from_bytes(packet.to_bytes())

    assert decoded == packet
    assert decoded.codec == "h264"
    assert decoded.bitstream == "annex_b"
    assert isinstance(decoded.data, bytes)
    assert decoded.data.startswith(b"\x00\x00\x00\x01")


def test_h264_packet_rejects_non_object_metadata() -> None:
    header = json.dumps(["not", "an", "object"]).encode("utf-8")
    payload = h264_module._H264_PACKET_MAGIC + struct.pack(">I", len(header)) + header + b"data"

    with pytest.raises(ValueError, match="must be a JSON object"):
        H264Packet.from_bytes(payload)


def test_h264_packet_rejects_non_boolean_keyframe_metadata() -> None:
    metadata = _packet(0, key=True).metadata()
    metadata["is_keyframe"] = "false"

    with pytest.raises(ValueError, match="is_keyframe.*boolean"):
        H264Packet.from_parts(data=b"\x00\x00\x00\x01\x65", metadata=metadata)


def test_h264_packet_rejects_invalid_dimensions() -> None:
    metadata = _packet(0, key=True).metadata()
    metadata["width"] = 0

    with pytest.raises(ValueError, match="width.*>= 1"):
        H264Packet.from_parts(data=b"\x00\x00\x00\x01\x65", metadata=metadata)


def test_h264_packet_rejects_non_bytes_payload() -> None:
    metadata = _packet(0, key=True).metadata()

    with pytest.raises(ValueError, match="payload must be bytes"):
        H264Packet.from_parts(data="not-bytes", metadata=metadata)  # type: ignore[arg-type]


def test_image_remains_raw_only_public_type() -> None:
    image = _image()

    assert isinstance(image.data, np.ndarray)
    assert not hasattr(image, "encoding")
    assert not hasattr(image, "codec_metadata")
    assert not hasattr(Image, "encoded")


def test_access_unit_assembles_depayloaded_annex_b_fragments() -> None:
    unit = H264AccessUnit.from_rtp_payloads(
        [b"payload-a", b"payload-b"],
        lambda payload: b"\x00\x00\x00\x01" + payload,
    )

    assert unit.data == b"\x00\x00\x00\x01payload-a\x00\x00\x00\x01payload-b"


def test_encoder_emits_packet_metadata_and_periodic_keyframes() -> None:
    codec = FakeCodec(encoded_force_keyframes=[], decoded_sequences=[])
    encoder = H264Encoder(H264Config(keyframe_interval=2, max_gop_frames=2), codec=codec)

    p0 = encoder.encode(_image())
    p1 = encoder.encode(_image())
    p2 = encoder.encode(_image())

    assert [p0.seq, p1.seq, p2.seq] == [
        0,
        1,
        2,
    ]
    assert [p0.is_keyframe, p1.is_keyframe, p2.is_keyframe] == [True, False, True]
    assert [p0.keyframe_seq, p1.keyframe_seq, p2.keyframe_seq] == [0, 0, 2]
    assert codec.encoded_force_keyframes == [True, False, True]
    assert isinstance(p0.data, bytes)
    assert b"\x67" in p0.data and b"\x68" in p0.data


def test_encoder_forces_keyframe_when_source_shape_changes() -> None:
    codec = FakeCodec(encoded_force_keyframes=[], decoded_sequences=[])
    encoder = H264Encoder(H264Config(keyframe_interval=30, max_gop_frames=30), codec=codec)
    changed_shape = Image(
        data=np.zeros((8, 6, 3), dtype=np.uint8),
        format=ImageFormat.RGB,
        frame_id="cam",
        ts=124.0,
    )

    p0 = encoder.encode(_image())
    p1 = encoder.encode(_image())
    p2 = encoder.encode(changed_shape)

    assert [p0.is_keyframe, p1.is_keyframe, p2.is_keyframe] == [True, False, True]
    assert codec.encoded_force_keyframes == [True, False, True]


def test_encoder_forces_keyframe_when_source_format_changes() -> None:
    codec = FakeCodec(encoded_force_keyframes=[], decoded_sequences=[])
    encoder = H264Encoder(H264Config(keyframe_interval=30, max_gop_frames=30), codec=codec)
    changed_format = Image(
        data=np.zeros((4, 6, 3), dtype=np.uint8),
        format=ImageFormat.BGR,
        frame_id="cam",
        ts=124.0,
    )

    p0 = encoder.encode(_image())
    p1 = encoder.encode(_image())
    p2 = encoder.encode(changed_format)

    assert [p0.is_keyframe, p1.is_keyframe, p2.is_keyframe] == [True, False, True]
    assert codec.encoded_force_keyframes == [True, False, True]


def test_gop_buffer_suppresses_delta_after_sequence_gap_until_keyframe() -> None:
    codec = FakeCodec(encoded_force_keyframes=[], decoded_sequences=[])
    decoder = H264Decoder(codec=codec, gop_buffer=GopBuffer())

    assert decoder.decode(_packet(0, key=True)).frame_id == "cam"
    assert decoder.decode(_packet(1, key=False, keyframe_seq=0)).frame_id == "cam"

    with pytest.raises(VideoDecodeGapError):
        decoder.decode(_packet(3, key=False, keyframe_seq=0))
    with pytest.raises(VideoDecodeGapError):
        decoder.decode(_packet(4, key=False, keyframe_seq=0))

    assert decoder.decode(_packet(5, key=True)).frame_id == "cam"
    assert codec.decoded_sequences == [0, 1, 5]


def test_unsupported_image_format_and_dtype_fail_explicitly() -> None:
    codec = FakeCodec(encoded_force_keyframes=[], decoded_sequences=[])
    encoder = H264Encoder(codec=codec)

    with pytest.raises(UnsupportedVideoImageError, match="RGBA"):
        encoder.encode(_image(ImageFormat.RGBA))
    with pytest.raises(UnsupportedVideoImageError, match="uint8"):
        encoder.encode(_image(dtype=np.dtype(np.uint16)))


def test_missing_aiortc_dependencies_raise_actionable_error(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    real_import = builtins.__import__

    def fake_import(name: str, *args: object, **kwargs: object) -> object:
        if name == "av" or name.startswith("aiortc"):
            raise ImportError(name)
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", fake_import)

    with pytest.raises(MissingVideoDependencyError, match="H.264 image mode requires"):
        AiortcH264Codec()

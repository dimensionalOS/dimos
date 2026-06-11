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

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.VideoPacket import VideoPacket
from dimos.protocol.video.h264 import (
    AiortcH264Codec,
    GopBuffer,
    H264AccessUnit,
    H264Config,
    H264Decoder,
    H264Encoder,
    MissingVideoDependencyError,
    UnsupportedVideoImageError,
    VideoDecodeGapError,
)


@dataclass
class FakeCodec:
    encoded_force_keyframes: list[bool]
    decoded_packets: list[int]

    def encode_image(self, image: Image, *, force_keyframe: bool) -> tuple[bytes, int]:
        self.encoded_force_keyframes.append(force_keyframe)
        if force_keyframe:
            return b"\x00\x00\x00\x01\x67sps\x00\x00\x00\x01\x68pps\x00\x00\x00\x01\x65idr", 90
        return b"\x00\x00\x00\x01\x41delta", 180

    def decode_packet(self, packet: VideoPacket) -> Image:
        self.decoded_packets.append(packet.seq)
        return Image(
            data=np.zeros((packet.height, packet.width, 3), dtype=np.uint8),
            format=ImageFormat(packet.format),
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


def _packet(seq: int, *, key: bool, keyframe_seq: int | None = None) -> VideoPacket:
    return VideoPacket(
        seq=seq,
        ts=123.0 + seq,
        frame_id="cam",
        width=6,
        height=4,
        format=ImageFormat.RGB.value,
        codec="h264",
        bitstream="annex_b",
        is_keyframe=key,
        keyframe_seq=seq if key else (0 if keyframe_seq is None else keyframe_seq),
        pts=seq * 90,
        data=b"\x00\x00\x00\x01\x65" if key else b"\x00\x00\x00\x01\x41",
    )


def test_video_packet_serializes_complete_access_unit() -> None:
    packet = _packet(0, key=True)

    decoded = VideoPacket.lcm_decode(packet.lcm_encode())

    assert decoded == packet
    assert decoded.codec == "h264"
    assert decoded.bitstream == "annex_b"
    assert decoded.data.startswith(b"\x00\x00\x00\x01")


def test_access_unit_assembles_depayloaded_annex_b_fragments() -> None:
    unit = H264AccessUnit.from_rtp_payloads(
        [b"payload-a", b"payload-b"],
        lambda payload: b"\x00\x00\x00\x01" + payload,
    )

    assert unit.data == b"\x00\x00\x00\x01payload-a\x00\x00\x00\x01payload-b"


def test_encoder_emits_keyframe_metadata_and_periodic_keyframes() -> None:
    codec = FakeCodec(encoded_force_keyframes=[], decoded_packets=[])
    encoder = H264Encoder(H264Config(keyframe_interval=2, max_gop_frames=2), codec=codec)

    p0 = encoder.encode(_image())
    p1 = encoder.encode(_image())
    p2 = encoder.encode(_image())

    assert [p0.seq, p1.seq, p2.seq] == [0, 1, 2]
    assert [p0.is_keyframe, p1.is_keyframe, p2.is_keyframe] == [True, False, True]
    assert [p0.keyframe_seq, p1.keyframe_seq, p2.keyframe_seq] == [0, 0, 2]
    assert codec.encoded_force_keyframes == [True, False, True]
    assert b"\x67" in p0.data and b"\x68" in p0.data


def test_gop_buffer_suppresses_delta_after_sequence_gap_until_keyframe() -> None:
    codec = FakeCodec(encoded_force_keyframes=[], decoded_packets=[])
    decoder = H264Decoder(codec=codec, gop_buffer=GopBuffer())

    assert decoder.decode(_packet(0, key=True)).frame_id == "cam"
    assert decoder.decode(_packet(1, key=False, keyframe_seq=0)).frame_id == "cam"

    with pytest.raises(VideoDecodeGapError):
        decoder.decode(_packet(3, key=False, keyframe_seq=0))
    with pytest.raises(VideoDecodeGapError):
        decoder.decode(_packet(4, key=False, keyframe_seq=0))

    assert decoder.decode(_packet(5, key=True)).frame_id == "cam"
    assert codec.decoded_packets == [0, 1, 5]


def test_unsupported_image_format_and_dtype_fail_explicitly() -> None:
    codec = FakeCodec(encoded_force_keyframes=[], decoded_packets=[])
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

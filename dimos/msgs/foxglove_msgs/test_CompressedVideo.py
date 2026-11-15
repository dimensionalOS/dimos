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

from fractions import Fraction
from functools import cache

import av
import numpy as np
import pytest
from pytest_mock import MockerFixture
import reactivex as rx
from reactivex.subject import Subject
import rerun as rr

from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo, _FrameDecoder, decode_video
from dimos.msgs.helpers import resolve_msg_type
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat

PACKET = b"\x00\x00\x00\x01\x65payload"


def test_lcm_encode_decode() -> None:
    original = CompressedVideo(data=PACKET, format="h264", frame_id="front_camera", ts=1234567890.5)
    decoded = CompressedVideo.lcm_decode(original.lcm_encode())

    assert decoded.data.tobytes() == PACKET
    assert decoded.format == "h264"
    assert decoded.frame_id == "front_camera"
    assert abs(decoded.ts - original.ts) < 1e-6


def test_resolves_over_the_raw_lcm_type() -> None:
    """The rerun bridge decodes by type name — it must land on the to_rerun() class."""
    resolved = resolve_msg_type("foxglove_msgs.CompressedVideo")
    assert resolved is CompressedVideo


def test_to_rerun_video_stream() -> None:
    stream = CompressedVideo(data=PACKET, format="h264").to_rerun()
    assert isinstance(stream, rr.VideoStream)
    assert bytes(stream.sample.as_arrow_array().to_pylist()[0]) == PACKET  # type: ignore[union-attr]


def test_to_rerun_unknown_codec() -> None:
    with pytest.raises(ValueError, match="mjpeg"):
        CompressedVideo(data=PACKET, format="mjpeg").to_rerun()


WIDTH, HEIGHT, BANDS = 160, 96, 8

ENCODERS = {
    "h264": ("libx264", {"preset": "ultrafast", "tune": "zerolatency"}),
    "h265": (
        "libx265",
        {"preset": "ultrafast", "tune": "zerolatency", "x265-params": "log-level=error"},
    ),
    "av1": (
        "libsvtav1",
        {"preset": "8", "crf": "10", "svtav1-params": "pred-struct=1:lookahead=0:scd=0"},
    ),
    "vp8": ("libvpx", {"deadline": "realtime", "lag-in-frames": "0"}),
    "vp9": ("libvpx-vp9", {"deadline": "realtime", "lag-in-frames": "0"}),
}


def _numbered_frame(number: int) -> np.ndarray:
    """A picture of ``number``: vertical band b is bright when bit b is set."""
    bands = np.array([220 if number >> b & 1 else 30 for b in range(BANDS)], np.uint8)
    row = np.repeat(bands, WIDTH // BANDS)
    pixels = np.broadcast_to(row[None, :, None], (HEIGHT, WIDTH, 3)).copy()
    pixels[:, :, 0] //= 2
    return pixels


def _number_of(image: Image) -> int:
    bands = image.data.reshape(HEIGHT, BANDS, WIDTH // BANDS, 3).mean(axis=(0, 2, 3))
    return sum(1 << b for b in range(BANDS) if bands[b] > 128)


@cache
def _stream(format: str, b_frames: int = 0) -> tuple[CompressedVideo, ...]:
    """24 numbered frames, a keyframe every 8, each packet stamped 1/32 s after
    the frame before it in capture order."""
    codec, options = ENCODERS[format]
    encoder = av.CodecContext.create(codec, "w")
    assert isinstance(encoder, av.VideoCodecContext)
    encoder.width, encoder.height, encoder.pix_fmt = WIDTH, HEIGHT, "yuv420p"
    encoder.time_base = Fraction(1, 32)
    encoder.framerate = Fraction(32, 1)
    encoder.gop_size = 8
    encoder.max_b_frames = b_frames
    encoder.options = options
    frames: list[av.VideoFrame | None] = []
    for number in range(24):
        frame = av.VideoFrame.from_ndarray(_numbered_frame(number), format="bgr24")
        frame.pts = number
        frames.append(frame)
    return tuple(
        CompressedVideo(bytes(packet), format, "camera_optical", ts=1000.0 + (packet.pts or 0) / 32)
        for frame in [*frames, None]
        for packet in encoder.encode(frame)
    )


def _decode(
    packets: tuple[CompressedVideo, ...], keyframes_only: bool = False, max_hz: float | None = None
) -> list[tuple[int, float]]:
    """The (frame number, timestamp) of every image ``decode_video`` publishes."""
    images: list[Image] = []
    rx.from_iterable(packets).pipe(
        decode_video(keyframes_only=keyframes_only, max_hz=max_hz)
    ).subscribe(images.append)
    return [(_number_of(image), image.ts) for image in images]


def _stamped(*numbers: int) -> list[tuple[int, float]]:
    return [(number, 1000.0 + number / 32) for number in numbers]


@pytest.mark.parametrize("format", ["h264", "h265", "av1", "vp8", "vp9"])
def test_decode_video_publishes_every_frame_with_its_packet_time(format: str) -> None:
    assert _decode(_stream(format)) == _stamped(*range(24))


def test_decode_video_image_fields() -> None:
    images: list[Image] = []
    rx.from_iterable(_stream("h264")).pipe(decode_video()).subscribe(images.append)

    assert images[0].data.shape == (HEIGHT, WIDTH, 3)
    assert images[0].format == ImageFormat.BGR
    assert images[0].frame_id == "camera_optical"


@pytest.mark.parametrize(
    "image_format, pixel, dtype, tolerance",
    [
        (ImageFormat.BGR, (15, 30, 30), "uint8", 4),
        (ImageFormat.RGB, (30, 30, 15), "uint8", 4),
        (ImageFormat.RGBA, (30, 30, 15, 255), "uint8", 4),
        (ImageFormat.BGRA, (15, 30, 30, 255), "uint8", 4),
        (ImageFormat.GRAY, 28, "uint8", 4),
        (ImageFormat.GRAY16, 28 * 256, "uint16", 4 * 256),
    ],
)
def test_decode_video_output_format(
    image_format: ImageFormat, pixel: tuple[int, ...] | int, dtype: str, tolerance: int
) -> None:
    images: list[Image] = []
    rx.from_iterable(_stream("h264", b_frames=2)).pipe(
        decode_video(image_format=image_format)
    ).subscribe(images.append)

    assert len(images) == 24
    assert all(image.format == image_format for image in images)
    assert images[0].data.dtype == np.dtype(dtype)
    expected = np.broadcast_to(pixel, (HEIGHT, WIDTH, *np.shape(pixel)))
    np.testing.assert_allclose(images[0].data, expected, atol=tolerance)
    assert [(image.frame_id, image.ts) for image in images] == [
        ("camera_optical", 1000.0 + number / 32) for number in range(24)
    ]


@pytest.mark.parametrize("image_format", [ImageFormat.DEPTH, ImageFormat.DEPTH16])
def test_decode_video_rejects_depth_output(image_format: ImageFormat) -> None:
    with pytest.raises(ValueError, match="unsupported video image format"):
        decode_video(image_format=image_format)


def test_decode_video_joined_mid_stream_starts_at_the_next_keyframe() -> None:
    assert _decode(_stream("h264")[3:]) == _stamped(*range(8, 24))


@pytest.mark.parametrize("format", ["h264", "h265", "av1", "vp8", "vp9"])
def test_decode_video_keyframes_only(format: str) -> None:
    assert _decode(_stream(format), keyframes_only=True) == _stamped(0, 8, 16)


def test_decode_video_max_hz_caps_published_frames() -> None:
    assert _decode(_stream("h264"), max_hz=4) == _stamped(0, 8, 16)


def test_decode_video_reopens_the_decoder_when_the_format_changes() -> None:
    packets = _stream("h264", b_frames=2) + _stream("av1")

    assert _decode(packets) == _stamped(*range(24), *range(24))


@pytest.mark.parametrize("max_hz, expected", [(None, range(24)), (4, range(0, 24, 8))])
def test_decode_video_reordered_frames_keep_their_capture_time(
    max_hz: float | None, expected: range
) -> None:
    packets = _stream("h264", b_frames=2)
    assert [packet.ts for packet in packets] != sorted(packet.ts for packet in packets)

    assert _decode(packets, max_hz=max_hz) == _stamped(*expected)


@pytest.mark.parametrize("max_hz", [None, 4])
def test_decode_video_skips_missing_timestamps_without_stopping_or_reordering(
    mocker: MockerFixture, max_hz: float | None
) -> None:
    frames = [
        av.VideoFrame.from_ndarray(_numbered_frame(number), format="bgr24") for number in range(5)
    ]
    frames[0].pts = 0
    frames[2].pts = 500_000_000
    frames[4].pts = 1_000_000_000
    context = mocker.Mock(spec=av.VideoCodecContext)
    context.decode.side_effect = [[], [], [frames[0]], [frames[1]], [frames[2]], frames[3:]]
    mocker.patch.object(_FrameDecoder, "_open", return_value=context)
    packets = tuple(CompressedVideo(PACKET, ts=number / 4) for number in range(5))

    assert _decode(packets, max_hz=max_hz) == [(0, 0.0), (2, 0.5), (4, 1.0)]


def test_decode_video_subscriptions_do_not_share_a_decoder() -> None:
    packets: Subject[CompressedVideo] = Subject()
    decoded = packets.pipe(decode_video(max_hz=4))
    first: list[int] = []
    second: list[int] = []
    decoded.subscribe(lambda image: first.append(_number_of(image)))
    decoded.subscribe(lambda image: second.append(_number_of(image)))
    for packet in _stream("h264"):
        packets.on_next(packet)

    assert (first, second) == ([0, 8, 16], [0, 8, 16])


def test_decode_video_unknown_format_errors_the_stream() -> None:
    errors: list[Exception] = []
    rx.of(CompressedVideo(PACKET, "mjpeg")).pipe(decode_video()).subscribe(on_error=errors.append)

    assert [str(error) for error in errors] == ["no video decoder for format 'mjpeg'"]

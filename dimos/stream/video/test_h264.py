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

import io

import numpy as np
import pytest
import reactivex as rx

from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.stream.video.h264 import (
    H264DecoderModule,
    H264InputMixin,
    h264_decode,
)

WIDTH, HEIGHT, FRAMES = 64, 64, 8


@pytest.fixture(scope="module")
def packets() -> list[CompressedVideo]:
    """A real, short H.264 stream — one keyframe then P-frames."""
    import av

    buf = io.BytesIO()
    container = av.open(buf, mode="w", format="h264")
    stream = container.add_stream("libx264", rate=30)
    stream.width, stream.height, stream.pix_fmt = WIDTH, HEIGHT, "yuv420p"
    rng = np.random.default_rng(0)
    base = rng.integers(0, 255, (HEIGHT, WIDTH, 3), dtype=np.uint8)

    out: list[CompressedVideo] = []
    ts = 0.0
    for i in range(FRAMES):
        frame = av.VideoFrame.from_ndarray(np.roll(base, i * 4, axis=1), format="rgb24")
        for pkt in stream.encode(frame):
            out.append(CompressedVideo(data=bytes(pkt), format="h264", frame_id="cam", ts=ts))
            ts += 1.0
    for pkt in stream.encode(None):
        out.append(CompressedVideo(data=bytes(pkt), format="h264", frame_id="cam", ts=ts))
        ts += 1.0
    container.close()
    return out


def _decode_all(source: list[CompressedVideo]) -> list[Image]:
    seen: list[Image] = []
    rx.from_iterable(source).pipe(h264_decode()).subscribe(seen.append)
    return seen


def test_operator_decodes_a_real_stream_to_bgr(packets: list[CompressedVideo]) -> None:
    images = _decode_all(packets)

    assert images, "no frames decoded"
    assert all(img.format is ImageFormat.BGR for img in images)
    assert images[0].as_numpy().shape == (HEIGHT, WIDTH, 3)


def test_operator_carries_frame_id_and_timestamp(packets: list[CompressedVideo]) -> None:
    images = _decode_all(packets)

    assert {img.frame_id for img in images} == {"cam"}
    # Stamps come from the packets, so they advance with the stream.
    stamps = [img.ts for img in images]
    assert stamps == sorted(stamps)


def test_operator_survives_joining_mid_gop(packets: list[CompressedVideo]) -> None:
    """P-frames before the first keyframe resolve to nothing, not an exception."""
    tail = packets[len(packets) // 2 :]

    images = _decode_all(tail)

    assert len(images) < len(tail)  # some packets produced no picture


def test_operator_state_is_per_subscription(packets: list[CompressedVideo]) -> None:
    """A second subscription starts its own decoder rather than inheriting one."""
    first = _decode_all(packets)
    second = _decode_all(packets)

    assert len(first) == len(second)


def test_garbage_packets_do_not_raise() -> None:
    junk = [CompressedVideo(data=b"\x00\x00\x00\x01\x65nonsense", format="h264", ts=1.0)]

    assert _decode_all(junk) == []


def test_decoder_module_exposes_video_in_and_image_out() -> None:
    module = H264DecoderModule()
    try:
        assert set(module.inputs) == {"video"}
        assert set(module.outputs) == {"color_image"}
    finally:
        module.stop()


def test_mixin_feeds_the_hosts_own_image_port() -> None:
    """The decoded frames land on the consumer's existing In, not a new port."""
    from dimos.perception.fiducial.marker_detection_stream_module import (
        VideoMarkerDetectionModule,
    )

    module = VideoMarkerDetectionModule(marker_length_m=0.18)
    try:
        assert {"video", "color_image"} <= set(module.inputs)
        assert module.color_image is module.inputs["color_image"]
        assert set(module.outputs) == {"detections"}
    finally:
        module.stop()


def test_a_host_can_point_the_mixin_at_a_differently_named_port() -> None:
    """color_image is the default, not a requirement — and hosts that rename it
    must not inherit a stray one."""

    class ThermalConsumer(Module):
        thermal_image: In[Image]
        detections: Out[Detection3DArray]

    class VideoThermal(H264InputMixin, ThermalConsumer):
        @property
        def image_in(self) -> In[Image]:
            return self.thermal_image

    module = VideoThermal()
    try:
        assert set(module.inputs) == {"thermal_image", "video"}
        assert module.image_in is module.thermal_image
    finally:
        module.stop()


def test_the_mixin_is_not_offered_as_a_deployable_module() -> None:
    """It subclasses Module to reach its host's attributes, but has no graph of
    its own to run in, so the registry must not list it."""
    from dimos.robot.all_blueprints import all_modules

    assert "h264-input-mixin" not in all_modules
    assert "h264-decoder-module" in all_modules

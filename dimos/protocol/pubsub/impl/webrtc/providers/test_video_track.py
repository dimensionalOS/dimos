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

"""CameraVideoTrack's frame conversion runs in aiortc's sender task, which
ends the track on an exception."""

from __future__ import annotations

import asyncio
from contextlib import ExitStack

from dimos_cdrgen.registry import decode, encode
import numpy as np
import pytest

from dimos.core.transport import WebRTCVideoTransport
from dimos.message_utils.image import image_from_array
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.sensor_msgs.msg import Image as CdrImage
from dimos.messages.std_msgs.msg import Header
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.teleop.hosted.camera_mux import CameraMuxModule

pytest.importorskip("aiortc")

from aiortc.codecs.h264 import H264Encoder

from dimos.protocol.pubsub.impl.webrtc.providers.video_track import CameraVideoTrack


async def test_16bit_frames_are_converted_to_rgb() -> None:
    track = CameraVideoTrack(asyncio.get_running_loop())
    track.arm()
    for fmt in (ImageFormat.GRAY16, ImageFormat.DEPTH16):
        track.set_latest(Image(data=np.full((4, 6), 0x8000, np.uint16), format=fmt))
        frame = await asyncio.wait_for(track.recv(), 2)
        assert (frame.format.name, frame.width, frame.height) == ("rgb24", 6, 4)
        assert frame.to_ndarray()[0, 0].tolist() == [128, 128, 128]
    track.set_latest(Image(data=np.zeros((4, 6, 3), np.uint8), format=ImageFormat.BGR))
    frame = await asyncio.wait_for(track.recv(), 2)
    assert frame.format.name == "bgr24"


async def test_odd_sized_frames_are_cropped_to_what_the_h264_encoder_opens() -> None:
    # Conversion alone passes an odd size; libx264 refuses it when the encoder
    # opens, inside aiortc's sender task, so the encoder itself runs here.
    track = CameraVideoTrack(asyncio.get_running_loop())
    track.arm()
    track.set_latest(Image(data=np.zeros((47, 63, 3), np.uint8), format=ImageFormat.RGB))
    frame = await asyncio.wait_for(track.recv(), 2)
    assert (frame.width, frame.height) == (62, 46)
    payloads, _timestamp = H264Encoder().encode(frame, force_keyframe=True)
    assert payloads


async def test_unconvertible_frames_are_skipped_not_raised() -> None:
    track = CameraVideoTrack(asyncio.get_running_loop())
    track.arm()
    pending = asyncio.ensure_future(track.recv())
    for bad in (
        Image(data=np.zeros((4, 6), np.float32), format=ImageFormat.DEPTH),  # no av format
        Image(data=np.zeros((4, 6, 3), np.uint16), format=ImageFormat.GRAY16),  # to_rgb() raises
    ):
        track.set_latest(bad)
        await asyncio.sleep(0.05)
        assert not pending.done()
    track.set_latest(Image(data=np.zeros((4, 6), np.uint8), format=ImageFormat.GRAY))
    frame = await asyncio.wait_for(pending, 2)
    assert frame.format.name == "gray"


@pytest.fixture
async def cdr_track():
    track = CameraVideoTrack(asyncio.get_running_loop())
    track.arm()
    yield track
    track.stop()


async def test_generated_16bit_and_padded_images_convert_without_changing_source(cdr_track):
    for encoding in ("mono16", "16UC1"):
        source = image_from_array(
            np.full((4, 6), 0x8000, np.uint16),
            encoding=encoding,
            header=Header(Time(1_789_654_321, 1), "camera"),
        )
        cdr_track.set_latest(source)
        frame = await asyncio.wait_for(cdr_track.recv(), 2)
        assert (frame.format.name, frame.width, frame.height) == ("rgb24", 6, 4)
        assert frame.to_ndarray()[0, 0].tolist() == [128, 128, 128]
        assert source.header == Header(Time(1_789_654_321, 1), "camera")
    pixels = np.array([255, 0, 0, 0, 255, 0, 9, 9] * 2, np.uint8)
    padded = CdrImage(Header(Time(0, 1), "camera"), 2, 2, "rgb8", 0, 8, pixels)
    cdr_track.set_latest(padded)
    frame = await asyncio.wait_for(cdr_track.recv(), 2)
    assert frame.to_ndarray(format="rgb24")[0].tolist() == [[255, 0, 0], [0, 255, 0]]
    np.testing.assert_array_equal(padded.data, pixels)


async def test_generated_odd_image_opens_real_h264_encoder(cdr_track):
    source = image_from_array(np.full((47, 63, 3), [255, 0, 0], np.uint8), encoding="rgb8")
    cdr_track.set_latest(source)
    frame = await asyncio.wait_for(cdr_track.recv(), 2)
    assert (frame.width, frame.height) == (62, 46)
    assert frame.to_ndarray(format="rgb24")[0, 0].tolist() == [255, 0, 0]
    payloads, _ = H264Encoder().encode(frame, force_keyframe=True)
    assert payloads
    assert (source.width, source.height) == (63, 47)


async def test_generated_bad_frames_leave_receiver_waiting_for_valid_image(cdr_track, mocker):
    converted = asyncio.Event()
    original = cdr_track._to_av

    def convert(image):
        result = original(image)
        converted.set()
        return result

    mocker.patch.object(cdr_track, "_to_av", side_effect=convert)
    pending = asyncio.create_task(cdr_track.recv())
    try:
        for bad in (
            image_from_array(np.zeros((4, 6), np.float32), encoding="32FC1"),
            CdrImage(Header(Time(0, 0), ""), 4, 6, "mono16", 0, 1, np.zeros(24, np.uint8)),
        ):
            cdr_track.set_latest(bad)
            await asyncio.wait_for(converted.wait(), 2)
            converted.clear()
            assert not pending.done()
        cdr_track.set_latest(image_from_array(np.zeros((4, 6), np.uint8), encoding="mono8"))
        frame = await asyncio.wait_for(pending, 2)
        assert frame.format.name == "gray"
    finally:
        pending.cancel()
        await asyncio.gather(pending, return_exceptions=True)


async def test_cdr_camera_mux_port_feeds_media_track_and_real_encoder(cdr_track, mocker):
    provider = mocker.Mock()
    provider.is_connected = True
    provider.set_video_frame.side_effect = cdr_track.set_latest
    config = mocker.Mock()
    config.provider.return_value = provider
    source = image_from_array(
        np.full((47, 63, 3), [255, 0, 0], np.uint8),
        encoding="rgb8",
        header=Header(Time(17, 1), "camera"),
    )
    with ExitStack() as resources:
        mux = CameraMuxModule(cameras=["cam1"])
        resources.callback(mux.stop)
        mux.set_transport("mux_image", WebRTCVideoTransport(config=config))
        mux._on_cam("cam1", decode(encode(source), CdrImage))
        frame = await asyncio.wait_for(cdr_track.recv(), 2)
        assert (frame.width, frame.height) == (62, 46)
        assert frame.to_ndarray(format="rgb24")[0, 0].tolist() == [255, 0, 0]
        sent = provider.set_video_frame.call_args.args[0]
        assert type(sent) is CdrImage
        assert sent.header == Header(Time(17, 1), "camera")
        provider.start.assert_not_called()
        payloads, _ = H264Encoder().encode(frame, force_keyframe=True)
        assert payloads
        assert source.width == 63 and source.height == 47

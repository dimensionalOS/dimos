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
from dataclasses import replace

import numpy as np
import pytest

from dimos.msgs.image import image_from_array

pytest.importorskip("aiortc")

from aiortc.codecs.h264 import H264Encoder

from dimos.protocol.pubsub.impl.webrtc.providers.video_track import CameraVideoTrack


async def test_16bit_frames_are_converted_to_rgb() -> None:
    track = CameraVideoTrack(asyncio.get_running_loop())
    track.arm()
    for encoding in ("mono16", "16UC1"):
        track.set_latest(image_from_array(np.full((4, 6), 0x8000, np.uint16), encoding=encoding))
        frame = await asyncio.wait_for(track.recv(), 2)
        assert (frame.format.name, frame.width, frame.height) == ("rgb24", 6, 4)
        assert frame.to_ndarray()[0, 0].tolist() == [128, 128, 128]
    track.set_latest(image_from_array(np.zeros((4, 6, 3), np.uint8), encoding="bgr8"))
    frame = await asyncio.wait_for(track.recv(), 2)
    assert frame.format.name == "bgr24"


async def test_odd_sized_frames_are_cropped_to_what_the_h264_encoder_opens() -> None:
    # Conversion alone passes an odd size; libx264 refuses it when the encoder
    # opens, inside aiortc's sender task, so the encoder itself runs here.
    track = CameraVideoTrack(asyncio.get_running_loop())
    track.arm()
    track.set_latest(image_from_array(np.zeros((47, 63, 3), np.uint8), encoding="rgb8"))
    frame = await asyncio.wait_for(track.recv(), 2)
    assert (frame.width, frame.height) == (62, 46)
    payloads, _timestamp = H264Encoder().encode(frame, force_keyframe=True)
    assert payloads


async def test_unconvertible_frames_are_skipped_not_raised() -> None:
    track = CameraVideoTrack(asyncio.get_running_loop())
    track.arm()
    pending = asyncio.ensure_future(track.recv())
    for bad in (
        image_from_array(np.zeros((4, 6), np.float32), encoding="32FC1"),  # no av format
        replace(
            image_from_array(np.zeros((4, 6), np.uint16), encoding="mono16"),
            step=1,  # malformed wire metadata must not end the sender task
        ),
    ):
        track.set_latest(bad)
        await asyncio.sleep(0.05)
        assert not pending.done()
    track.set_latest(image_from_array(np.zeros((4, 6), np.uint8), encoding="mono8"))
    frame = await asyncio.wait_for(pending, 2)
    assert frame.format.name == "gray"

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

# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

import asyncio

import numpy as np
import pytest

from dimos.msgs.image import image_from_array
from dimos.protocol.pubsub.impl.webrtc.providers.video_track import CameraVideoTrack


@pytest.mark.asyncio
@pytest.mark.parametrize(
    "encoding,channels", [("rgb8", 3), ("bgr8", 3), ("rgba8", 4), ("bgra8", 4), ("mono8", 1)]
)
async def test_video_track_uses_generated_encoding_without_network(encoding, channels):
    pixels = np.full((4, 4) if channels == 1 else (4, 4, channels), 127, dtype=np.uint8)
    image = image_from_array(pixels, encoding=encoding)
    original = image.encode()
    track = CameraVideoTrack(asyncio.get_running_loop())
    try:
        track.arm()
        track.set_latest(image)
        frame = await asyncio.wait_for(track.recv(), timeout=1)
        assert frame.width == 4 and frame.height == 4
        assert frame.pts == 0
        assert image.encode() == original
    finally:
        track.stop()

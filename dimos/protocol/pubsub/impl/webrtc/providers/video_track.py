# Copyright 2025-2026 Dimensional Inc.
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

"""aiortc video track sourced from an Image stream.

Extracted from ``hosted_teleop_module`` so the broker-handshake file stays
focused on session lifecycle rather than media plumbing.
"""

from __future__ import annotations

import asyncio
import time

from aiortc.mediastreams import VIDEO_CLOCK_RATE, VIDEO_TIME_BASE, VideoStreamTrack
import av

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat

_AV_FORMAT_MAP = {
    ImageFormat.BGR: "bgr24",
    ImageFormat.RGB: "rgb24",
    ImageFormat.BGRA: "bgra",
    ImageFormat.RGBA: "rgba",
    ImageFormat.GRAY: "gray",
}


class CameraVideoTrack(VideoStreamTrack):
    """aiortc video track sourced from the latest Image on the In port.

    Drain-mode (recv only returns on a NEW frame) + wall-clock PTSs — so the
    browser paces playback at the source's real cadence, not aiortc's 30fps
    schedule, and we don't feed duplicates at startup (would warm up the
    encoder and the browser would play the burst in fast-forward).
    """

    def __init__(self, loop: asyncio.AbstractEventLoop) -> None:
        super().__init__()
        self._loop: asyncio.AbstractEventLoop = loop
        self._latest: Image | None = None
        self._frame_seq = 0
        self._consumed_seq = 0
        self._armed = False
        self._first_mono: float | None = None
        self._new_frame = asyncio.Event()

    def arm(self) -> None:
        """Discard buffered frames; start delivering from now.

        Called on the event loop once the PC is ``connected`` so the operator's
        video starts at "this instant", not "whenever the robot booted".
        """
        self._consumed_seq = self._frame_seq
        self._armed = True

    def set_latest(self, img: Image) -> None:
        """Publish the latest frame. Called from the producer (stream) thread.

        aiortc / asyncio.Event aren't thread-safe, so marshal the swap +
        notification onto the loop instead of locking it from this thread.
        """

        def _set() -> None:
            self._latest = img
            self._frame_seq += 1
            self._new_frame.set()

        try:
            self._loop.call_soon_threadsafe(_set)
        except RuntimeError:
            return

    async def recv(self) -> av.VideoFrame:
        # Wait (no busy-poll) for a fresh, post-arm frame.
        while True:
            await self._new_frame.wait()
            self._new_frame.clear()
            if self._armed and self._latest is not None and self._frame_seq > self._consumed_seq:
                img = self._latest
                self._consumed_seq = self._frame_seq
                break

        # Monotonic (not wall) clock so PTS never goes backward on an NTP/clock
        # step — aiortc requires non-decreasing PTS.
        now = time.monotonic()
        if self._first_mono is None:
            self._first_mono = now
        pts = int((now - self._first_mono) * VIDEO_CLOCK_RATE)

        frame = av.VideoFrame.from_ndarray(img.data, format=_AV_FORMAT_MAP.get(img.format, "bgr24"))
        frame.pts = pts
        frame.time_base = VIDEO_TIME_BASE
        return frame

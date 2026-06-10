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
import threading
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

    def __init__(self) -> None:
        super().__init__()
        self._lock = threading.Lock()
        self._latest: Image | None = None
        self._frame_seq = 0
        self._consumed_seq = 0
        self._armed = False
        self._first_wall: float | None = None

    def arm(self) -> None:
        """Discard buffered frames; start delivering from now.

        Called once the PC is ``connected`` so the operator's video starts at
        "this instant", not "whenever the robot booted".
        """
        with self._lock:
            self._consumed_seq = self._frame_seq
            self._armed = True

    def set_latest(self, img: Image) -> None:
        with self._lock:
            self._latest = img
            self._frame_seq += 1

    async def recv(self) -> av.VideoFrame:
        while True:
            with self._lock:
                if (
                    self._armed
                    and self._latest is not None
                    and self._frame_seq > self._consumed_seq
                ):
                    img = self._latest
                    self._consumed_seq = self._frame_seq
                    break
            await asyncio.sleep(0.005)

        now = time.time()
        if self._first_wall is None:
            self._first_wall = now
        pts = int((now - self._first_wall) * VIDEO_CLOCK_RATE)

        frame = av.VideoFrame.from_ndarray(img.data, format=_AV_FORMAT_MAP.get(img.format, "bgr24"))
        frame.pts = pts
        frame.time_base = VIDEO_TIME_BASE
        return frame


__all__ = ["CameraVideoTrack"]

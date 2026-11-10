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

Kept separate from ``broker.py`` so the broker-handshake file stays focused
on session lifecycle rather than media plumbing.

Both image families are accepted while consumers migrate opt-in: legacy
``dimos.msgs.sensor_msgs.Image`` (``format``/``to_rgb()``) and generated
``dimos_messages.sensor_msgs.msg.Image`` (``encoding``/``message_utils``).
"""

from __future__ import annotations

import asyncio
import time
from typing import Any

from aiortc import RTCRtpSender
from aiortc.mediastreams import VIDEO_CLOCK_RATE, VIDEO_TIME_BASE, VideoStreamTrack
import av
import numpy as np

from dimos.message_utils import image
from dimos.msgs.sensor_msgs.Image import Image as LegacyImage, ImageFormat
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_AV_FORMAT_MAP = {
    "bgr8": "bgr24",
    "rgb8": "rgb24",
    "bgra8": "bgra",
    "rgba8": "rgba",
    "mono8": "gray",
}

_LEGACY_AV_FORMAT_MAP = {
    ImageFormat.BGR: "bgr24",
    ImageFormat.RGB: "rgb24",
    ImageFormat.BGRA: "bgra",
    ImageFormat.RGBA: "rgba",
    ImageFormat.GRAY: "gray",
}


def prefer_video_codec(pc: Any, codec: str) -> None:
    """Put `codec` (e.g. "h264") first in every video transceiver's codec
    preferences. Best-effort: an unknown codec keeps aiortc's default order,
    so a misconfigured knob cannot kill the connection."""
    want = f"video/{codec}".lower()
    caps = RTCRtpSender.getCapabilities("video")
    preferred = [c for c in caps.codecs if c.mimeType.lower() == want]
    if not preferred:
        logger.warning("video codec %r not in local capabilities; using defaults", codec)
        return
    rest = [c for c in caps.codecs if c.mimeType.lower() != want]
    for transceiver in pc.getTransceivers():
        if transceiver.kind == "video":
            transceiver.setCodecPreferences(preferred + rest)
            logger.info("video codec preference: %s first", want)


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
        self._latest: Any = None
        self._frame_seq = 0
        self._consumed_seq = 0
        self._armed = False
        self._first_mono: float | None = None
        self._new_frame = asyncio.Event()
        self._dropped: set[Any] = set()

    def arm(self) -> None:
        """Discard buffered frames; start delivering from now.

        Called on the event loop once the PC is ``connected`` so the operator's
        video starts at "this instant", not "whenever the robot booted".
        """
        self._consumed_seq = self._frame_seq
        self._armed = True

    def set_latest(self, img: Any) -> None:
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

    def _to_av(self, img: Any) -> av.VideoFrame | None:
        """None, logged once per source format, for what neither to_rgb() nor
        av takes (DEPTH float32, a malformed array). Never raises: recv() runs
        in aiortc's sender task, which ends the track on an exception."""
        if isinstance(img, LegacyImage):
            return self._legacy_to_av(img)
        try:
            if img.encoding in ("mono16", "16UC1"):
                gray = (image.view(img) / 256).astype(np.uint8)
                frame = image.from_array(
                    np.repeat(gray[:, :, None], 3, axis=2), encoding="rgb8", header=img.header
                )
            else:
                frame = (
                    img
                    if img.encoding in _AV_FORMAT_MAP
                    else image.from_array(image.to_rgb(img), encoding="rgb8", header=img.header)
                )
            data = image.view(frame)
            return self._frame(data, _AV_FORMAT_MAP[frame.encoding])
        except Exception as e:
            self._drop(img.encoding, img.data.dtype, e)
            return None

    def _legacy_to_av(self, img: LegacyImage) -> av.VideoFrame | None:
        try:
            frame = img if img.format in _LEGACY_AV_FORMAT_MAP else img.to_rgb()
            return self._frame(frame.data, _LEGACY_AV_FORMAT_MAP[frame.format])
        except Exception as e:
            self._drop(img.format, img.data.dtype, e)
            return None

    @staticmethod
    def _frame(data: Any, format: str) -> av.VideoFrame:
        h, w = data.shape[:2]
        if h % 2 or w % 2:
            # libx264 takes yuv420p, whose chroma planes are half-size: an
            # odd width or height fails avcodec_open2 in aiortc's sender
            # task, past this guard. The last row/column is cropped.
            data = np.ascontiguousarray(data[: h - h % 2, : w - w % 2])
        return av.VideoFrame.from_ndarray(data, format=format)

    def _drop(self, source_format: Any, dtype: Any, error: Exception) -> None:
        if source_format in self._dropped:
            return
        self._dropped.add(source_format)
        name = getattr(source_format, "value", source_format)
        logger.warning("video track: dropping %s/%s frames: %s", name, dtype, error)

    async def recv(self) -> av.VideoFrame:
        # Wait (no busy-poll) for a fresh, post-arm frame that converts.
        while True:
            await self._new_frame.wait()
            self._new_frame.clear()
            if self._armed and self._latest is not None and self._frame_seq > self._consumed_seq:
                img = self._latest
                self._consumed_seq = self._frame_seq
                frame = self._to_av(img)
                if frame is not None:
                    break

        # Monotonic (not wall) clock so PTS never goes backward on an NTP/clock
        # step — aiortc requires non-decreasing PTS.
        now = time.monotonic()
        if self._first_mono is None:
            self._first_mono = now
        pts = int((now - self._first_mono) * VIDEO_CLOCK_RATE)

        frame.pts = pts
        frame.time_base = VIDEO_TIME_BASE
        return frame

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

"""PCM → aiortc audio track for the Go2's speaker.

The dog's WebRTC connection negotiates ``m=audio sendrecv`` but never feeds the
outbound sender — this track fills that half. Push interleaved s16 PCM from any
thread; the dog-side PC pulls frames off ``recv()`` and Opus-encodes them.
Drain-mode (like ``CameraVideoTrack``): no frames pushed = nothing sent.
"""

from __future__ import annotations

import asyncio
import fractions
from typing import Any

from aiortc.mediastreams import MediaStreamTrack

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Bound the queue: if the dog-side sender stalls, drop oldest instead of
# building unbounded latency (~1s of 20ms frames).
_MAX_QUEUED_FRAMES = 50


class PCMAudioTrack(MediaStreamTrack):
    """Audio track fed by ``push(pcm, sample_rate, channels)`` calls."""

    kind = "audio"

    def __init__(self) -> None:
        super().__init__()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._queue: asyncio.Queue[tuple[bytes, int, int]] = asyncio.Queue(
            maxsize=_MAX_QUEUED_FRAMES
        )
        self._pts = 0

    def push(self, pcm: bytes, sample_rate: int, channels: int) -> None:
        """Queue one interleaved-s16 frame. Thread-safe; drops until recv()
        has bound the loop, and drops oldest when the queue is full."""
        loop = self._loop
        if loop is None or not loop.is_running():
            return

        def _put() -> None:
            if self._queue.full():
                try:
                    self._queue.get_nowait()  # drop oldest, keep the link live
                except asyncio.QueueEmpty:
                    pass
            self._queue.put_nowait((pcm, sample_rate, channels))

        loop.call_soon_threadsafe(_put)

    async def recv(self) -> Any:
        import av

        if self._loop is None:
            self._loop = asyncio.get_running_loop()

        # Skip malformed frames rather than raise out of recv() — aiortc treats
        # that as fatal and kills operator audio for the session.
        while True:
            pcm, sample_rate, channels = await self._queue.get()
            channels = max(1, int(channels))
            stride = 2 * channels
            samples = len(pcm) // stride
            if samples <= 0 or sample_rate <= 0:
                continue
            usable = samples * stride
            if usable != len(pcm):
                pcm = pcm[:usable]
            try:
                frame = av.AudioFrame(
                    format="s16",
                    layout="stereo" if channels == 2 else "mono",
                    samples=samples,
                )
                frame.planes[0].update(pcm)
            except Exception:
                logger.warning("speaker: dropping malformed audio frame", exc_info=True)
                continue
            frame.sample_rate = sample_rate
            frame.pts = self._pts
            frame.time_base = fractions.Fraction(1, sample_rate)
            self._pts += samples
            return frame

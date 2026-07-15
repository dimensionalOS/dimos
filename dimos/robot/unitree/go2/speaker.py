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
from typing import TYPE_CHECKING, Any

from aiortc.mediastreams import MediaStreamTrack

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.robot.unitree.connection import UnitreeWebRTCConnection

logger = setup_logger()

# Bound the queue: if the dog-side sender stalls, drop oldest instead of
# building unbounded latency (~1s of 20ms frames).
_MAX_QUEUED_FRAMES = 50


class PCMAudioTrack(MediaStreamTrack):
    """Audio track fed by ``push(pcm, sample_rate, channels)`` calls."""

    kind = "audio"

    def __init__(self, loop: asyncio.AbstractEventLoop) -> None:
        super().__init__()
        self._loop: asyncio.AbstractEventLoop = loop
        self._queue: asyncio.Queue[tuple[bytes, int, int]] = asyncio.Queue(
            maxsize=_MAX_QUEUED_FRAMES
        )
        self._pts = 0

    def push(self, pcm: bytes, sample_rate: int, channels: int) -> None:
        """Queue one interleaved-s16 frame; drops oldest when full. Thread-safe."""
        loop = self._loop

        def _put() -> None:
            if self._queue.full():
                try:
                    self._queue.get_nowait()  # drop oldest, keep the link live
                except asyncio.QueueEmpty:
                    pass
            self._queue.put_nowait((pcm, sample_rate, channels))

        try:
            loop.call_soon_threadsafe(_put)
        except RuntimeError:
            return

    async def recv(self) -> Any:
        import av

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


class Go2Speaker:
    """Operator mic → the dog's speaker: owns the track + its PC wiring.

    Keeps the WebRTC plumbing (reaching into the vendor driver's PC to swap the
    audio sender's track) out of GO2Connection. Wire it up with::

        speaker = Go2Speaker()
        set_audio_sink(speaker.push)      # provider → operator PCM frames
        speaker.attach(connection)        # PCM frames → the dog's speaker
        ...
        speaker.detach()
    """

    def __init__(self) -> None:
        self._track: PCMAudioTrack | None = None

    def push(self, pcm: bytes, sample_rate: int, channels: int) -> None:
        """Operator mic frame → the speaker track. No-op until attached."""
        track = self._track
        if track is not None:
            track.push(pcm, sample_rate, channels)

    def attach(self, connection: UnitreeWebRTCConnection) -> bool:
        """Feed the dog PC's negotiated audio m-line: replaceTrack + enable the
        audio channel. Best-effort — returns False if the vendor PC isn't up."""
        drv = connection.conn
        loop = connection.loop
        pc = getattr(drv, "pc", None)
        if pc is None:
            logger.debug("speaker: vendor PC not available — skipped")
            return False
        try:
            sender = next((t.sender for t in pc.getTransceivers() if t.kind == "audio"), None)
            if sender is None:
                logger.warning("speaker: dog PC has no audio transceiver")
                return False
            self._track = PCMAudioTrack(loop)
            loop.call_soon_threadsafe(sender.replaceTrack, self._track)
            loop.call_soon_threadsafe(drv.datachannel.switchAudioChannel, True)
            logger.debug("speaker: operator audio track attached")
            return True
        except Exception:
            self._track = None
            logger.exception("speaker attach failed — operator audio won't play on the dog")
            return False

    def detach(self) -> None:
        """Stop and drop the track so a stop()/start() cycle doesn't leak it."""
        track = self._track
        self._track = None
        if track is not None:
            try:
                track.stop()
            except Exception:
                logger.debug("speaker track stop failed", exc_info=True)

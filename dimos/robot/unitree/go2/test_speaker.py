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

"""Unit tests for PCMAudioTrack (dog-speaker audio path).

No WebRTC: the track is exercised directly — ``push()`` from the caller side,
``recv()`` inside a local event loop standing in for aiortc's sender."""

from __future__ import annotations

import asyncio

from dimos.robot.unitree.go2.speaker import _MAX_QUEUED_FRAMES, PCMAudioTrack

# 20 ms of 48 kHz mono s16.
_SR = 48_000
_FRAME = b"\x01\x02" * 960


def test_push_before_loop_is_dropped() -> None:
    """Frames pushed before recv() has bound a loop must be silently dropped
    (drain mode: an idle/unattached track costs nothing and never buffers)."""
    track = PCMAudioTrack()
    track.push(_FRAME, _SR, 1)
    assert track._queue.empty()


def test_recv_yields_frame_with_pts_progression() -> None:
    async def scenario() -> None:
        track = PCMAudioTrack()
        track._loop = asyncio.get_running_loop()
        track.push(_FRAME, _SR, 1)
        track.push(_FRAME, _SR, 1)
        await asyncio.sleep(0)  # let call_soon_threadsafe callbacks run

        f1 = await track.recv()
        f2 = await track.recv()
        assert f1.sample_rate == _SR
        assert f1.samples == 960
        assert f1.layout.name == "mono"
        # pts advances by the samples consumed — aiortc uses it for pacing.
        assert (f1.pts, f2.pts) == (0, 960)

    asyncio.run(scenario())


def test_stereo_layout_and_sample_count() -> None:
    async def scenario() -> None:
        track = PCMAudioTrack()
        track._loop = asyncio.get_running_loop()
        track.push(_FRAME, _SR, 2)  # same bytes, 2 channels → half the samples
        await asyncio.sleep(0)

        frame = await track.recv()
        assert frame.layout.name == "stereo"
        assert frame.samples == 480

    asyncio.run(scenario())


def test_ragged_pcm_is_trimmed_not_fatal() -> None:
    """A pcm buffer that isn't a whole number of interleaved s16 samples must
    be trimmed, not raise out of recv() (aiortc treats that as fatal and kills
    operator audio for the rest of the session)."""

    async def scenario() -> None:
        track = PCMAudioTrack()
        track._loop = asyncio.get_running_loop()
        track.push(_FRAME + b"\x07", _SR, 1)  # odd trailing byte
        await asyncio.sleep(0)

        frame = await track.recv()  # must not raise
        assert frame.samples == 960  # ragged tail dropped

    asyncio.run(scenario())


def test_unusable_frame_is_skipped() -> None:
    """An empty or zero-rate frame is skipped; recv() returns the next good
    frame instead of raising or yielding a degenerate frame."""

    async def scenario() -> None:
        track = PCMAudioTrack()
        track._loop = asyncio.get_running_loop()
        track.push(b"", _SR, 1)  # empty → skipped
        track.push(_FRAME, 0, 1)  # zero sample rate → skipped
        track.push(_FRAME, _SR, 1)  # good
        await asyncio.sleep(0)

        frame = await track.recv()
        assert frame.samples == 960
        assert frame.sample_rate == _SR

    asyncio.run(scenario())


def test_overflow_drops_oldest_keeps_link_live() -> None:
    """A stalled sender must bound latency: the queue caps at
    _MAX_QUEUED_FRAMES and evicts the oldest frame, not the newest."""

    async def scenario() -> None:
        track = PCMAudioTrack()
        track._loop = asyncio.get_running_loop()
        for i in range(_MAX_QUEUED_FRAMES + 3):
            track.push(bytes([i % 256]) * 4, _SR, 1)
        await asyncio.sleep(0)

        assert track._queue.qsize() == _MAX_QUEUED_FRAMES
        first, _, _ = track._queue.get_nowait()
        assert first == bytes([3]) * 4  # frames 0-2 were evicted

    asyncio.run(scenario())

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

"""Failure-path unit tests for RelayClient against a stub session.

No Deno, no network: a duck-typed session object exercises every client-side
failure path directly.
"""

import asyncio

import pytest

from dimos.web.relay_bridge.protocol import (
    DataFrame,
    FrameHeader,
    encode_data_frame,
)
from dimos.web.relay_bridge.wt_client import RelayClient


class StubSession:
    """Minimal SessionProtocol stand-in for the pure-Python client paths."""

    def __init__(self) -> None:
        self.frames: asyncio.Queue[DataFrame] = asyncio.Queue()
        self.closed = asyncio.Event()
        self.frames_dropped = 0
        self._next_id = 100

    def send_frame(self, header: FrameHeader, payload: bytes) -> int:
        # Real encode so a non-serializable meta raises here, exactly as the
        # aioquic session would.
        encode_data_frame(header, payload)
        self._next_id += 1
        return self._next_id

    def stream_in_flight(self, stream_id: int) -> bool:
        return False  # delivered instantly; the pump loops straight back

    def reset_if_in_flight(self, stream_id: int) -> bool:
        return False

    async def wait_closed(self) -> None:
        await self.closed.wait()


def _client(session: StubSession) -> RelayClient:
    return RelayClient("https://127.0.0.1:1", "robot", session, ctx=None)


def _frame(seq: int) -> DataFrame:
    return DataFrame(
        header=FrameHeader(ch="odom", seq=seq, ts=0.0, delivery="reliable"),
        payload=seq.to_bytes(4, "little"),
    )


async def test_pump_dies_visibly_on_encode_error() -> None:
    session = StubSession()
    writer = _client(session).latest_writer("cam")
    writer.offer(b"data", meta=object())  # object() is not JSON-serializable
    await asyncio.sleep(0.05)  # let the pump run and die
    assert writer._task.done()
    assert isinstance(writer._task.exception(), TypeError)
    # The dead channel is visible at the producer, not silently accepting.
    with pytest.raises(RuntimeError):
        writer.offer(b"more")


async def test_pump_stops_cleanly_on_session_close() -> None:
    session = StubSession()
    writer = _client(session).latest_writer("cam")
    session.closed.set()
    await asyncio.sleep(0.05)
    assert writer._task.done()
    assert writer._task.exception() is None  # a close is not an error
    with pytest.raises(RuntimeError):
        writer.offer(b"x")


async def test_offer_survives_delivery_and_keeps_sending() -> None:
    # A healthy pump keeps accepting; sanity check that the stub path works.
    session = StubSession()
    writer = _client(session).latest_writer("cam")
    for i in range(5):
        writer.offer(i.to_bytes(4, "little"))
        await asyncio.sleep(0)
    await asyncio.sleep(0.02)
    assert not writer._task.done()
    assert writer.sent >= 1
    writer.stop()


async def test_frames_cancel_does_not_steal_next_frame() -> None:
    session = StubSession()
    client = _client(session)

    async def consume_forever() -> None:
        async for _ in client.frames():
            pass

    consumer = asyncio.ensure_future(consume_forever())
    await asyncio.sleep(0.05)  # consumer now suspended waiting on an empty queue
    consumer.cancel()
    with pytest.raises(asyncio.CancelledError):
        await consumer

    # A frame delivered after the cancel must reach a fresh consumer, not be
    # swallowed by an orphaned queue getter.
    session.frames.put_nowait(_frame(0))
    got: DataFrame | None = None

    async def consume_one() -> None:
        nonlocal got
        async for frame in client.frames():
            got = frame
            break

    await asyncio.wait_for(consume_one(), timeout=1.0)
    assert got is not None and got.header.seq == 0


async def test_frames_drains_buffer_then_ends_on_close() -> None:
    session = StubSession()
    client = _client(session)
    session.frames.put_nowait(_frame(1))
    session.frames.put_nowait(_frame(2))
    session.closed.set()  # closed, but buffered frames must still drain

    seen = []
    async for frame in client.frames():
        seen.append(frame.header.seq)
    assert seen == [1, 2]
    assert client.is_closed

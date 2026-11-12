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

"""Ordered topic delivery uses real aioquic buffers/ACKs, with only UDP omitted."""

import asyncio
from collections.abc import AsyncIterator, Callable

from aioquic.quic.connection import QuicConnection
from aioquic.quic.packet_builder import QuicDeliveryState
import pytest
from pytest_mock import MockerFixture

from dimos.utils.asyncio_timeout import timeout
from dimos.web.relay_bridge._wt_session import SessionProtocol, make_quic_configuration
from dimos.web.relay_bridge.protocol import FrameHeader, decode_data_frame, encode_data_frame
from dimos.web.relay_bridge.topic_transport import (
    LatestTopicWriter,
    OrderedTopicWriter,
    TopicByteBudget,
)

ROUTE = "@t:ordered"
LATEST_ROUTE = "@t:latest"


def header(seq: int) -> FrameHeader:
    return FrameHeader(ch=ROUTE, seq=seq, ts=0, delivery="reliable")


def latest_header(seq: int, key: str = "demo/a") -> FrameHeader:
    return FrameHeader(ch=LATEST_ROUTE, seq=seq, ts=0, delivery="latest", meta={"key": key})


@pytest.fixture
async def transport(mocker: MockerFixture) -> AsyncIterator:
    session = SessionProtocol(QuicConnection(configuration=make_quic_configuration(insecure=True)))
    mocker.patch.object(session, "transmit")
    session.open_session("relay.test", "/robot")
    written: asyncio.Queue[tuple[int, bytes, int]] = asyncio.Queue()
    real_write = session.write_ordered_stream

    def write(stream_id: int, frame: bytes) -> int:
        end = real_write(stream_id, frame)
        written.put_nowait((stream_id, frame, end))
        return end

    mocker.patch.object(session, "write_ordered_stream", side_effect=write)
    try:
        yield session, written
    finally:
        session.close()


@pytest.fixture
async def writer(transport) -> AsyncIterator:
    session, _ = transport
    budget = TopicByteBudget(limit=3 * len(encode_data_frame(header(1), b"event")))
    errors: asyncio.Queue[str] = asyncio.Queue()
    writer = OrderedTopicWriter(session, ROUTE, budget, errors.put_nowait, max_frames=3)
    writer.start()
    try:
        yield writer, budget, errors
    finally:
        await writer.aclose()


async def test_events_pipeline_on_one_stream_and_remain_budgeted_until_acked(transport, writer):
    session, written = transport
    sender, budget, errors = writer
    for seq in range(1, 4):
        sender.offer(header(seq), b"event")
    async with timeout(1):
        sent = [await written.get() for _ in range(3)]

    assert [decode_data_frame(frame).header.seq for _, frame, _ in sent] == [1, 2, 3]
    assert len({stream for stream, _, _ in sent}) == 1
    assert sender.buffered_frames == 3
    assert budget.used == sum(len(frame) for _, frame, _ in sent)
    assert budget.peak == budget.used
    with pytest.raises(BufferError):
        sender.offer(header(4), b"event")

    stream_id, first, end = sent[0]
    quic_sender = session._quic._streams[stream_id].sender
    quic_sender.on_data_delivery(QuicDeliveryState.ACKED, 0, end, fin=False)
    session._wake_delivery_waiters()
    sender.offer(header(4), b"event")
    async with timeout(1):
        last_stream, last_frame, _ = await written.get()
    assert last_stream == stream_id
    assert decode_data_frame(last_frame).header.seq == 4
    assert budget.used == sum(len(frame) for _, frame, _ in sent) - len(first) + len(last_frame)
    assert errors.empty()
    peak = budget.peak
    await sender.aclose()
    assert budget.used == 0
    assert budget.peak == peak


async def test_unsubscribe_releases_quic_payload_before_reset_ack(transport, writer):
    session, written = transport
    sender, budget, _ = writer
    sender.offer(header(1), b"event")
    async with timeout(1):
        stream_id, _, _ = await written.get()
    await sender.aclose()
    assert budget.used == 0
    assert bytes(session._quic._streams[stream_id].sender._buffer) == b""
    with pytest.raises(ConnectionError):
        sender.offer(header(2), b"event")


async def test_session_loss_wakes_writer_and_reports_interruption(transport, writer):
    session, written = transport
    sender, budget, errors = writer
    sender.offer(header(1), b"event")
    async with timeout(1):
        await written.get()
        session.fail_session("test disconnect")
        reason = await errors.get()
    assert reason == "topic_stream_failed"
    assert budget.used == 0


async def test_aggregate_budget_bounds_two_routes_before_any_quic_write(transport):
    session, _ = transport
    frame_bytes = len(encode_data_frame(header(1), b"event"))
    budget = TopicByteBudget(limit=frame_bytes)
    errors: asyncio.Queue[str] = asyncio.Queue()
    first = OrderedTopicWriter(session, ROUTE, budget, errors.put_nowait)
    second = OrderedTopicWriter(session, "@t:another", budget, errors.put_nowait)
    first.start()
    second.start()
    try:
        first.offer(header(1), b"event")
        with pytest.raises(BufferError):
            second.offer(FrameHeader(ch="@t:another", seq=1, ts=0, delivery="reliable"), b"event")
        assert budget.used == frame_bytes
    finally:
        await first.aclose()
        await second.aclose()
    assert budget.used == 0


@pytest.fixture
async def latest(transport, mocker: MockerFixture) -> AsyncIterator:
    """A latest writer with every one-shot stream it sends recorded."""
    session, _ = transport
    sent: asyncio.Queue[tuple[int, bytes]] = asyncio.Queue()
    real_send = session.send_frame_bytes

    def send(frame: bytes) -> int:
        stream_id = real_send(frame)
        sent.put_nowait((stream_id, frame))
        return stream_id

    mocker.patch.object(session, "send_frame_bytes", side_effect=send)
    budget = TopicByteBudget(limit=4 * len(encode_data_frame(latest_header(1), b"sample")))
    errors: asyncio.Queue[str] = asyncio.Queue()
    writer = LatestTopicWriter(session, LATEST_ROUTE, budget, errors.put_nowait, stale_after=0.02)
    writer.start()
    try:
        yield writer, budget, errors, sent
    finally:
        await writer.aclose()


async def settled(predicate: Callable[[], bool]) -> None:
    async with timeout(1):
        while not predicate():
            await asyncio.sleep(0.001)


def ack_whole_stream(session: SessionProtocol, stream_id: int) -> None:
    quic_sender = session._quic._streams[stream_id].sender
    quic_sender.on_data_delivery(QuicDeliveryState.ACKED, 0, quic_sender._buffer_fin, fin=True)
    session._wake_delivery_waiters()


async def test_latest_sample_stays_budgeted_until_its_stream_is_acked(transport, latest):
    session, _ = transport
    writer, budget, errors, sent = latest
    writer.offer(latest_header(1), b"sample")
    async with timeout(1):
        stream_id, frame = await sent.get()
    assert decode_data_frame(frame).header.seq == 1
    assert session.stream_in_flight(stream_id)
    assert budget.used == len(frame)

    ack_whole_stream(session, stream_id)
    await settled(lambda: budget.used == 0)
    assert not session.stream_in_flight(stream_id)
    assert budget.peak == len(frame)
    assert errors.empty()


async def test_latest_stream_expires_and_releases_its_bytes_without_a_new_sample(transport, latest):
    session, _ = transport
    writer, budget, _, sent = latest
    writer.offer(latest_header(1), b"sample")
    async with timeout(1):
        stream_id, _ = await sent.get()

    # Nothing is ACKed and nothing else is offered: the stale timer alone
    # resets the stream and gives the bytes back.
    await settled(lambda: budget.used == 0)
    assert bytes(session._quic._streams[stream_id].sender._buffer) == b""
    assert sent.empty()


async def test_latest_replacement_before_send_releases_the_pending_sample(transport, latest):
    writer, budget, _, sent = latest
    writer.offer(latest_header(1), b"first")
    writer.offer(latest_header(2), b"second")
    assert writer.dropped == 1
    assert budget.used == len(encode_data_frame(latest_header(2), b"second"))

    async with timeout(1):
        _, frame = await sent.get()
    assert decode_data_frame(frame).header.seq == 2
    await asyncio.sleep(0.005)
    assert sent.empty()


async def test_latest_shutdown_resets_in_flight_streams_and_frees_the_budget(transport, latest):
    session, _ = transport
    writer, budget, _, sent = latest
    writer.offer(latest_header(1), b"sample")
    writer.offer(latest_header(1, key="demo/b"), b"sample")
    async with timeout(1):
        streams = [(await sent.get())[0] for _ in range(2)]
    assert budget.used > 0

    await writer.aclose()
    assert budget.used == 0
    for stream_id in streams:
        assert bytes(session._quic._streams[stream_id].sender._buffer) == b""
    with pytest.raises(ConnectionError):
        writer.offer(latest_header(2), b"sample")

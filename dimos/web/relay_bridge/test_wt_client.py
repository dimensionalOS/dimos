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
from collections.abc import AsyncIterator, Iterator
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading
import time
from typing import Any

from aioquic.quic.connection import QuicConnection
from aioquic.quic.packet_builder import QuicDeliveryState
import pytest
from pytest_mock import MockerFixture

from dimos.web.relay_bridge import wt_client
from dimos.web.relay_bridge._wt_session import SessionProtocol, make_quic_configuration
from dimos.web.relay_bridge.protocol import (
    CONTROL_CHANNEL,
    MAX_CONTROL_PAYLOAD_BYTES,
    MAX_TOKEN_LEN,
    PROTOCOL_VERSION,
    DataFrame,
    Error,
    FrameHeader,
    Hello,
    Msg,
    Ping,
    Pong,
    ProtocolError,
    Pub,
    PubAck,
    PubNack,
    RobotInfo,
    RobotManifest,
    Role,
    Sub,
    Subs,
    decode_datagram,
    encode_data_frame,
    encode_datagram,
)
from dimos.web.relay_bridge.wt_client import (
    LatestChannelWriter,
    RelayClient,
    RelayInfo,
    RelayRejectedError,
    fetch_relay_info,
)


class StubSession:
    """Minimal SessionProtocol stand-in for the pure-Python client paths."""

    def __init__(self) -> None:
        self.frames: asyncio.Queue[DataFrame] = asyncio.Queue()
        self.control_msgs: asyncio.Queue[Msg] = asyncio.Queue()
        self.closed = asyncio.Event()
        self.welcomed = asyncio.Event()
        self.relay_error: Error | None = None
        self.sent_msgs: list[Msg] = []
        self.sent_frames: list[tuple[FrameHeader, bytes]] = []
        self.resets: list[int] = []
        # Stream ids reported as still in flight (empty = instant delivery).
        self.in_flight: set[int] = set()
        self.frames_dropped = 0
        self.control_dropped = 0
        self._next_id = 100
        self.answer_pings = True
        self.pong_waiters: dict[int, asyncio.Future[Pong]] = {}
        self.failed: str | None = None

    def send_frame(self, header: FrameHeader, payload: bytes) -> int:
        # Real encode so a non-serializable meta raises here, exactly as the
        # aioquic session would.
        encode_data_frame(header, payload)
        self._next_id += 1
        self.sent_frames.append((header, payload))
        return self._next_id

    def stream_in_flight(self, stream_id: int) -> bool:
        return stream_id in self.in_flight

    def reset_if_in_flight(self, stream_id: int) -> bool:
        self.resets.append(stream_id)
        self.in_flight.discard(stream_id)
        return True

    async def wait_closed(self) -> None:
        await self.closed.wait()

    def send_msg(self, msg: Msg) -> None:
        self.sent_msgs.append(msg)
        if self.answer_pings and isinstance(msg, Ping):
            waiter = self.pong_waiters.get(msg.n)
            if waiter is not None:
                waiter.set_result(Pong(n=msg.n, ts=msg.ts))

    def register_pong_waiter(self, n: int) -> asyncio.Future[Pong]:
        waiter: asyncio.Future[Pong] = asyncio.get_running_loop().create_future()
        self.pong_waiters[n] = waiter
        return waiter

    def discard_pong_waiter(self, n: int) -> None:
        self.pong_waiters.pop(n, None)

    def fail_session(self, reason: str) -> None:
        self.failed = reason
        self.closed.set()

    def transmit(self) -> None:
        pass


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
    writer.offer(b"data", meta=object())  # not a dict: header validation rejects it
    with pytest.raises(ValueError):  # pydantic ValidationError
        await asyncio.wait_for(writer._task, timeout=5)
    # The dead channel is visible at the producer, not silently accepting.
    with pytest.raises(RuntimeError):
        writer.offer(b"more")


async def test_pump_stops_cleanly_on_session_close() -> None:
    session = StubSession()
    writer = _client(session).latest_writer("cam")
    session.closed.set()
    await asyncio.wait_for(writer._task, timeout=5)  # a close is not an error
    with pytest.raises(RuntimeError):
        writer.offer(b"x")


async def test_offer_survives_delivery_and_keeps_sending() -> None:
    # A healthy pump keeps accepting; sanity check that the stub path works.
    session = StubSession()
    writer = _client(session).latest_writer("cam")
    try:
        for i in range(5):
            writer.offer(i.to_bytes(4, "little"))
            await asyncio.sleep(0)
        await asyncio.sleep(0.02)
        assert not writer._task.done()
        assert writer.sent >= 1
    finally:
        writer.stop()


@pytest.fixture
async def delivery_session(mocker: MockerFixture) -> SessionProtocol:
    session = SessionProtocol(QuicConnection(configuration=make_quic_configuration(insecure=True)))
    # Keep real stream buffers and delivery notifications; only omit UDP I/O.
    mocker.patch.object(session, "transmit")
    session.open_session("relay.test", "/robot")
    return session


@pytest.fixture
async def delivery_writer(
    delivery_session: SessionProtocol, mocker: MockerFixture
) -> AsyncIterator[
    tuple[LatestChannelWriter, asyncio.Queue[tuple[int, bytes]], asyncio.Queue[None]]
]:
    sent: asyncio.Queue[tuple[int, bytes]] = asyncio.Queue()
    waiting: asyncio.Queue[None] = asyncio.Queue()
    real_send = delivery_session.send_frame
    real_wait = delivery_session.wait_delivery_progress

    def send_frame(header: FrameHeader, payload: bytes) -> int:
        stream_id = real_send(header, payload)
        sent.put_nowait((stream_id, payload))
        return stream_id

    async def wait_progress(timeout: float | None) -> None:
        waiting.put_nowait(None)
        await real_wait(timeout)

    mocker.patch.object(delivery_session, "send_frame", side_effect=send_frame)
    mocker.patch.object(delivery_session, "wait_delivery_progress", side_effect=wait_progress)
    client = RelayClient("https://relay.test", "robot", delivery_session, ctx=None)
    writer = client.latest_writer("cam", stale_after=10.0)
    try:
        yield writer, sent, waiting
    finally:
        writer.stop()
        await asyncio.gather(writer._task, return_exceptions=True)


async def test_mailbox_sends_only_newest_frame_on_delivery_notification(
    delivery_session: SessionProtocol, delivery_writer
) -> None:
    writer, sent, waiting = delivery_writer
    writer.offer(b"f0")
    stream_id, payload = await asyncio.wait_for(sent.get(), timeout=1)
    assert payload == b"f0"
    await asyncio.wait_for(waiting.get(), timeout=1)
    for i in range(1, 6):
        writer.offer(b"f%d" % i)

    sender = delivery_session._quic._streams[stream_id].sender
    sender.on_data_delivery(QuicDeliveryState.ACKED, 0, sender._buffer_stop, fin=True)
    delivery_session._wake_delivery_waiters()
    _, payload = await asyncio.wait_for(sent.get(), timeout=1)
    assert payload == b"f5"
    assert writer.sent == 2
    assert writer.dropped == 4
    assert writer.resets == 0


@pytest.mark.parametrize("stale_after", [0.0, 0.02])
async def test_replacement_wakes_stalled_sender_and_preserves_deadline(
    delivery_writer, stale_after: float
) -> None:
    writer, sent, waiting = delivery_writer
    writer.stale_after = stale_after
    started = time.monotonic()
    writer.offer(b"old")
    await asyncio.wait_for(sent.get(), timeout=1)
    await asyncio.wait_for(waiting.get(), timeout=1)
    # Even an expired frame stays in flight until there is a replacement.
    assert writer.resets == 0
    writer.offer(b"new")
    _, payload = await asyncio.wait_for(sent.get(), timeout=1)
    assert payload == b"new"
    assert writer.resets == 1
    assert time.monotonic() - started >= stale_after


async def test_session_failure_wakes_delivery_waiters_and_stops_sender(
    delivery_session: SessionProtocol, delivery_writer
) -> None:
    writer, sent, waiting = delivery_writer
    writer.offer(b"old")
    stream_id, _ = await asyncio.wait_for(sent.get(), timeout=1)
    await asyncio.wait_for(waiting.get(), timeout=1)
    waiter = asyncio.create_task(delivery_session.wait_delivered(stream_id, timeout=None))
    try:
        await asyncio.wait_for(waiting.get(), timeout=1)
        writer.offer(b"new")
        delivery_session.fail_session("test close")
        assert await asyncio.wait_for(waiter, timeout=1) is False
        await asyncio.wait_for(writer._task, timeout=1)
        assert writer.sent == 1
        assert writer.resets == 0
        assert delivery_session._delivery_waiters == []
    finally:
        waiter.cancel()
        await asyncio.gather(waiter, return_exceptions=True)


async def test_stopping_sender_cancels_delivery_waiter(
    delivery_session: SessionProtocol, delivery_writer
) -> None:
    writer, _, waiting = delivery_writer
    writer.offer(b"pending")
    await asyncio.wait_for(waiting.get(), timeout=1)
    writer.stop()
    with pytest.raises(asyncio.CancelledError):
        await asyncio.wait_for(writer._task, timeout=1)
    assert delivery_session._delivery_waiters == []


async def test_keepalive_fails_the_session_after_unanswered_pings(
    monkeypatch: pytest.MonkeyPatch, mocker: MockerFixture
) -> None:
    monkeypatch.setattr(wt_client, "KEEPALIVE_INTERVAL_S", 0.02)
    session = StubSession()
    transmit = mocker.patch.object(session, "transmit")
    sent_pings: asyncio.Queue[Ping] = asyncio.Queue()
    real_send = session.send_msg

    def send_msg(msg: Msg) -> None:
        real_send(msg)
        if isinstance(msg, Ping):
            sent_pings.put_nowait(msg)

    mocker.patch.object(session, "send_msg", side_effect=send_msg)
    client = _client(session)
    client.start_keepalive()
    client.start_keepalive()  # idempotent
    task = client._keepalive_task
    assert task is not None
    try:
        pings = [
            await asyncio.wait_for(sent_pings.get(), timeout=1.0)
            for _ in range(wt_client.KEEPALIVE_MISSES)
        ]
        assert session.failed is None  # answered: the session lives
        assert len({p.n for p in pings}) == len(pings)

        session.answer_pings = False
        await asyncio.wait_for(task, timeout=2.0)

        assert session.closed.is_set()
        assert session.failed == f"{wt_client.KEEPALIVE_MISSES} keepalive pings unanswered"
        assert session.pong_waiters == {}  # timed-out waiters are discarded
        transmit.assert_called_once_with()  # flush the close without an incoming datagram
    finally:
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)


async def test_keepalive_tolerates_a_lone_missed_pong(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(wt_client, "KEEPALIVE_INTERVAL_S", 0.02)
    session = StubSession()
    answered = 0

    real_send = session.send_msg

    def send_msg(msg: Msg) -> None:
        nonlocal answered
        if isinstance(msg, Ping):
            answered += 1
            if answered == 2:
                session.sent_msgs.append(msg)
                return  # one lost pong
        real_send(msg)

    session.send_msg = send_msg  # type: ignore[method-assign]
    client = _client(session)
    client.start_keepalive()
    await asyncio.sleep(0.2)
    assert session.failed is None
    assert answered >= 4
    client._keepalive_task.cancel()  # type: ignore[union-attr]


async def test_frames_cancel_does_not_steal_next_frame() -> None:
    session = StubSession()
    client = _client(session)

    async def consume_forever() -> None:
        async for _ in client.frames():
            pass

    consumer = asyncio.create_task(consume_forever())
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


async def test_control_messages_drain_then_end_on_close() -> None:
    session = StubSession()
    client = _client(session)
    session.control_msgs.put_nowait(Subs(chs=["cam"], n=1))
    session.control_msgs.put_nowait(Subs(chs=[], n=2))
    session.closed.set()  # closed, but buffered snapshots must still drain

    seen = [msg async for msg in client.control_messages()]
    assert seen == [Subs(chs=["cam"], n=1), Subs(chs=[], n=2)]


async def test_control_messages_wakes_on_late_push() -> None:
    session = StubSession()
    client = _client(session)

    async def push_later() -> None:
        await asyncio.sleep(0.02)
        session.control_msgs.put_nowait(Subs(chs=["odom"], n=7))
        await asyncio.sleep(0.02)
        session.closed.set()

    pusher = asyncio.ensure_future(push_later())
    seen = [msg async for msg in client.control_messages()]
    await pusher
    assert seen == [Subs(chs=["odom"], n=7)]


async def test_robot_hello_rides_control_frames_and_retries() -> None:
    # v5: the robot hello is an @control data frame on a fresh one-shot
    # stream per attempt, repeated until the welcome datagram lands.
    session = StubSession()
    client = _client(session)

    async def welcome_after_two() -> None:
        while len(session.sent_frames) < 2:
            await asyncio.sleep(0.005)
        session.welcomed.set()

    welcomer = asyncio.create_task(welcome_after_two())
    await asyncio.wait_for(
        client.hello(robot=RobotInfo(id="r1", name="r1", model="test")), timeout=5.0
    )
    await welcomer
    assert len(session.sent_frames) >= 2
    assert all(header.ch == CONTROL_CHANNEL for header, _ in session.sent_frames)
    msg = decode_datagram(session.sent_frames[0][1])
    assert msg == Hello(
        v=PROTOCOL_VERSION, role="robot", robot=RobotInfo(id="r1", name="r1", model="test")
    )
    assert session.sent_msgs == []  # nothing rode datagrams


async def test_robot_hello_resend_retires_in_flight_stream() -> None:
    # A relay that never ACKs: each resend must first reset its still-in-
    # flight predecessor, and the exit path must retire the final attempt, so
    # a failed hello leaves no stream outstanding. The timeout message names
    # the likely cause (protocol-version mismatch).
    session = StubSession()
    real_send = session.send_frame

    def send_frame(header: FrameHeader, payload: bytes) -> int:
        stream_id = real_send(header, payload)
        session.in_flight.add(stream_id)
        return stream_id

    session.send_frame = send_frame  # type: ignore[method-assign]
    client = _client(session)
    with pytest.raises(TimeoutError, match="protocol-version"):
        await client.hello(timeout=0.7, robot=RobotInfo(id="r1", name="r1", model="test"))
    n = len(session.sent_frames)
    assert n >= 2
    # Ids are sequential from 101; every stream was retired - predecessors at
    # resend time, the last one on exit.
    assert session.resets == list(range(101, 101 + n))
    assert session.in_flight == set()


async def test_robot_hello_oversized_control_payload_refused() -> None:
    # The relay caps @control payloads at MAX_CONTROL_PAYLOAD_BYTES; the
    # client must refuse locally, before any frame or datagram is queued.
    session = StubSession()
    client = _client(session)
    manifest = RobotManifest(pad="a" * MAX_CONTROL_PAYLOAD_BYTES)
    with pytest.raises(ProtocolError, match="trim the manifest"):
        await client.hello(robot=RobotInfo(id="r1", name="r1", model="test"), manifest=manifest)
    assert session.sent_frames == []
    assert session.sent_msgs == []


async def test_robot_hello_control_payload_boundary_is_exact() -> None:
    # Exactly MAX_CONTROL_PAYLOAD_BYTES sends; one byte more is refused.
    robot = RobotInfo(id="r1", name="r1", model="test")
    base = len(
        encode_datagram(Hello(v=PROTOCOL_VERSION, role="robot", robot=robot, manifest={"pad": ""}))
    )
    pad = MAX_CONTROL_PAYLOAD_BYTES - base

    at_cap = StubSession()
    with pytest.raises(TimeoutError):
        await _client(at_cap).hello(timeout=0.05, robot=robot, manifest={"pad": "a" * pad})
    assert len(at_cap.sent_frames[0][1]) == MAX_CONTROL_PAYLOAD_BYTES

    over_cap = StubSession()
    with pytest.raises(ProtocolError, match=str(MAX_CONTROL_PAYLOAD_BYTES)):
        await _client(over_cap).hello(timeout=0.05, robot=robot, manifest={"pad": "a" * (pad + 1)})
    assert over_cap.sent_frames == []


async def test_send_control_frame_uses_the_hello_framing() -> None:
    # Publish acks ride the same robot-opened one-shot @control path as
    # hello: a datagram-encoded payload in an @control data frame.
    session = StubSession()
    client = _client(session)
    ack = PubAck(id="p1", ch="human_input", relayTs=1.5, bridgeTs=2.5)
    stream_id = client.send_control_frame(ack)
    assert stream_id > 0
    ((header, payload),) = session.sent_frames
    assert header.ch == CONTROL_CHANNEL
    assert decode_datagram(payload) == ack
    assert session.sent_msgs == []  # nothing rode datagrams
    with pytest.raises(ProtocolError, match=str(MAX_CONTROL_PAYLOAD_BYTES)):
        client.send_control_frame(
            PubNack(id="p2", code="decode_failed", message="x" * MAX_CONTROL_PAYLOAD_BYTES)
        )
    assert len(session.sent_frames) == 1


async def test_send_control_refuses_an_unsendable_datagram() -> None:
    # aioquic retries an oversize datagram forever, wedging the whole queue;
    # the test viewer's control plane must refuse it locally instead.
    session = StubSession()
    client = _client(session)
    client.send_control(Sub(ch="odom"))
    assert len(session.sent_msgs) == 1
    with pytest.raises(ProtocolError, match="wedges aioquic"):
        client.send_control(Pub(id="a", ch="chat", data="x" * 2048))
    assert len(session.sent_msgs) == 1


async def test_robot_hello_cancellation_is_prompt_and_retires_stream() -> None:
    session = StubSession()
    real_send = session.send_frame

    def send_frame(header: FrameHeader, payload: bytes) -> int:
        stream_id = real_send(header, payload)
        session.in_flight.add(stream_id)
        return stream_id

    session.send_frame = send_frame  # type: ignore[method-assign]
    client = _client(session)
    task = asyncio.create_task(client.hello(robot=RobotInfo(id="r1", name="r1", model="test")))
    await asyncio.sleep(0.05)  # inside the resend loop
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await asyncio.wait_for(task, timeout=1.0)
    assert session.sent_frames  # at least one attempt went out before cancel
    # Cancellation may not orphan the outstanding hello stream: hello() can
    # run again on this live client.
    assert session.in_flight == set()


async def test_viewer_hello_rides_datagrams() -> None:
    session = StubSession()
    session.welcomed.set()
    client = RelayClient("https://127.0.0.1:1", "viewer", session, ctx=None)
    await client.hello()
    assert session.sent_msgs == [Hello(v=PROTOCOL_VERSION, role="viewer")]
    assert session.sent_frames == []


async def test_viewer_hello_carries_the_token() -> None:
    session = StubSession()
    session.welcomed.set()
    client = RelayClient("https://127.0.0.1:1", "viewer", session, ctx=None)
    await client.hello(token="viewer-token-0123456789abcdef")
    assert session.sent_msgs == [
        Hello(v=PROTOCOL_VERSION, role="viewer", token="viewer-token-0123456789abcdef")
    ]


async def test_overlong_token_is_refused_without_echoing_it() -> None:
    # Refused before the model validates: pydantic's message would quote the
    # secret, and that message ends up in logs.
    session = StubSession()
    client = RelayClient("https://127.0.0.1:1", "viewer", session, ctx=None)
    token = "s" * (MAX_TOKEN_LEN + 1)
    with pytest.raises(ProtocolError, match=f"{MAX_TOKEN_LEN + 1} characters") as exc_info:
        await client.hello(token=token)
    assert token not in str(exc_info.value)
    assert session.sent_msgs == []


async def test_viewer_hello_oversized_datagram_refused() -> None:
    # An unsendable datagram wedges aioquic's whole datagram queue, so an
    # oversized viewer hello is refused before anything is queued.
    session = StubSession()
    client = RelayClient("https://127.0.0.1:1", "viewer", session, ctx=None)
    with pytest.raises(ProtocolError, match="hello datagram"):
        await client.hello(manifest={"pad": "a" * 2000})
    assert session.sent_msgs == []


class FakeCtx:
    def __init__(self) -> None:
        self.exits = 0

    async def __aexit__(self, *exc: object) -> None:
        self.exits += 1
        await asyncio.sleep(0)


async def test_close_is_idempotent_and_concurrent_safe() -> None:
    # Supervisor, watchdog, and teardown can all close the same client; a
    # second __aexit__ entering the connect context manager while the first
    # is still awaiting inside it would raise. Only the first may run.
    ctx = FakeCtx()
    client = RelayClient("https://127.0.0.1:1", "robot", StubSession(), ctx=ctx)
    await asyncio.gather(client.close(), client.close())
    await client.close()
    assert ctx.exits == 1


class BlockingCtx:
    def __init__(self) -> None:
        self.entered = asyncio.Event()
        self.release = asyncio.Event()
        self.exits = 0

    async def __aexit__(self, *exc: object) -> None:
        self.exits += 1
        self.entered.set()
        await self.release.wait()


async def test_cancelled_close_waiter_does_not_cancel_shared_shutdown() -> None:
    ctx = BlockingCtx()
    client = RelayClient("https://127.0.0.1:1", "robot", StubSession(), ctx=ctx)

    first = asyncio.create_task(client.close())
    await asyncio.wait_for(ctx.entered.wait(), timeout=1.0)
    first.cancel()
    with pytest.raises(asyncio.CancelledError):
        await first

    second = asyncio.create_task(client.close())
    await asyncio.sleep(0)
    assert not second.done()

    ctx.release.set()
    await asyncio.wait_for(second, timeout=1.0)
    assert ctx.exits == 1


async def test_hello_rejection_preserves_error_code() -> None:
    session = StubSession()
    session.relay_error = Error(code="robot_id_conflict", message="already connected")
    session.welcomed.set()
    client = _client(session)

    with pytest.raises(RelayRejectedError) as exc_info:
        await client.hello(robot=RobotInfo(id="r1", name="r1", model="test"))

    assert exc_info.value.code == "robot_id_conflict"
    assert exc_info.value.message == "already connected"


@pytest.mark.parametrize(
    ("url", "role"),
    [
        ("https://127.0.0.1:1/viewer", "robot"),
        ("https://127.0.0.1:1/robot", "viewer"),
        ("https://127.0.0.1:1/unknown", "viewer"),
    ],
)
async def test_connect_rejects_wrong_endpoint_path(url: str, role: Role) -> None:
    with pytest.raises(ValueError, match="relay URL path"):
        await RelayClient.connect(url, role)


async def test_connect_handshake_timeout_is_bounded(monkeypatch: pytest.MonkeyPatch) -> None:
    class HangingConnect:
        async def __aenter__(self) -> None:
            await asyncio.Event().wait()

        async def __aexit__(self, *args: Any) -> None:
            raise AssertionError("a timed-out __aenter__ must not be exited twice")

    monkeypatch.setattr(wt_client, "_quic_connect", lambda *args, **kwargs: HangingConnect())
    with pytest.raises(asyncio.TimeoutError):
        await RelayClient.connect("https://127.0.0.1:1", "robot", timeout=0.01)


async def test_connect_defaults_port_to_443_and_loads_relay_ca(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    dials: list[tuple[str, int, str | None]] = []

    class Refused:
        async def __aenter__(self) -> None:
            raise ConnectionRefusedError

        async def __aexit__(self, *args: Any) -> None:
            pass

    def fake_connect(host: str, port: int, *, configuration: Any, **kwargs: Any) -> Refused:
        dials.append((host, port, configuration.cafile))
        return Refused()

    monkeypatch.setattr(wt_client, "_quic_connect", fake_connect)
    with pytest.raises(ConnectionRefusedError):
        await RelayClient.connect("https://relay.example", "robot", cafile="/ca.pem")
    assert dials == [("relay.example", 443, "/ca.pem")]


# /api/info discovery against an in-process HTTP server.


class _InfoHandler(BaseHTTPRequestHandler):
    """Answers every GET with `reply` (status, body) and records the paths."""

    reply: tuple[int, bytes] = (200, b"")
    paths: list[str] = []

    def do_GET(self) -> None:
        _InfoHandler.paths.append(self.path)
        status, body = _InfoHandler.reply
        self.send_response(status)
        self.send_header("content-type", "application/json")
        self.send_header("content-length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format: str, *args: Any) -> None:
        pass


@pytest.fixture
def info_server() -> Iterator[str]:
    """A loopback HTTP base whose /api/info answers _InfoHandler.reply."""
    _InfoHandler.reply = (200, b"")
    _InfoHandler.paths = []
    server = ThreadingHTTPServer(("127.0.0.1", 0), _InfoHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield f"http://127.0.0.1:{server.server_address[1]}"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)


def _serve(body: Any, status: int = 200) -> None:
    _InfoHandler.reply = (status, body if isinstance(body, bytes) else json.dumps(body).encode())


GOOD_INFO = {"wtUrl": "https://127.0.0.1:4433", "certHash": "aGFzaA==", "v": PROTOCOL_VERSION}


async def test_fetch_relay_info_good_shape(info_server: str) -> None:
    _serve(GOOD_INFO)
    info = await fetch_relay_info(info_server)
    assert info == RelayInfo(
        wt_url="https://127.0.0.1:4433", cert_hash="aGFzaA==", v=PROTOCOL_VERSION
    )
    assert _InfoHandler.paths == ["/api/info"]


async def test_fetch_relay_info_cert_hash_is_optional(info_server: str) -> None:
    # A relay with a real certificate (T12c) advertises no hash.
    _serve({"wtUrl": "https://relay.example:443", "v": PROTOCOL_VERSION})
    assert (await fetch_relay_info(info_server)).cert_hash is None


async def test_fetch_relay_info_rtc_flag(info_server: str) -> None:
    # v7: a relay with Cloudflare configured advertises rtc: true; absent is
    # false, anything but a boolean is a shape error.
    _serve(GOOD_INFO)
    assert (await fetch_relay_info(info_server)).rtc is False
    _serve({**GOOD_INFO, "rtc": True})
    assert (await fetch_relay_info(info_server)).rtc is True
    _serve({**GOOD_INFO, "rtc": "yes"})
    with pytest.raises(ProtocolError, match="unexpected shape"):
        await fetch_relay_info(info_server)


@pytest.mark.parametrize(
    "body",
    [
        {"certHash": "x", "v": PROTOCOL_VERSION},  # no wtUrl
        {"wtUrl": "https://127.0.0.1:4433", "certHash": "x", "v": True},  # bool is not a version
        {"wtUrl": "https://127.0.0.1:4433", "certHash": 7, "v": PROTOCOL_VERSION},
        [],
        b"<html>not a relay</html>",
    ],
)
async def test_fetch_relay_info_rejects_bad_shape(info_server: str, body: Any) -> None:
    _serve(body)
    with pytest.raises(ProtocolError, match="unexpected shape"):
        await fetch_relay_info(info_server)


async def test_fetch_relay_info_version_mismatch_names_both(info_server: str) -> None:
    _serve({**GOOD_INFO, "v": 99})
    with pytest.raises(
        ProtocolError, match=f"protocol v99, this bridge speaks v{PROTOCOL_VERSION}"
    ):
        await fetch_relay_info(info_server)


async def test_fetch_relay_info_http_error_stays_os_error(info_server: str) -> None:
    # A status error (a proxy mid-restart, say) is transient for the bridge's
    # reconnect loop: it must not look like a protocol problem.
    _serve(b"nope", status=404)
    with pytest.raises(OSError) as exc_info:
        await fetch_relay_info(info_server)
    assert not isinstance(exc_info.value, ProtocolError)


async def test_fetch_relay_info_unreachable_is_os_error() -> None:
    with pytest.raises(OSError):
        await fetch_relay_info("http://127.0.0.1:1")


@pytest.mark.parametrize(
    ("suffix", "path"),
    [
        ("", "/api/info"),
        ("/", "/api/info"),
        ("/relay", "/relay/api/info"),
        ("/relay/", "/relay/api/info"),
    ],
)
async def test_fetch_relay_info_resolves_like_the_sdk(
    info_server: str, suffix: str, path: str
) -> None:
    _serve(GOOD_INFO)
    await fetch_relay_info(info_server + suffix)
    assert _InfoHandler.paths == [path]

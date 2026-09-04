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
    CONTROL_CHANNEL,
    MAX_CONTROL_PAYLOAD_BYTES,
    PROTOCOL_VERSION,
    DataFrame,
    Error,
    FrameHeader,
    Hello,
    Msg,
    ProtocolError,
    RobotInfo,
    RobotManifest,
    Role,
    Subs,
    decode_datagram,
    encode_data_frame,
    encode_datagram,
)
from dimos.web.relay_bridge.wt_client import RelayClient, RelayRejectedError


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


async def test_mailbox_drops_stale_frames_under_slow_writer() -> None:
    # A wedged transport must never queue stale frames: the 1-slot mailbox
    # sheds everything but the newest, which is what goes out on unwedge.
    session = StubSession()
    payloads: list[bytes] = []
    polls = 0

    real_send = session.send_frame

    def send_frame(header: FrameHeader, payload: bytes) -> int:
        payloads.append(payload)
        return real_send(header, payload)

    def stream_in_flight(stream_id: int) -> bool:
        nonlocal polls
        polls += 1
        return polls < 20  # first send stays in flight for ~20 pump polls

    session.send_frame = send_frame  # type: ignore[method-assign]
    session.stream_in_flight = stream_in_flight  # type: ignore[method-assign]

    # stale_after high enough that the wedge never triggers the reset path:
    # this test pins the mailbox semantics alone.
    writer = _client(session).latest_writer("cam", stale_after=10.0)
    try:
        writer.offer(b"f0")
        # Wait until the pump provably took f0 (send_frame ran): it is wedged
        # in its in-flight poll, so the offers below only touch the mailbox.
        for _ in range(500):
            if payloads:
                break
            await asyncio.sleep(0.005)
        assert payloads, "pump did not send f0 within 2.5 s"
        for i in range(1, 6):
            writer.offer(b"f%d" % i)
        assert writer.dropped == 4  # f1..f4 displaced by their successors

        for _ in range(500):
            if writer.sent == 2:
                break
            await asyncio.sleep(0.005)
        assert payloads == [b"f0", b"f5"]  # only the newest frame followed the wedge
        assert writer.sent == 2
    finally:
        writer.stop()


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

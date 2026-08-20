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
    ChannelSpec,
    DataFrame,
    Error,
    FrameHeader,
    Msg,
    ProtocolError,
    RobotInfo,
    RobotManifest,
    Role,
    Subs,
    encode_data_frame,
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
        self.frames_dropped = 0
        self.control_dropped = 0
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
    await asyncio.sleep(0.05)  # let the pump run and die
    assert writer._task.done()
    assert isinstance(writer._task.exception(), ValueError)  # pydantic ValidationError
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


async def test_hello_oversized_datagram_refused() -> None:
    # An unsendable datagram wedges aioquic's whole datagram queue, so an
    # oversized hello must be refused before anything is queued. StubSession
    # has no send_msg/welcomed: a guard placed after the first send would fail
    # this test with AttributeError instead.
    client = _client(StubSession())
    manifest = RobotManifest(
        channels=[
            ChannelSpec(ch=f"channel_{i}", encoding="jpeg.v1", delivery="latest", maxHz=15.0)
            for i in range(40)
        ]
    )
    with pytest.raises(ProtocolError, match="hello datagram"):
        await client.hello(robot=RobotInfo(id="r1", name="r1", model="test"), manifest=manifest)


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

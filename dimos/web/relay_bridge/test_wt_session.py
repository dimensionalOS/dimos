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

"""Ingress-bound tests for the session's frame and control queues (no
network: frames are fed straight into the protocol callbacks)."""

from aioquic.quic.connection import QuicConnection
from aioquic.quic.events import StreamReset

from dimos.web.relay_bridge._wt_session import (
    _CONTROL_QUEUE_MAX,
    _FRAME_QUEUE_MAX,
    _FRAME_QUEUE_MAX_BYTES,
    SessionProtocol,
    _FrameQueue,
    make_quic_configuration,
)
from dimos.web.relay_bridge.protocol import (
    CONTROL_CHANNEL,
    MAX_CONTROL_PAYLOAD_BYTES,
    MAX_PUB_DATA_BYTES,
    DataFrame,
    Error,
    FrameHeader,
    Manifest,
    Stop,
    Subs,
    encode_data_frame,
    encode_datagram,
)


def _session() -> SessionProtocol:
    return SessionProtocol(QuicConnection(configuration=make_quic_configuration(insecure=True)))


def _frame_bytes(ch: str, seq: int, size: int) -> bytes:
    header = FrameHeader(ch=ch, seq=seq, ts=0.5, delivery="latest")
    return encode_data_frame(header, b"\xab" * size)


def _control_bytes(payload: bytes, seq: int = 1) -> bytes:
    header = FrameHeader(ch=CONTROL_CHANNEL, seq=seq, ts=0.5, delivery="reliable")
    return encode_data_frame(header, payload)


async def test_frame_queue_bounded_by_total_bytes():
    session = _session()
    size = 48 * 1024 * 1024  # three frames exceed the byte budget
    assert 2 * size <= _FRAME_QUEUE_MAX_BYTES < 3 * size
    for seq in range(3):
        session._stream_data_received(4 * seq, _frame_bytes("cam", seq, size), True)
    assert session.frames.qsize() == 2
    assert session.frames_dropped == 1
    assert session.frames.bytes <= _FRAME_QUEUE_MAX_BYTES
    # Oldest first: seq 0 was evicted, and get() releases its bytes.
    first = await session.frames.get()
    assert first.header.seq == 1
    assert session.frames.bytes == size


async def test_frame_queue_still_bounded_by_count():
    session = _session()
    for seq in range(_FRAME_QUEUE_MAX + 10):
        session._stream_data_received(4, _frame_bytes("cam", seq, 8), False)
    assert session.frames.qsize() == _FRAME_QUEUE_MAX
    assert session.frames_dropped == 10


async def test_single_frame_over_budget_still_enqueues_alone():
    # Eviction stops at an empty queue: a frame bigger than the whole byte
    # budget still makes progress instead of being unqueueable.
    queue = _FrameQueue(max_frames=4, max_bytes=1000)

    def frame(seq: int, size: int) -> DataFrame:
        return DataFrame(
            header=FrameHeader(ch="cam", seq=seq, ts=0.5, delivery="latest"),
            payload=b"\xab" * size,
        )

    queue.put_nowait(frame(0, 400))
    queue.put_nowait(frame(1, 400))
    queue.put_nowait(frame(2, 1200))
    assert queue.qsize() == 1
    assert queue.dropped == 2
    assert (await queue.get()).header.seq == 2
    assert queue.bytes == 0


async def test_per_encoding_payload_caps():
    session = _session()
    session._control_msg_received(
        Manifest(
            robotId="r1",
            manifest={
                "version": 1,
                "channels": [
                    {
                        "ch": "odom",
                        "encoding": "pose.json.v1",
                        "delivery": "reliable",
                        "maxHz": 20.5,
                    },
                    {"ch": "cam", "encoding": "jpeg.v1", "delivery": "latest", "maxHz": 15.5},
                    {
                        "ch": "global_costmap",
                        "encoding": "costmap.zlib.v1",
                        "delivery": "latest",
                        "maxHz": 5.5,
                    },
                ],
            },
        )
    )
    # The manifest still reaches the control consumer queue.
    assert session.control_msgs.qsize() == 1

    # Over the pose cap: dropped before it is queued.
    session._stream_data_received(4, _frame_bytes("odom", 1, 128 * 1024), False)
    assert session.frames.qsize() == 0
    assert session.frames_oversized == 1
    # Over the costmap cap: dropped too.
    session._stream_data_received(8, _frame_bytes("global_costmap", 1, 9 * 1024 * 1024), False)
    assert session.frames.qsize() == 0
    assert session.frames_oversized == 2
    # Under the caps: queued.
    session._stream_data_received(4, _frame_bytes("odom", 2, 100), False)
    session._stream_data_received(8, _frame_bytes("cam", 1, 1024 * 1024), False)
    session._stream_data_received(12, _frame_bytes("global_costmap", 2, 30 * 1024), False)
    assert session.frames.qsize() == 3
    # Channels with no known encoding only get the outer frame-size bound.
    session._stream_data_received(16, _frame_bytes("mystery", 1, 9 * 1024 * 1024), True)
    assert session.frames.qsize() == 4
    assert session.frames_oversized == 2


async def test_stream_reset_drops_the_frame_reader():
    # The relay ends every latest stream with a reset (reap/dispose); the
    # reader map must not grow one leaked entry per stream, and a partial
    # frame is stale by definition.
    session = _session()
    full = _frame_bytes("cam", 1, 1024)
    session._stream_data_received(4, full[: len(full) // 2], False)
    assert 4 in session._frame_readers
    session.quic_event_received(StreamReset(error_code=1, stream_id=4))
    assert 4 not in session._frame_readers
    assert session.frames.qsize() == 0

    # A reset for a stream that already dispatched its frame is a no-op.
    session._stream_data_received(8, _frame_bytes("cam", 2, 16), False)
    assert session.frames.qsize() == 1
    session.quic_event_received(StreamReset(error_code=1, stream_id=8))
    assert 8 not in session._frame_readers
    assert session.frames.qsize() == 1


async def test_control_frames_route_to_the_control_consumer():
    # Carrier @control frames feed control_msgs, never the data-frame queue.
    # Back-to-back frames in one chunk and a frame split across chunks both
    # dispatch (the carrier is one persistent stream, byte-count framing);
    # each snapshot supersedes the queued one, so only the newest remains -
    # the end state proves all three dispatched in order.
    session = _session()
    two = _control_bytes(encode_datagram(Subs(chs=[], n=1)), seq=1) + _control_bytes(
        encode_datagram(Subs(chs=["odom"], n=2)), seq=2
    )
    session._stream_data_received(3, two, False)
    assert session.control_msgs.qsize() == 1  # n=2 superseded n=1
    split = _control_bytes(encode_datagram(Subs(chs=["cam", "odom"], n=3)), seq=3)
    session._stream_data_received(3, split[:7], False)
    assert session.control_msgs.qsize() == 1  # the partial frame waits
    session._stream_data_received(3, split[7:], False)
    assert session.frames.qsize() == 0
    assert session.control_msgs.get_nowait() == Subs(chs=["cam", "odom"], n=3)
    assert session.control_dropped == 0
    assert not session.closed.is_set()


async def test_undecodable_control_payload_is_dropped_not_fatal():
    # Mirrors the relay's post-registration junk-control handling; an
    # exactly-at-cap payload also pins the size boundary as accepted.
    session = _session()
    session.incoming_is_carrier = True
    session._stream_data_received(3, _control_bytes(b"\xff" * MAX_CONTROL_PAYLOAD_BYTES), False)
    assert session.control_invalid == 1
    assert session.control_msgs.qsize() == 0
    assert session.frames.qsize() == 0
    assert not session.closed.is_set()


async def test_oversized_control_payload_fails_the_session():
    session = _session()
    session._stream_data_received(3, _control_bytes(b"x" * (MAX_CONTROL_PAYLOAD_BYTES + 1)), False)
    assert session.closed.is_set()
    assert session.control_msgs.qsize() == 0


async def test_corrupt_carrier_framing_fails_the_robot_session():
    # Robot role (incoming_is_carrier): the carrier is a control dependency,
    # so corruption ends the session instead of leaving it alive with frozen
    # subscriptions. Frames decoded before the corruption still dispatch
    # first.
    session = _session()
    session.incoming_is_carrier = True
    good = _control_bytes(encode_datagram(Subs(chs=["odom"], n=1)))
    session._stream_data_received(3, good + b"\xff" * 8, False)
    assert session.control_msgs.get_nowait() == Subs(chs=["odom"], n=1)
    assert session.closed.is_set()
    assert session._frame_readers[3] is None


async def test_corrupt_framing_on_a_viewer_session_poisons_only_that_stream():
    # Default (viewer) behavior is unchanged: the corrupt stream is abandoned
    # alone and the session lives on.
    session = _session()
    session._stream_data_received(4, b"\xff" * 8, False)
    assert session._frame_readers[4] is None
    assert not session.closed.is_set()
    # Later bytes on the poisoned stream are ignored; other streams work.
    session._stream_data_received(4, _frame_bytes("cam", 1, 16), False)
    assert session.frames.qsize() == 0
    session._stream_data_received(8, _frame_bytes("cam", 2, 16), False)
    assert session.frames.qsize() == 1


async def test_subs_snapshot_survives_a_teleop_flood():
    # All non-handshake control shares one drop-oldest queue; eviction must
    # take the oldest teleop message, never the one queued snapshot - with
    # the carrier there is no periodic resend to heal an evicted one.
    session = _session()
    session.incoming_is_carrier = True
    session._stream_data_received(
        3, _control_bytes(encode_datagram(Subs(chs=["odom"], n=1))), False
    )
    for seq in range(_CONTROL_QUEUE_MAX):
        session._control_msg_received(Stop(seq=seq, ts=0.5))
    assert session.control_msgs.qsize() == _CONTROL_QUEUE_MAX
    assert session.control_dropped == 1
    msgs = [session.control_msgs.get_nowait() for _ in range(_CONTROL_QUEUE_MAX)]
    assert msgs[0] == Subs(chs=["odom"], n=1)
    # The oldest teleop message (seq 0) was the eviction victim.
    assert [m.seq for m in msgs[1:] if isinstance(m, Stop)] == list(range(1, _CONTROL_QUEUE_MAX))


def _tx_bytes(payload: bytes, seq: int = 1) -> bytes:
    header = FrameHeader(
        ch="human_input",
        seq=seq,
        ts=0.5,
        delivery="reliable",
        meta={"id": f"p{seq}", "principal": "local", "relayTs": 0.5},
    )
    return encode_data_frame(header, payload)


async def test_carrier_tx_frames_join_the_control_queue_in_order():
    # A forwarded publish (non-control carrier frame) rides the same ordered
    # consumer as control; nothing lands in the undrained data-frame queue.
    session = _session()
    session.incoming_is_carrier = True
    chunk = _control_bytes(encode_datagram(Subs(chs=["odom"], n=1)), seq=1) + _tx_bytes(
        b'{"text":"salut"}', seq=2
    )
    session._stream_data_received(3, chunk, False)
    assert session.frames.qsize() == 0
    assert session.control_msgs.get_nowait() == Subs(chs=["odom"], n=1)
    frame = session.control_msgs.get_nowait()
    assert isinstance(frame, DataFrame)
    assert frame.header.ch == "human_input"
    assert frame.header.meta == {"id": "p2", "principal": "local", "relayTs": 0.5}
    assert frame.payload == b'{"text":"salut"}'
    assert not session.closed.is_set()


async def test_carrier_tx_oversize_payload_fails_the_session():
    # The relay caps serialized publish data; past the cap the control path
    # is broken. The exact cap still passes.
    session = _session()
    session.incoming_is_carrier = True
    session._stream_data_received(3, _tx_bytes(b"x" * MAX_PUB_DATA_BYTES), False)
    assert session.control_msgs.qsize() == 1
    assert not session.closed.is_set()
    session._stream_data_received(3, _tx_bytes(b"x" * (MAX_PUB_DATA_BYTES + 1), seq=2), False)
    assert session.closed.is_set()
    assert session.control_msgs.qsize() == 1  # the oversize frame never queued


async def test_viewer_leg_tx_channel_frames_stay_data_frames():
    # Only the robot leg's carrier reroutes non-control frames; a viewer
    # session keeps every data frame in the data-frame queue.
    session = _session()
    session._stream_data_received(4, _tx_bytes(b"{}"), False)
    assert session.frames.qsize() == 1
    assert session.control_msgs.qsize() == 0


async def test_subs_snapshot_survives_a_publish_flood():
    # Publish frames are ordinary eviction victims; the one queued snapshot
    # is not (an evicted publish is settled by the relay's timeout, an
    # evicted snapshot would freeze subscriptions).
    session = _session()
    session.incoming_is_carrier = True
    session._stream_data_received(
        3, _control_bytes(encode_datagram(Subs(chs=["odom"], n=1))), False
    )
    for seq in range(_CONTROL_QUEUE_MAX):
        session._stream_data_received(3, _tx_bytes(b"{}", seq=seq + 2), False)
    assert session.control_msgs.qsize() == _CONTROL_QUEUE_MAX
    assert session.control_dropped == 1
    first = session.control_msgs.get_nowait()
    assert first == Subs(chs=["odom"], n=1)
    # The oldest publish frame (seq 2) was the eviction victim.
    second = session.control_msgs.get_nowait()
    assert isinstance(second, DataFrame) and second.header.seq == 3


async def test_correlated_errors_join_the_consumer_queue_not_the_handshake_slot():
    # An error carrying requestId is addressed to one publish, not the
    # session: it must reach the consumer and must never unblock a hello()
    # waiter or overwrite the handshake error slot.
    session = _session()
    correlated = Error(code="not_publishable", message="nu", requestId="r-1")
    session._control_msg_received(correlated)
    assert session.control_msgs.get_nowait() == correlated
    assert session.relay_error is None
    assert not session.welcomed.is_set()
    # A session-level error keeps the handshake behavior.
    session._control_msg_received(Error(code="version_mismatch", message="v"))
    assert session.relay_error is not None and session.relay_error.code == "version_mismatch"
    assert session.welcomed.is_set()
    assert session.control_msgs.qsize() == 0


async def test_newer_subs_snapshot_supersedes_the_queued_one():
    # Snapshots are full state: a queued older one is replaced (not counted
    # as a drop) while other control keeps its order.
    session = _session()
    session._control_msg_received(Subs(chs=["odom"], n=1))
    session._control_msg_received(Stop(seq=1, ts=0.5))
    session._control_msg_received(Subs(chs=["cam", "odom"], n=2))
    msgs = [session.control_msgs.get_nowait() for _ in range(2)]
    assert msgs == [Stop(seq=1, ts=0.5), Subs(chs=["cam", "odom"], n=2)]
    assert session.control_dropped == 0


async def test_carrier_reset_fails_the_robot_session():
    # The relay never replaces a carrier, so a reset while the connection
    # lives (e.g. the relay's carrier dispose racing its delayed session
    # close) must fail the session now, not after the idle timeout; the
    # partial snapshot in the reader is discarded with it.
    session = _session()
    session.incoming_is_carrier = True
    frame = _control_bytes(encode_datagram(Subs(chs=["odom"], n=1)))
    session._stream_data_received(3, frame[: len(frame) // 2], False)
    session.quic_event_received(StreamReset(error_code=1, stream_id=3))
    assert 3 not in session._frame_readers
    assert session.closed.is_set()
    assert session.control_msgs.qsize() == 0


async def test_carrier_fin_fails_the_robot_session():
    # Deno aborts, never FINs; an ended carrier is equally unreplaceable.
    # Complete frames in the final chunk still dispatch first.
    session = _session()
    session.incoming_is_carrier = True
    session._stream_data_received(3, _control_bytes(encode_datagram(Subs(chs=["odom"], n=1))), True)
    assert session.control_msgs.get_nowait() == Subs(chs=["odom"], n=1)
    assert session.closed.is_set()


async def test_robot_bidi_receive_reset_is_not_carrier_loss():
    # The relay aborts its send half of EVERY robot-opened bidi stream on
    # accept, so the robot sees a reset per hello/data stream (id % 4 == 0);
    # only server-initiated uni ids (% 4 == 3) are the carrier.
    session = _session()
    session.incoming_is_carrier = True
    session.quic_event_received(StreamReset(error_code=1, stream_id=0))
    session.quic_event_received(StreamReset(error_code=1, stream_id=4))
    assert not session.closed.is_set()


async def test_viewer_uni_reset_and_fin_stay_routine():
    # Viewer legs receive many relay-opened uni data streams; resets (reap,
    # dispose, teardown) and FINs remain routine stream ends there.
    session = _session()
    session._stream_data_received(3, _frame_bytes("cam", 1, 16), True)
    assert session.frames.qsize() == 1
    session.quic_event_received(StreamReset(error_code=1, stream_id=7))
    assert not session.closed.is_set()


async def test_failed_session_does_not_leak_into_a_fresh_epoch():
    # Epoch isolation is structural - a reconnect builds a whole new
    # SessionProtocol - so a failed session's state must stay instance-local
    # and the replacement must ingest carrier frames normally.
    failed = _session()
    failed.incoming_is_carrier = True
    failed.quic_event_received(StreamReset(error_code=1, stream_id=3))
    assert failed.closed.is_set()
    fresh = _session()
    fresh.incoming_is_carrier = True
    fresh._stream_data_received(3, _control_bytes(encode_datagram(Subs(chs=["odom"], n=1))), False)
    assert fresh.control_msgs.qsize() == 1
    assert not fresh.closed.is_set()

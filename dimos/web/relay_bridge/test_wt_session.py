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

"""Ingress-bound tests for the viewer session's frame queue (no network:
frames are fed straight into the protocol callbacks)."""

from aioquic.quic.connection import QuicConnection
from aioquic.quic.events import StreamReset

from dimos.web.relay_bridge._wt_session import (
    _FRAME_QUEUE_MAX,
    _FRAME_QUEUE_MAX_BYTES,
    SessionProtocol,
    _FrameQueue,
    make_quic_configuration,
)
from dimos.web.relay_bridge.protocol import (
    DataFrame,
    FrameHeader,
    Manifest,
    encode_data_frame,
)


def _session() -> SessionProtocol:
    return SessionProtocol(QuicConnection(configuration=make_quic_configuration(insecure=True)))


def _frame_bytes(ch: str, seq: int, size: int) -> bytes:
    header = FrameHeader(ch=ch, seq=seq, ts=0.5, delivery="latest")
    return encode_data_frame(header, b"\xab" * size)


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

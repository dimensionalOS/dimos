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

"""aioquic session internals for the relay bridge.

The quirks this module works around are documented in web/README.md: control rides
datagrams (the relay may never write on our bidi streams), data frames are
one-shot bidi streams, delivery pacing uses `sender.is_finished` (ACK-based),
and reset_stream is only ever called on ids still present in `_quic._streams`
within one event-loop turn (resetting a discarded id corrupts aioquic's
stream-id allocator).
"""

from __future__ import annotations

import asyncio
import ssl
import time

from aioquic.asyncio.protocol import QuicConnectionProtocol
from aioquic.h3.connection import H3_ALPN, H3Connection
from aioquic.h3.events import (
    DatagramReceived,
    H3Event,
    HeadersReceived,
    WebTransportStreamDataReceived,
)
from aioquic.quic.configuration import QuicConfiguration
from aioquic.quic.events import ConnectionTerminated, QuicEvent, StreamReset

from dimos.utils.logging_config import setup_logger
from dimos.web.relay_bridge.protocol import (
    DataFrame,
    DataFrameStreamError,
    DataFrameStreamReader,
    Error,
    FrameHeader,
    Manifest,
    Msg,
    Pong,
    Welcome,
    decode_datagram,
    encode_data_frame,
    encode_datagram,
)

logger = setup_logger()

# Received data frames waiting for the consumer; drop-oldest beyond either
# bound. The byte bound is what matters against a hostile/compromised relay:
# 256 frames of near-MAX_DATA_FRAME_BYTES would otherwise retain ~16 GiB.
_FRAME_QUEUE_MAX = 256
_FRAME_QUEUE_MAX_BYTES = 128 * 1024 * 1024

# Realistic payload caps for channels whose encoding is known from the
# watched robot's manifest (frames themselves carry no encoding);
# MAX_DATA_FRAME_BYTES stays the outer bound for everything else. Generous:
# a 4K quality-90 JPEG is ~4 MiB, a pose JSON object ~100 B, a compressed
# long-run costmap ~10-30 KB (the cap leaves room for pathological grids).
_MAX_PAYLOAD_BYTES = {
    "jpeg.v1": 8 * 1024 * 1024,
    "pose.json.v1": 64 * 1024,
    "costmap.zlib.v1": 8 * 1024 * 1024,
}

# Relay-pushed control messages (subs snapshots, robots, manifest) waiting for
# the consumer; drop-oldest beyond this. Snapshots are idempotent and resent,
# so a dropped one heals on the next round.
_CONTROL_QUEUE_MAX = 64

# The relay's abort of its send half surfaces as a stream reset; also used
# when this side resets a stale latest-wins stream.
STALE_STREAM_ERROR_CODE = 0x01


class _FrameQueue:
    """Drop-oldest frame buffer bounded by frame count and total payload bytes.

    Single event loop only (put from the protocol callback, get from the
    consumer task), so the plain byte counter is safe.
    """

    def __init__(self, max_frames: int, max_bytes: int) -> None:
        self._queue: asyncio.Queue[DataFrame] = asyncio.Queue()
        self._max_frames = max_frames
        self._max_bytes = max_bytes
        self.bytes = 0
        self.dropped = 0

    def qsize(self) -> int:
        return self._queue.qsize()

    def put_nowait(self, frame: DataFrame) -> None:
        """Enqueue, evicting oldest frames until both bounds hold (a single
        frame over the byte bound still enqueues once it is alone)."""
        size = len(frame.payload)
        while self._queue.qsize() > 0 and (
            self._queue.qsize() >= self._max_frames or self.bytes + size > self._max_bytes
        ):
            self.bytes -= len(self._queue.get_nowait().payload)
            self.dropped += 1
        self._queue.put_nowait(frame)
        self.bytes += size

    async def get(self) -> DataFrame:
        frame = await self._queue.get()
        self.bytes -= len(frame.payload)
        return frame


def make_quic_configuration(insecure: bool) -> QuicConfiguration:
    config = QuicConfiguration(
        is_client=True,
        alpn_protocols=H3_ALPN,
        # Required: without it the relay's H3_DATAGRAM setting kills the
        # session at SETTINGS time.
        max_datagram_frame_size=65536,
    )
    if insecure:
        config.verify_mode = ssl.CERT_NONE
    return config


class SessionProtocol(QuicConnectionProtocol):
    """One WebTransport session (CONNECT + datagram control + data streams)."""

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)  # type: ignore[arg-type]
        self.h3 = H3Connection(self._quic, enable_webtransport=True)
        self.session_ready = asyncio.Event()
        self.welcomed = asyncio.Event()
        # Set when the QUIC connection terminates. Public mirror of the
        # base class's private _closed (see wait_closed()); producers and the
        # delivery poll consult it so they stop the moment the relay dies.
        self.closed = asyncio.Event()
        self.relay_error: Error | None = None
        self.frames = _FrameQueue(_FRAME_QUEUE_MAX, _FRAME_QUEUE_MAX_BYTES)
        self.frames_oversized = 0
        self.control_msgs: asyncio.Queue[Msg] = asyncio.Queue(maxsize=_CONTROL_QUEUE_MAX)
        self.control_dropped = 0
        self.session_id: int | None = None
        self._pong_waiters: dict[int | float, asyncio.Future[Pong]] = {}
        self._frame_readers: dict[int, DataFrameStreamReader | None] = {}
        # ch -> encoding, learned from manifest messages; arms the
        # per-encoding payload caps in _MAX_PAYLOAD_BYTES.
        self._encodings: dict[str, str] = {}

    def open_session(self, authority: str, path: str) -> None:
        self.session_id = self._quic.get_next_available_stream_id()
        self.h3.send_headers(
            self.session_id,
            [
                (b":method", b"CONNECT"),
                (b":protocol", b"webtransport"),
                (b":scheme", b"https"),
                (b":authority", authority.encode()),
                (b":path", path.encode()),
            ],
            end_stream=False,
        )
        self.transmit()

    def quic_event_received(self, event: QuicEvent) -> None:
        if isinstance(event, ConnectionTerminated):
            self.closed.set()
        elif isinstance(event, StreamReset):
            # aioquic's H3 layer ignores resets, and the relay ends every
            # latest stream with one (reap, dispose, or teardown): without
            # this pop the reader map grows by one entry per latest stream.
            # A partial frame in that reader is stale by definition.
            self._frame_readers.pop(event.stream_id, None)
        for h3_event in self.h3.handle_event(event):
            self._h3_event_received(h3_event)

    def _h3_event_received(self, event: H3Event) -> None:
        if isinstance(event, HeadersReceived) and event.stream_id == self.session_id:
            status = dict(event.headers).get(b":status")
            if status == b"200":
                self.session_ready.set()
            else:
                logger.warning(f"WebTransport CONNECT rejected: {event.headers}")
        elif isinstance(event, DatagramReceived):
            msg = decode_datagram(event.data)
            if msg is not None:
                self._control_msg_received(msg)
        elif isinstance(event, WebTransportStreamDataReceived):
            self._stream_data_received(event.stream_id, event.data, event.stream_ended)

    @property
    def frames_dropped(self) -> int:
        return self.frames.dropped

    def _control_msg_received(self, msg: Msg) -> None:
        if isinstance(msg, Welcome):
            self.welcomed.set()
        elif isinstance(msg, Error):
            logger.warning(f"relay error: {msg.code}: {msg.message}")
            self.relay_error = msg
            self.welcomed.set()  # unblock hello() waiters; they check relay_error
        elif isinstance(msg, Pong):
            waiter = self._pong_waiters.pop(msg.n, None)
            if waiter is not None and not waiter.done():
                waiter.set_result(msg)
        else:
            if isinstance(msg, Manifest) and msg.manifest is not None:
                # The nested manifest is an opaque raw record at this layer;
                # pick the encodings out defensively (payload caps are a
                # safety net, not a validator).
                channels = msg.manifest.get("channels")
                if isinstance(channels, list):
                    self._encodings.update(
                        {
                            c["ch"]: c["encoding"]
                            for c in channels
                            if isinstance(c, dict)
                            and isinstance(c.get("ch"), str)
                            and isinstance(c.get("encoding"), str)
                        }
                    )
            # Session messages (subs snapshots, robots, manifest, ...) go to
            # the consumer queue; see RelayClient.control_messages().
            if self.control_msgs.full():
                self.control_msgs.get_nowait()
                self.control_dropped += 1
            self.control_msgs.put_nowait(msg)

    def _stream_data_received(self, stream_id: int, data: bytes, ended: bool) -> None:
        # A latest stream carries one frame; a reliable channel's persistent
        # stream carries them back to back, so frames dispatch on byte count
        # (the peer's FIN can be seconds late). None poisons a stream that
        # produced a bad frame: framing is unrecoverable mid-stream.
        reader = self._frame_readers.get(stream_id, DataFrameStreamReader())
        if reader is not None:
            self._frame_readers[stream_id] = reader
            try:
                frames = reader.push(data)
            except DataFrameStreamError as e:
                # Frames decoded before the corruption still count; the
                # stream itself is unrecoverable.
                logger.warning(f"bad data frame on stream {stream_id}: {e}")
                frames = e.frames
                self._frame_readers[stream_id] = None
            except Exception:
                # Network ingress: no decoder bug may escape into aioquic's
                # datagram callback and tear down the whole session.
                logger.exception(f"unexpected error decoding data frame on stream {stream_id}")
                frames = []
                self._frame_readers[stream_id] = None
            for frame in frames:
                limit = _MAX_PAYLOAD_BYTES.get(self._encodings.get(frame.header.ch, ""))
                if limit is not None and len(frame.payload) > limit:
                    self.frames_oversized += 1
                    logger.warning(
                        f"dropping {frame.header.ch} frame: {len(frame.payload)} B over "
                        f"the {limit} B cap for its encoding"
                    )
                    continue
                self.frames.put_nowait(frame)
        if ended:
            self._frame_readers.pop(stream_id, None)

    def send_msg(self, msg: Msg) -> None:
        assert self.session_id is not None
        self.h3.send_datagram(self.session_id, encode_datagram(msg))
        self.transmit()

    def register_pong_waiter(self, n: int) -> asyncio.Future[Pong]:
        waiter: asyncio.Future[Pong] = asyncio.get_running_loop().create_future()
        self._pong_waiters[n] = waiter
        return waiter

    def discard_pong_waiter(self, n: int) -> None:
        """Drop a pong waiter whose ping timed out (else it leaks forever)."""
        self._pong_waiters.pop(n, None)

    def send_frame(self, header: FrameHeader, payload: bytes) -> int:
        """One-shot bidi stream: whole frame + FIN in one call. Returns the stream id."""
        assert self.session_id is not None
        stream_id = self.h3.create_webtransport_stream(self.session_id, is_unidirectional=False)
        self._quic.send_stream_data(stream_id, encode_data_frame(header, payload), end_stream=True)
        self.transmit()
        return stream_id

    def stream_in_flight(self, stream_id: int) -> bool:
        """True until every byte and the FIN (or a reset) is ACKed.

        `sender.is_finished` is the precise per-stream delivery signal; the
        stream's presence in `_streams` lags actual delivery by hundreds of ms
        (discard also waits on the receive direction).
        """
        stream = self._quic._streams.get(stream_id)
        if stream is None:
            return False
        return not bool(stream.sender.is_finished)

    def reset_if_in_flight(self, stream_id: int) -> bool:
        """Reset a stale stream. Membership check and reset happen in the same
        event-loop turn: aioquic's reset_stream() on a discarded id re-creates
        the stream and rewinds the stream-id allocator (see web/README.md)."""
        if stream_id not in self._quic._streams:
            return False
        self._quic.reset_stream(stream_id, STALE_STREAM_ERROR_CODE)
        self.transmit()
        return True

    async def wait_delivered(self, stream_id: int, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while self.stream_in_flight(stream_id):
            if self.closed.is_set() or time.monotonic() >= deadline:
                return False
            await asyncio.sleep(0.002)
        return True

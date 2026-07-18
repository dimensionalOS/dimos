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
from aioquic.quic.events import ConnectionTerminated, QuicEvent

from dimos.utils.logging_config import setup_logger
from dimos.web.relay_bridge.protocol import (
    DataFrame,
    DataFrameReader,
    Error,
    FrameHeader,
    Msg,
    Pong,
    ProtocolError,
    Welcome,
    decode_datagram,
    encode_data_frame,
    encode_datagram,
)

logger = setup_logger()

# Received data frames waiting for the consumer; drop-oldest beyond this.
_FRAME_QUEUE_MAX = 256

# The relay's abort of its send half surfaces as a stream reset; also used
# when this side resets a stale latest-wins stream.
STALE_STREAM_ERROR_CODE = 0x01


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
        self.frames: asyncio.Queue[DataFrame] = asyncio.Queue(maxsize=_FRAME_QUEUE_MAX)
        self.frames_dropped = 0
        self.session_id: int | None = None
        self._pong_waiters: dict[int, asyncio.Future[Pong]] = {}
        self._frame_readers: dict[int, DataFrameReader | None] = {}

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

    def _stream_data_received(self, stream_id: int, data: bytes, ended: bool) -> None:
        # None marks a stream whose frame is already complete (the late FIN
        # arrives up to ~1 s afterwards; any trailing bytes are ignored).
        reader = self._frame_readers.get(stream_id, DataFrameReader())
        if reader is not None:
            self._frame_readers[stream_id] = reader
            try:
                frame = reader.push(data)
            except ProtocolError as e:
                logger.warning(f"bad data frame on stream {stream_id}: {e}")
                frame = None
                self._frame_readers[stream_id] = None
            except Exception:
                # Network ingress: no decoder bug may escape into aioquic's
                # datagram callback and tear down the whole session.
                logger.exception(f"unexpected error decoding data frame on stream {stream_id}")
                frame = None
                self._frame_readers[stream_id] = None
            if frame is not None:
                self._frame_readers[stream_id] = None
                if self.frames.full():
                    self.frames.get_nowait()
                    self.frames_dropped += 1
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

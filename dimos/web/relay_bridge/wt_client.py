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

"""WebTransport client for the DimOS relay (robot leg, plus a test viewer)."""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator
import contextlib
import itertools
import time
from types import TracebackType
from typing import Any, cast
from urllib.parse import urlparse

from aioquic.asyncio.client import connect as aioquic_connect

from dimos.utils.logging_config import setup_logger
from dimos.web.relay_bridge._wt_session import SessionProtocol, make_quic_configuration
from dimos.web.relay_bridge.protocol import (
    PROTOCOL_VERSION,
    DataFrame,
    Delivery,
    FrameHeader,
    Hello,
    Ping,
    ProtocolError,
    Role,
)

logger = setup_logger()

_LOOPBACK_HOSTS = frozenset({"127.0.0.1", "localhost", "::1"})


class RelayClient:
    """One WebTransport session with the relay.

    Use :meth:`connect` (or :func:`connect_with_backoff`); the constructor is
    internal. All methods must be called from the event loop that connected.
    """

    def __init__(self, url: str, role: Role, session: SessionProtocol, ctx: Any) -> None:
        self.url = url
        self.role = role
        self._session = session
        self._ctx = ctx  # aioquic.asyncio.connect() context manager
        self._ping_n = itertools.count(1)
        self._seq: dict[str, itertools.count[int]] = {}
        self._writers: list[LatestChannelWriter] = []

    @classmethod
    async def connect(
        cls,
        url: str,
        role: Role,
        *,
        insecure: bool | None = None,
        timeout: float = 10.0,
    ) -> RelayClient:
        """Connect to `url` (the relay's wtUrl, e.g. https://127.0.0.1:4433).

        `insecure` skips certificate verification and defaults to True for
        loopback hosts only (the local relay uses an ephemeral self-signed
        cert). Passing insecure=True for a non-loopback host is refused.
        """
        parsed = urlparse(url)
        host = parsed.hostname
        port = parsed.port
        if parsed.scheme != "https" or host is None or port is None:
            raise ValueError(f"relay URL must look like https://host:port, got {url!r}")
        is_loopback = host in _LOOPBACK_HOSTS
        if insecure is None:
            insecure = is_loopback
        if insecure and not is_loopback:
            raise ValueError(f"insecure=True is only allowed for loopback hosts, got {host!r}")
        path = parsed.path if parsed.path not in ("", "/") else f"/{role}"

        ctx = aioquic_connect(
            host,
            port,
            configuration=make_quic_configuration(insecure),
            create_protocol=SessionProtocol,
        )
        session = cast("SessionProtocol", await ctx.__aenter__())
        try:
            session.open_session(f"{host}:{port}", path)
            await asyncio.wait_for(session.session_ready.wait(), timeout)
        except BaseException:
            await ctx.__aexit__(None, None, None)
            raise
        logger.info(f"WebTransport session established: {url} path={path}")
        return cls(url, role, session, ctx)

    async def __aenter__(self) -> RelayClient:
        return self

    async def __aexit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        await self.close()

    async def close(self) -> None:
        for writer in self._writers:
            writer.stop()
        self._writers.clear()
        await self._ctx.__aexit__(None, None, None)

    async def hello(self, timeout: float = 5.0) -> None:
        """Send hello datagrams until the relay's welcome arrives.

        Datagrams are lossy, so the hello is repeated every 200 ms. Raises
        ProtocolError if the relay answers with an error (version mismatch),
        TimeoutError if nothing answers within `timeout`.
        """
        deadline = time.monotonic() + timeout
        while True:
            self._session.send_msg(Hello(v=PROTOCOL_VERSION, role=self.role))
            with contextlib.suppress(asyncio.TimeoutError):
                await asyncio.wait_for(self._session.welcomed.wait(), 0.2)
            if self._session.relay_error is not None:
                err = self._session.relay_error
                raise ProtocolError(f"relay rejected hello: {err.code}: {err.message}")
            if self._session.welcomed.is_set():
                return
            if time.monotonic() >= deadline:
                raise TimeoutError(f"no welcome from relay within {timeout} s")

    async def ping(self, timeout: float = 5.0) -> float:
        """Datagram ping; returns the round-trip time in seconds."""
        n = next(self._ping_n)
        waiter = self._session.register_pong_waiter(n)
        start = time.monotonic()
        self._session.send_msg(Ping(n=n, ts=time.time()))
        try:
            await asyncio.wait_for(waiter, timeout)
        finally:
            # A lost pong (or a dead connection) never resolves the waiter;
            # drop it so _pong_waiters cannot grow without bound.
            self._session.discard_pong_waiter(n)
        return time.monotonic() - start

    async def wait_closed(self) -> None:
        """Block until the relay connection terminates (any role)."""
        await self._session.wait_closed()

    @property
    def is_closed(self) -> bool:
        """True once the relay connection has terminated."""
        return self._session.closed.is_set()

    def send_frame(
        self,
        ch: str,
        payload: bytes,
        *,
        delivery: Delivery = "reliable",
        meta: dict[str, Any] | None = None,
        ts: float | None = None,
    ) -> int:
        """Send one data frame (one-shot bidi stream). Returns the stream id.

        The frame is buffered by aioquic and delivered in the background;
        reliable senders that need pacing can `await wait_delivered(...)`.
        """
        seq = next(self._seq.setdefault(ch, itertools.count()))
        header = FrameHeader(
            ch=ch, seq=seq, ts=time.time() if ts is None else ts, delivery=delivery, meta=meta
        )
        return self._session.send_frame(header, payload)

    async def wait_delivered(self, stream_id: int, timeout: float = 10.0) -> bool:
        """True once the frame on `stream_id` is fully ACKed."""
        return await self._session.wait_delivered(stream_id, timeout)

    def latest_writer(self, ch: str, *, stale_after: float = 0.5) -> LatestChannelWriter:
        """Delivery-paced latest-wins writer for `ch` (e.g. camera frames)."""
        writer = LatestChannelWriter(self, ch, stale_after=stale_after)
        self._writers.append(writer)
        return writer

    async def frames(self) -> AsyncIterator[DataFrame]:
        """Data frames pushed by the relay (viewer role).

        Buffered frames drain before the close is honored. Cancelling the
        consumer mid-iteration never orphans the queue getter (which would
        steal the next delivered frame).
        """
        closed = asyncio.ensure_future(self._session.wait_closed())
        try:
            while True:
                get = asyncio.ensure_future(self._session.frames.get())
                try:
                    await asyncio.wait({get, closed}, return_when=asyncio.FIRST_COMPLETED)
                    # get first: a frame buffered before close still drains.
                    if not get.done():
                        return
                    frame = get.result()
                finally:
                    get.cancel()
                yield frame
        finally:
            closed.cancel()

    @property
    def frames_dropped(self) -> int:
        """Frames dropped locally because the consumer lagged (drop-oldest)."""
        return self._session.frames_dropped


class LatestChannelWriter:
    """Latest-wins channel writer: 1-slot mailbox + delivery-paced sender.

    `offer()` never blocks; when frames arrive faster than the link delivers,
    intermediate frames are dropped at the mailbox (counted in `dropped`) and
    the newest one is sent as soon as the in-flight stream is ACKed. A stream
    stalled longer than `stale_after` with a newer frame waiting is reset
    (receivers discard the partial frame).
    """

    def __init__(self, client: RelayClient, ch: str, *, stale_after: float) -> None:
        self._client = client
        self.ch = ch
        self.stale_after = stale_after
        self.dropped = 0
        self.sent = 0
        self.resets = 0
        self._mailbox: asyncio.Queue[tuple[bytes, dict[str, Any] | None]] = asyncio.Queue(maxsize=1)
        self._task = asyncio.ensure_future(self._pump())
        self._task.add_done_callback(self._on_pump_done)

    def _on_pump_done(self, task: asyncio.Future[None]) -> None:
        # Surface an unexpected pump death; a silent one would make offer()
        # accept frames that are never sent. Clean stop/close is not an error.
        if task.cancelled():
            return
        exc = task.exception()
        if exc is not None:
            logger.error(f"latest-wins writer for {self.ch} died", exc_info=exc)

    def offer(self, payload: bytes, meta: dict[str, Any] | None = None) -> None:
        """Queue the newest frame, dropping any not-yet-sent predecessor.

        Event-loop only; producers on other threads must go through
        `loop.call_soon_threadsafe(writer.offer, ...)`. Raises RuntimeError if
        the pump is no longer running (session closed, stopped, or died) so a
        dead channel is visible at the producer instead of silently dropping.
        """
        if self._task.done():
            raise RuntimeError(f"latest-wins writer for {self.ch} is not running")
        if self._mailbox.full():
            self._mailbox.get_nowait()
            self.dropped += 1
        self._mailbox.put_nowait((payload, meta))

    def stop(self) -> None:
        self._task.cancel()

    async def _pump(self) -> None:
        session = self._client._session
        closed = asyncio.ensure_future(session.closed.wait())
        try:
            while not session.closed.is_set():
                get = asyncio.ensure_future(self._mailbox.get())
                try:
                    await asyncio.wait({get, closed}, return_when=asyncio.FIRST_COMPLETED)
                    if not get.done():
                        break  # session closed with an empty mailbox
                    payload, meta = get.result()
                finally:
                    get.cancel()
                if session.closed.is_set():
                    break
                stream_id = self._client.send_frame(self.ch, payload, delivery="latest", meta=meta)
                self.sent += 1
                started = time.monotonic()
                while session.stream_in_flight(stream_id):
                    if session.closed.is_set():
                        break
                    await asyncio.sleep(0.002)
                    if time.monotonic() - started > self.stale_after and not self._mailbox.empty():
                        # Stalled with a newer frame waiting: abandon this one.
                        # reset_if_in_flight rechecks membership in this same
                        # event-loop turn (required, see web/README.md).
                        if session.reset_if_in_flight(stream_id):
                            self.resets += 1
                        break
        finally:
            closed.cancel()
        logger.info(f"latest-wins writer for {self.ch}: session closed, stopping")


async def connect_with_backoff(
    url: str,
    role: Role,
    *,
    insecure: bool | None = None,
    max_attempts: int = 8,
    base_delay: float = 0.5,
    max_delay: float = 10.0,
) -> RelayClient:
    """connect() with exponential backoff for flaky startup ordering."""
    delay = base_delay
    for attempt in range(1, max_attempts + 1):
        try:
            return await RelayClient.connect(url, role, insecure=insecure)
        except (OSError, asyncio.TimeoutError, ConnectionError) as e:
            if attempt == max_attempts:
                raise
            logger.info(f"relay connect attempt {attempt} failed ({e}); retrying in {delay:.1f}s")
            await asyncio.sleep(delay)
            delay = min(delay * 2, max_delay)
    raise AssertionError("unreachable")

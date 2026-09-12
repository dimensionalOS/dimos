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

"""WebTransport client for the DimOS relay (robot leg, plus a test viewer),
and /api/info discovery of its WebTransport endpoint."""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator
import contextlib
from dataclasses import dataclass
import itertools
import json
import ssl
import time
from types import TracebackType
from typing import Any, cast
from urllib.parse import urljoin, urlparse
import urllib.request

from aioquic.asyncio.client import connect as aioquic_connect

from dimos.utils.logging_config import setup_logger
from dimos.web.relay_bridge._wt_session import SessionProtocol, make_quic_configuration
from dimos.web.relay_bridge.protocol import (
    CONTROL_CHANNEL,
    MAX_CONTROL_PAYLOAD_BYTES,
    PROTOCOL_VERSION,
    DataFrame,
    Delivery,
    FrameHeader,
    Hello,
    Msg,
    Ping,
    ProtocolError,
    RobotInfo,
    RobotManifest,
    Role,
    encode_datagram,
)

logger = setup_logger()

_LOOPBACK_HOSTS = frozenset({"127.0.0.1", "localhost", "::1"})

# aioquic never drops an unsendable datagram: max_datagram_size is fixed at
# 1200 B (no PMTUD) and _write_application retries _datagrams_pending[0]
# forever, so a datagram that cannot fit one packet (~1165 B encoded) wedges
# every datagram queued behind it - hello resends, pings, all send_control.
# Refuse to queue one; 1100 keeps margin under the real cliff. Robot hellos
# left datagrams in v5; this guards the viewer hello and send_control.
_DATAGRAM_MAX_BYTES = 1100


class RelayRejectedError(ProtocolError):
    """The relay explicitly rejected this client's hello handshake."""

    def __init__(self, code: str, message: str) -> None:
        self.code = code
        self.message = message
        super().__init__(f"relay rejected hello: {code}: {message}")


@dataclass(frozen=True)
class RelayInfo:
    """What GET /api/info advertises."""

    wt_url: str
    """WebTransport base URL (no path); clients append /robot or /viewer."""
    cert_hash: str | None
    """SHA-256 of the relay's ephemeral certificate; None once a relay serves
    a real certificate."""
    v: int


def resolve_info_url(base_url: str) -> str:
    """`<base_url>/api/info`, keeping a path prefix ("https://x/relay" ->
    "https://x/relay/api/info"); mirror of the SDK's resolveInfoUrl."""
    return urljoin(base_url if base_url.endswith("/") else base_url + "/", "api/info")


def _get_json(url: str, timeout: float, cafile: str | None) -> Any:
    context = ssl.create_default_context(cafile=cafile) if cafile is not None else None
    with urllib.request.urlopen(url, timeout=timeout, context=context) as response:
        body = response.read()
    try:
        return json.loads(body)
    except ValueError:
        return None  # reported as a shape problem by the caller


async def fetch_relay_info(
    base_url: str, *, timeout: float = 5.0, cafile: str | None = None
) -> RelayInfo:
    """Discover the relay's WebTransport endpoint through GET /api/info (the
    mirror of the SDK's fetchRelayInfo). A relay restart means a new QUIC
    port and certificate behind the same HTTP URL, so callers fetch on every
    connect. For an https base, `cafile` (a PEM CA bundle: mkcert, a private
    CA) replaces the system trust store. Raises OSError when the relay is
    unreachable, answers an HTTP error, or fails certificate verification
    (transient), ProtocolError for a non-relay answer or a protocol version
    mismatch.
    """
    url = resolve_info_url(base_url)
    data = await asyncio.to_thread(_get_json, url, timeout, cafile)
    if not isinstance(data, dict):
        raise ProtocolError(f"{url} returned an unexpected shape")
    wt_url, cert_hash, v = data.get("wtUrl"), data.get("certHash"), data.get("v")
    if (
        not isinstance(wt_url, str)
        or not (cert_hash is None or isinstance(cert_hash, str))
        or not isinstance(v, int)
        or isinstance(v, bool)
    ):
        raise ProtocolError(f"{url} returned an unexpected shape")
    if v != PROTOCOL_VERSION:
        raise ProtocolError(f"relay speaks protocol v{v}, this bridge speaks v{PROTOCOL_VERSION}")
    return RelayInfo(wt_url=wt_url, cert_hash=cert_hash, v=v)


class RelayClient:
    """One WebTransport session with the relay.

    Use :meth:`connect`; the constructor is internal. All methods must be
    called from the event loop that connected.
    """

    def __init__(self, url: str, role: Role, session: SessionProtocol, ctx: Any) -> None:
        self.url = url
        self.role = role
        self._session = session
        self._ctx = ctx  # aioquic.asyncio.connect() context manager
        self._ping_n = itertools.count(1)
        self._seq: dict[str, itertools.count[int]] = {}
        self._writers: list[LatestChannelWriter] = []
        self._close_task: asyncio.Task[None] | None = None

    @classmethod
    async def connect(
        cls,
        url: str,
        role: Role,
        *,
        insecure: bool | None = None,
        timeout: float = 10.0,
        cafile: str | None = None,
    ) -> RelayClient:
        """Connect to `url` (the relay's wtUrl, e.g. https://127.0.0.1:4433;
        the port defaults to 443).

        `insecure` skips certificate verification and defaults to True for
        loopback hosts only (the local relay uses an ephemeral self-signed
        cert). Passing insecure=True for a non-loopback host is refused.
        Otherwise aioquic checks the certificate against the URL host (DNS or
        IP SANs) and its chain against `cafile` (a PEM CA bundle: mkcert, a
        private CA), or certifi's bundle without one. `timeout` bounds the
        QUIC handshake and the WebTransport session setup separately.
        """
        parsed = urlparse(url)
        host = parsed.hostname
        if parsed.scheme != "https" or host is None:
            raise ValueError(f"relay URL must look like https://host[:port], got {url!r}")
        port = parsed.port if parsed.port is not None else 443
        is_loopback = host in _LOOPBACK_HOSTS
        if insecure is None:
            insecure = is_loopback
        if insecure and not is_loopback:
            raise ValueError(
                "insecure=True (trusting the relay's ephemeral self-signed certificate) is "
                f"only allowed for loopback hosts, got {host!r}; attaching from another host "
                "needs a relay with a real certificate"
            )
        expected_path = f"/{role}"
        if parsed.path in ("", "/"):
            path = expected_path
        elif parsed.path == expected_path:
            path = parsed.path
        else:
            raise ValueError(
                f"relay URL path must be {expected_path!r} for role={role}, got {parsed.path!r}"
            )

        ctx = aioquic_connect(
            host,
            port,
            configuration=make_quic_configuration(insecure, cafile),
            create_protocol=SessionProtocol,
        )
        # Bounded: aioquic gives up on an endpoint nobody listens on only at
        # its 60 s idle timeout (UDP surfaces no ICMP), and a stale wtUrl
        # after a relay restart must fail fast so the caller rediscovers.
        # aioquic's own finally closes the socket on cancellation.
        session = cast("SessionProtocol", await asyncio.wait_for(ctx.__aenter__(), timeout))
        # The robot leg's only incoming uni stream is the relay-opened control
        # carrier: corruption, reset, or an end of it must fail the whole
        # session (the bridge reconnects) instead of leaving it alive without
        # control. Viewer legs keep per-stream poisoning and routine ends.
        session.incoming_is_carrier = role == "robot"
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
        # Assign before the first await so concurrent watchdog/supervisor/
        # teardown callers share one context-manager exit. Shielding keeps a
        # cancelled caller from cancelling the shutdown itself; later callers
        # still await its completion (or receive its exception).
        if self._close_task is None:
            self._close_task = asyncio.create_task(self._close())
        await asyncio.shield(self._close_task)

    async def _close(self) -> None:
        for writer in self._writers:
            writer.stop()
        self._writers.clear()
        await self._ctx.__aexit__(None, None, None)

    async def hello(
        self,
        timeout: float = 5.0,
        *,
        robot: RobotInfo | None = None,
        manifest: RobotManifest | None = None,
    ) -> None:
        """Register with the relay; returns once its welcome datagram arrives.

        A robot hello carries identity and channel manifest as one @control
        data frame on a fresh one-shot bidi stream (v5); a viewer hello rides
        datagrams (the test viewer's control plane). The welcome datagram is
        lossy either way, so the hello repeats every 200 ms; a robot resend
        first retires the previous hello stream if it is still in flight.
        Raises ProtocolError if the encoded hello exceeds its transport
        budget, RelayRejectedError if the relay answers with an error
        (version mismatch, missing robot id, ...), TimeoutError if nothing
        answers within `timeout`.
        """
        msg = Hello(v=PROTOCOL_VERSION, role=self.role, robot=robot, manifest=manifest)
        control_payload: bytes | None = None
        if self.role == "robot":
            control_payload = encode_datagram(msg)
            if len(control_payload) > MAX_CONTROL_PAYLOAD_BYTES:
                raise ProtocolError(
                    f"hello control payload is {len(control_payload)} B "
                    f"(limit {MAX_CONTROL_PAYLOAD_BYTES}); trim the manifest"
                )
        else:
            size = len(encode_datagram(msg))
            if size > _DATAGRAM_MAX_BYTES:
                raise ProtocolError(
                    f"hello datagram is {size} B (limit {_DATAGRAM_MAX_BYTES}); an "
                    "oversized datagram wedges aioquic's whole datagram queue"
                )
        deadline = time.monotonic() + timeout
        hello_stream: int | None = None

        def retire_hello_stream() -> None:
            # In-flight check and reset in the same event-loop turn (the
            # aioquic-safe reset rule, web/README.md bug 9); a delivered
            # stream is left alone so a reset cannot destroy a hello the
            # relay has yet to read.
            if hello_stream is not None and self._session.stream_in_flight(hello_stream):
                self._session.reset_if_in_flight(hello_stream)

        try:
            while True:
                if control_payload is None:
                    self._session.send_msg(msg)
                else:
                    retire_hello_stream()
                    hello_stream = self.send_frame(CONTROL_CHANNEL, control_payload)
                with contextlib.suppress(asyncio.TimeoutError):
                    await asyncio.wait_for(self._session.welcomed.wait(), 0.2)
                if self._session.relay_error is not None:
                    err = self._session.relay_error
                    raise RelayRejectedError(err.code, err.message)
                if self._session.welcomed.is_set():
                    return
                if time.monotonic() >= deadline:
                    raise TimeoutError(
                        f"no welcome from relay within {timeout} s - possible protocol-version "
                        f"mismatch (this client speaks v{PROTOCOL_VERSION}; an older relay "
                        "ignores @control stream hellos); check the relay version"
                    )
        finally:
            # Every exit - welcome, rejection, timeout, cancellation - retires
            # a still-in-flight hello stream: hello() may run again on a live
            # client, and an abandoned stream would pin its buffered bytes and
            # stream state until the connection closes.
            retire_hello_stream()

    def send_control(self, msg: Msg) -> None:
        """Send one control message to the relay (datagram: lossy, ordered-less).

        Refuses a datagram over the packet-size cliff: aioquic would retry it
        forever and wedge every datagram queued behind it (the hello guard's
        rule, applied to the test viewer's whole control plane).
        """
        size = len(encode_datagram(msg))
        if size > _DATAGRAM_MAX_BYTES:
            raise ProtocolError(
                f"control datagram is {size} B (limit {_DATAGRAM_MAX_BYTES}); an "
                "oversized datagram wedges aioquic's whole datagram queue"
            )
        self._session.send_msg(msg)

    def send_control_frame(self, msg: Msg) -> int:
        """Send one @control frame on a fresh one-shot bidi stream: the
        reliable robot->relay control path (publish acks; hello wraps the
        same framing in its own retry/retire loop). Returns the stream id;
        raises ProtocolError past the control payload cap.
        """
        payload = encode_datagram(msg)
        if len(payload) > MAX_CONTROL_PAYLOAD_BYTES:
            raise ProtocolError(
                f"@control payload is {len(payload)} B (limit {MAX_CONTROL_PAYLOAD_BYTES})"
            )
        return self.send_frame(CONTROL_CHANNEL, payload)

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
        closed = asyncio.create_task(self._session.wait_closed())
        try:
            while True:
                get = asyncio.create_task(self._session.frames.get())
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

    async def control_messages(self) -> AsyncIterator[Msg | DataFrame]:
        """Control messages pushed by the relay (subs snapshots, robots, ...).

        Fed by relay datagrams and, on the robot leg, by the relay-opened
        control carrier: @control frames arrive decoded as messages, and
        forwarded publishes (tx channel data) arrive as raw DataFrames in the
        same order. Same contract as :meth:`frames`: buffered messages drain
        before the close is honored, and cancelling the consumer never
        orphans the queue getter. Ends when the session closes.
        """
        closed = asyncio.ensure_future(self._session.wait_closed())
        try:
            while True:
                get = asyncio.ensure_future(self._session.control_msgs.get())
                try:
                    await asyncio.wait({get, closed}, return_when=asyncio.FIRST_COMPLETED)
                    if not get.done():
                        return
                    msg = get.result()
                finally:
                    get.cancel()
                yield msg
        finally:
            closed.cancel()

    @property
    def frames_dropped(self) -> int:
        """Frames dropped locally because the consumer lagged (drop-oldest)."""
        return self._session.frames_dropped

    @property
    def control_dropped(self) -> int:
        """Control messages dropped locally under consumer lag (drop-oldest)."""
        return self._session.control_dropped


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
        self._mailbox: asyncio.Queue[tuple[bytes, dict[str, Any] | None, float | None]] = (
            asyncio.Queue(maxsize=1)
        )
        self._task = asyncio.create_task(self._pump())
        self._task.add_done_callback(self._on_pump_done)

    def _on_pump_done(self, task: asyncio.Future[None]) -> None:
        # Surface an unexpected pump death; a silent one would make offer()
        # accept frames that are never sent. Clean stop/close is not an error.
        if task.cancelled():
            return
        exc = task.exception()
        if exc is not None:
            logger.error(f"latest-wins writer for {self.ch} died", exc_info=exc)

    def offer(
        self, payload: bytes, meta: dict[str, Any] | None = None, ts: float | None = None
    ) -> None:
        """Queue the newest frame, dropping any not-yet-sent predecessor.

        `ts` overrides the frame-header timestamp (a replayed frame carries
        its source time); None stamps send time. Event-loop only; producers
        on other threads must go through
        `loop.call_soon_threadsafe(writer.offer, ...)`. Raises RuntimeError if
        the pump is no longer running (session closed, stopped, or died) so a
        dead channel is visible at the producer instead of silently dropping.
        """
        if self._task.done():
            raise RuntimeError(f"latest-wins writer for {self.ch} is not running")
        if self._mailbox.full():
            self._mailbox.get_nowait()
            self.dropped += 1
        self._mailbox.put_nowait((payload, meta, ts))

    def stop(self) -> None:
        self._task.cancel()

    async def _pump(self) -> None:
        session = self._client._session
        closed = asyncio.create_task(session.closed.wait())
        try:
            while not session.closed.is_set():
                get = asyncio.create_task(self._mailbox.get())
                try:
                    await asyncio.wait({get, closed}, return_when=asyncio.FIRST_COMPLETED)
                    if not get.done():
                        break  # session closed with an empty mailbox
                    payload, meta, ts = get.result()
                finally:
                    get.cancel()
                if session.closed.is_set():
                    break
                stream_id = self._client.send_frame(
                    self.ch, payload, delivery="latest", meta=meta, ts=ts
                )
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

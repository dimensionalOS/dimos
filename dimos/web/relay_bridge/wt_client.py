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
from collections import deque
from collections.abc import AsyncIterator
import contextlib
from dataclasses import dataclass, replace
import itertools
import json
import socket
import ssl
import time
from types import TracebackType
from typing import Any
from urllib.parse import urljoin, urlparse
import urllib.request

from aioquic.quic.configuration import QuicConfiguration
from aioquic.quic.connection import QuicConnection

from dimos.utils.logging_config import setup_logger
from dimos.web.relay_bridge._wt_session import SessionProtocol, make_quic_configuration
from dimos.web.relay_bridge.pacing import passes_rate_gate
from dimos.web.relay_bridge.protocol import (
    CONTROL_CHANNEL,
    MAX_CONTROL_PAYLOAD_BYTES,
    MAX_TOKEN_LEN,
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

# Application keepalive (start_keepalive): a datagram ping every interval,
# and the session fails after this many unanswered in a row. Detects a dead
# or wedged relay in ~9 s where the QUIC idle timeout (IDLE_TIMEOUT_S) is
# the backstop, and keeps a subscriber-less session's traffic flowing in
# both directions.
KEEPALIVE_INTERVAL_S = 3.0
KEEPALIVE_MISSES = 3

# Latest streams in flight per channel before the writer waits for an ACK.
# With one, every frame waits for the previous frame's QUIC ACK (a relay
# round trip plus up to 25 ms of delayed ACK), so the rate caps at
# 1 / (RTT + ~14 ms): ~70 fps on loopback, ~15 fps to a relay 50 ms away.
# Three cover 30 fps at a 50 ms RTT. Each stream is independent and viewers
# keep the newest frame by seq, so the relay needs nothing.
MAX_IN_FLIGHT = 3
# Unacknowledged bytes a reliable channel's persistent stream may hold
# before the bridge resets it (dropping what it held, counted in
# `reliable_resets` and logged) and continues on a fresh stream: aioquic
# buffers without bound, so on a stalled relay the backlog would otherwise
# grow until the session dies. The relay's own per-viewer reliable queue cap.
RELIABLE_MAX_UNACKED_BYTES = 16 * 1024 * 1024
_RELIABLE_RESET_LOG_S = 5.0


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
    rtc: bool = False
    """True when the relay brokers video.webrtc.v1 track channels through
    its Cloudflare Realtime SFU (protocol v7)."""


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
    rtc = data.get("rtc", False)
    if (
        not isinstance(wt_url, str)
        or not (cert_hash is None or isinstance(cert_hash, str))
        or not isinstance(v, int)
        or isinstance(v, bool)
        or not isinstance(rtc, bool)
    ):
        raise ProtocolError(f"{url} returned an unexpected shape")
    if v != PROTOCOL_VERSION:
        raise ProtocolError(f"relay speaks protocol v{v}, this bridge speaks v{PROTOCOL_VERSION}")
    return RelayInfo(wt_url=wt_url, cert_hash=cert_hash, v=v, rtc=rtc)


@contextlib.asynccontextmanager
async def _quic_connect(
    host: str, port: int, *, configuration: QuicConfiguration
) -> AsyncIterator[SessionProtocol]:
    """aioquic.asyncio.client.connect() with a socket of the server's address family.

    aioquic's dual-stack [::]:0 socket can get a port an IPv4 UDP socket holds
    on macOS, and that socket then receives the IPv4 server's replies."""
    loop = asyncio.get_running_loop()
    family, _, _, _, addr = (await loop.getaddrinfo(host, port, type=socket.SOCK_DGRAM))[0]
    connection = QuicConnection(configuration=replace(configuration, server_name=host))
    # Connected socket: an ICMP port-unreachable (the relay process died)
    # then surfaces as error_received instead of being invisible until the
    # idle timeout.
    transport, protocol = await loop.create_datagram_endpoint(
        lambda: SessionProtocol(connection), family=family, remote_addr=(addr[0], addr[1])
    )
    try:
        protocol.connect(addr)
        await protocol.wait_connected()
        yield protocol
    finally:
        protocol.close()
        await protocol.wait_closed()
        transport.close()


class RelayClient:
    """One WebTransport session with the relay.

    Use :meth:`connect`; the constructor is internal. All methods must be
    called from the event loop that connected.
    """

    def __init__(self, url: str, role: Role, session: SessionProtocol, ctx: Any) -> None:
        self.url = url
        self.role = role
        self._session = session
        self._ctx = ctx  # _quic_connect() context manager
        self._ping_n = itertools.count(1)
        self._seq: dict[str, itertools.count[int]] = {}
        # Persistent reliable channel streams reset to hold the byte bound,
        # per channel.
        self.reliable_resets: dict[str, int] = {}
        self._reliable_reset_logged: dict[str, float] = {}
        self._writers: list[LatestChannelWriter] = []
        self._close_task: asyncio.Task[None] | None = None
        self._keepalive_task: asyncio.Task[None] | None = None

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

        ctx = _quic_connect(host, port, configuration=make_quic_configuration(insecure, cafile))
        # Bounded: aioquic gives up on an endpoint nobody listens on only at
        # its 60 s idle timeout (UDP surfaces no ICMP), and a stale wtUrl
        # after a relay restart must fail fast so the caller rediscovers.
        # _quic_connect's finally closes the socket on cancellation.
        session = await asyncio.wait_for(ctx.__aenter__(), timeout)
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
        if self._keepalive_task is not None:
            self._keepalive_task.cancel()
        await self._ctx.__aexit__(None, None, None)

    def start_keepalive(self) -> None:
        """Ping the relay every KEEPALIVE_INTERVAL_S and fail the session
        after KEEPALIVE_MISSES unanswered pings in a row. Call once hello
        succeeded (the relay closes a viewer that sends anything else first).
        Idempotent; stops with close()."""
        if self._keepalive_task is None:
            self._keepalive_task = asyncio.create_task(self._keepalive())

    async def _keepalive(self) -> None:
        loop = asyncio.get_running_loop()
        misses = 0
        while not self._session.closed.is_set():
            started = loop.time()
            try:
                await self.ping(timeout=KEEPALIVE_INTERVAL_S)
                misses = 0
            except asyncio.TimeoutError:
                misses += 1
                if misses >= KEEPALIVE_MISSES:
                    self._session.fail_session(f"{misses} keepalive pings unanswered")
                    self._session.transmit()
                    return
            await asyncio.sleep(max(0.0, started + KEEPALIVE_INTERVAL_S - loop.time()))

    async def hello(
        self,
        timeout: float = 5.0,
        *,
        robot: RobotInfo | None = None,
        manifest: RobotManifest | None = None,
        token: str | None = None,
    ) -> None:
        """Register with the relay; returns once its welcome datagram arrives.

        A robot hello carries identity and channel manifest as one @control
        data frame on a fresh one-shot bidi stream (v5); a viewer hello rides
        datagrams (the test viewer's control plane). The welcome datagram is
        lossy either way, so the hello repeats every 200 ms; a robot resend
        first retires the previous hello stream if it is still in flight.
        `token` is the robot key or viewer token for a relay started with
        --auth-file (the relay answers auth_failed without a valid one).
        Raises ProtocolError if the encoded hello exceeds its transport
        budget, RelayRejectedError if the relay answers with an error
        (version mismatch, missing robot id, auth_failed, ...), TimeoutError
        if nothing answers within `timeout`.
        """
        if token is not None and len(token) > MAX_TOKEN_LEN:
            # Before the model: pydantic's error would quote the value.
            raise ProtocolError(f"hello token is {len(token)} characters (limit {MAX_TOKEN_LEN})")
        msg = Hello(v=PROTOCOL_VERSION, role=self.role, robot=robot, manifest=manifest, token=token)
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
            # aioquic-safe reset rule, docs/web/protocol.md workaround 9); a delivered
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
        """Send one data frame. Returns the stream id.

        A reliable data channel's frames ride its persistent bidi stream,
        back to back and in order; latest and @control frames each get a
        one-shot stream. The frame is buffered by aioquic and delivered in
        the background; senders that need pacing can `await
        wait_delivered(...)`. A reliable channel's stream holding more than
        RELIABLE_MAX_UNACKED_BYTES is reset (the viewer sees a seq gap),
        counted in `reliable_resets` and logged, and the frame opens a fresh
        one.
        """
        seq = next(self._seq.setdefault(ch, itertools.count()))
        header = FrameHeader(
            ch=ch, seq=seq, ts=time.time() if ts is None else ts, delivery=delivery, meta=meta
        )
        if delivery != "reliable" or ch == CONTROL_CHANNEL:
            return self._session.send_frame(header, payload)
        stream_id = self._session.channel_stream(ch)
        if (
            stream_id is not None
            and self._session.stream_unacked_bytes(stream_id) + len(payload)
            > RELIABLE_MAX_UNACKED_BYTES
        ):
            # Membership check and reset in one event-loop turn (workaround 9).
            self._session.reset_if_in_flight(stream_id)
            self._session.drop_channel_stream(ch)
            self.reliable_resets[ch] = self.reliable_resets.get(ch, 0) + 1
            if passes_rate_gate(
                self._reliable_reset_logged, ch, time.monotonic(), _RELIABLE_RESET_LOG_S
            ):
                logger.warning(
                    f"reliable channel {ch}: over {RELIABLE_MAX_UNACKED_BYTES} B unacknowledged, "
                    f"resetting its stream ({self.reliable_resets[ch]} resets so far)"
                )
        return self._session.send_frame_on(ch, header, payload)

    async def wait_delivered(self, stream_id: int, timeout: float = 10.0) -> bool:
        """True once the frame on `stream_id` is fully ACKed."""
        return await self._session.wait_delivered(stream_id, timeout)

    def latest_writer(
        self, ch: str, *, stale_after: float = 0.5, max_in_flight: int = MAX_IN_FLIGHT
    ) -> LatestChannelWriter:
        """Delivery-paced latest-wins writer for `ch` (e.g. camera frames)."""
        writer = LatestChannelWriter(self, ch, stale_after=stale_after, max_in_flight=max_in_flight)
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
    """Latest-wins channel writer: a 1-slot pending frame and a delivery-paced
    sender with `max_in_flight` streams on the wire.

    `offer()` never blocks; a frame offered while the pending slot is taken
    replaces it (counted in `dropped`). Frames go out at once while fewer than
    `max_in_flight` streams are unACKed; past that the pump waits for ACK
    progress, and once the oldest in-flight stream is `stale_after` old with
    a newer frame waiting it is reset to make room (receivers discard the
    partial frame).
    """

    def __init__(
        self,
        client: RelayClient,
        ch: str,
        *,
        stale_after: float,
        max_in_flight: int = MAX_IN_FLIGHT,
    ) -> None:
        if max_in_flight < 1:
            raise ValueError(f"max_in_flight must be >= 1, got {max_in_flight}")
        self._client = client
        self.ch = ch
        self.stale_after = stale_after
        self.max_in_flight = max_in_flight
        self.dropped = 0
        self.sent = 0
        self.resets = 0
        self._pending: tuple[bytes, dict[str, Any] | None, float | None] | None = None
        self._offered = asyncio.Event()
        # (stream id, send time) in send order; pruned as ACKs land.
        self._in_flight: deque[tuple[int, float]] = deque()
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
        if self._pending is not None:
            self.dropped += 1
        self._pending = (payload, meta, ts)
        self._offered.set()

    def stop(self) -> None:
        self._task.cancel()

    def in_flight(self) -> int:
        """Streams sent and not yet ACKed (or reset)."""
        session = self._client._session
        self._in_flight = deque(e for e in self._in_flight if session.stream_in_flight(e[0]))
        return len(self._in_flight)

    async def _pump(self) -> None:
        session = self._client._session
        closed = asyncio.create_task(session.closed.wait())
        try:
            while not session.closed.is_set():
                if self._pending is None:
                    offered = asyncio.create_task(self._offered.wait())
                    try:
                        await asyncio.wait({offered, closed}, return_when=asyncio.FIRST_COMPLETED)
                    finally:
                        offered.cancel()
                    self._offered.clear()
                    continue
                if self.in_flight() < self.max_in_flight:
                    payload, meta, ts = self._pending
                    self._pending = None
                    stream_id = self._client.send_frame(
                        self.ch, payload, delivery="latest", meta=meta, ts=ts
                    )
                    self.sent += 1
                    self._in_flight.append((stream_id, time.monotonic()))
                    continue
                # The budget is full and a newer frame waits: an ACK frees a
                # slot, or the oldest stream is reset once stale. The
                # membership recheck inside reset_if_in_flight shares this
                # event-loop turn (required, see docs/web/protocol.md).
                oldest_id, sent_at = self._in_flight[0]
                remaining = sent_at + self.stale_after - time.monotonic()
                if remaining <= 0:
                    self._in_flight.popleft()
                    if session.reset_if_in_flight(oldest_id):
                        self.resets += 1
                    continue
                await session.wait_delivery_progress(remaining)
        finally:
            closed.cancel()
        logger.info(f"latest-wins writer for {self.ch}: session closed, stopping")

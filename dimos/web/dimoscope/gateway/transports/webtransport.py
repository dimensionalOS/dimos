#!/usr/bin/env python3
# WebTransport (HTTP/3 / QUIC) transport: small frames on datagrams (unreliable, no head-of-line
# blocking), big frames on a unidirectional stream (reliable). Own UDP port (QUIC can't share the HTTP
# TCP port). Optional (needs aioquic + cryptography). Fed from the shared Bus.
#
# Read-path (subs/rate/priority) mirrors the /ws data plane over a browser-opened bidirectional control
# stream: subscribe/unsubscribe/rate/list in, hello/topic out, drained through the shared PriorityOutbox.
# Teleop/goal/rpc stay on /ws.
from __future__ import annotations

import asyncio
import json
import time

from dimos.utils.logging_config import setup_logger

from ..bus import Bus, Sample
from ..egress import DEFAULT_TTL, SafetyEgress
from ..qos import PriorityOutbox, declared_to_class, default_priority
from ._common import frame, wants

logger = setup_logger()

DATAGRAM_MAX = 1100  # QUIC datagrams are path-MTU-capped; bigger frames go on a stream

try:
    import datetime
    import hashlib
    import ipaddress
    import tempfile

    from aioquic.asyncio import QuicConnectionProtocol, serve as quic_serve
    from aioquic.h3.connection import H3_ALPN, H3Connection
    from aioquic.h3.events import H3Event, HeadersReceived, WebTransportStreamDataReceived
    from aioquic.quic.configuration import QuicConfiguration
    from aioquic.quic.events import ConnectionTerminated, ProtocolNegotiated, QuicEvent
    from cryptography import x509
    from cryptography.hazmat.primitives import hashes, serialization
    from cryptography.hazmat.primitives.asymmetric import ec
    from cryptography.x509.oid import NameOID

    HAS_WEBTRANSPORT = True
except Exception:  # pragma: no cover
    HAS_WEBTRANSPORT = False


def _make_cert() -> tuple[str, str, str]:
    """Self-signed ECDSA P-256 cert (≤10 days) so the browser accepts it via serverCertificateHashes."""
    key = ec.generate_private_key(ec.SECP256R1())
    name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, "localhost")])
    now = datetime.datetime.now(datetime.timezone.utc)
    cert = (
        x509.CertificateBuilder()
        .subject_name(name)
        .issuer_name(name)
        .public_key(key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(now - datetime.timedelta(days=1))
        .not_valid_after(now + datetime.timedelta(days=10))
        .add_extension(
            x509.SubjectAlternativeName(
                [x509.DNSName("localhost"), x509.IPAddress(ipaddress.ip_address("127.0.0.1"))]
            ),
            critical=False,
        )
        .sign(key, hashes.SHA256())
    )
    cf = tempfile.NamedTemporaryFile(suffix=".pem", delete=False)
    kf = tempfile.NamedTemporaryFile(suffix=".pem", delete=False)
    cf.write(cert.public_bytes(serialization.Encoding.PEM))
    kf.write(
        key.private_bytes(
            serialization.Encoding.PEM,
            serialization.PrivateFormat.PKCS8,
            serialization.NoEncryption(),
        )
    )
    cf.close()
    kf.close()
    sha = hashlib.sha256(cert.public_bytes(serialization.Encoding.DER)).hexdigest()
    return cf.name, kf.name, sha


class WebTransportServer:
    """QUIC server on its own UDP port, fed from the Bus. Per-session read-path (subs/rate/priority
    outbox) mirrors the /ws data plane; write-path (teleop/goal/rpc) is delegated to the shared
    SafetyEgress (same clamp/deadman/whitelist as /ws)."""

    def __init__(self, bus: Bus, egress: SafetyEgress) -> None:
        self.bus = bus
        self.egress = egress  # same SafetyEgress instance as the /ws plane
        self.sessions: set = set()
        self.cert_hash = ""
        bus.subscribe(self._on_sample)
        bus.on_new_topic(self._on_new_topic)

    # bus fan-out (loop thread, keep cheap)
    def _on_sample(self, s: Sample) -> None:
        if not self.sessions:
            return
        now = time.time() * 1000.0
        default = default_priority(s.topic, s.type)  # server default; clients may override per sub
        f: bytes | None = None
        for sess in list(self.sessions):
            if not wants(sess.subs, s.topic):
                continue  # on-demand: unsubscribed topics never transit
            hz = sess.rate.get(s.topic, 0)
            if hz > 0:
                if now - sess.last.get(s.topic, 0) < 1000.0 / hz:
                    continue  # per-topic downsample
                sess.last[s.topic] = now
            if sess.outbox is None:
                continue
            if f is None:
                f = frame(s)  # [f64 gateway-send-ms][LC02], stamped once
            prio, conflate, depth = sess.qos.get(s.topic, default)  # client override else default
            sess.outbox.put_data(s.topic, prio, conflate, depth, f)  # priority outbox; never blocks

    def _on_new_topic(self, topic: str, typ: str) -> None:
        for sess in list(self.sessions):
            sess.send_control(
                {"op": "topic", "topic": topic, "type": typ}
            )  # no-op until ctl stream up

    async def start(self, host: str, port: int) -> None:
        if not HAS_WEBTRANSPORT:
            logger.warning("aioquic not available — WebTransport disabled")
            return
        certfile, keyfile, self.cert_hash = _make_cert()
        config = QuicConfiguration(
            is_client=False, alpn_protocols=H3_ALPN, max_datagram_frame_size=65536
        )
        config.load_cert_chain(certfile, keyfile)
        server = self
        sessions = self.sessions

        class _Protocol(QuicConnectionProtocol):
            def __init__(self, *a, **k) -> None:
                super().__init__(*a, **k)
                self._http: H3Connection | None = None
                self._session_id: int | None = None
                # per-session read-path state (mirror data.py::_Client)
                self.subs: set[str] = set()
                self.rate: dict[str, float] = {}
                self.last: dict[str, float] = {}
                self.qos: dict[str, tuple] = {}
                self.outbox: PriorityOutbox | None = None
                self.ctl_stream_id: int | None = None
                self._ctl_buf = b""
                self._drain_task: asyncio.Future | None = None
                self._rpc_tasks: set = set()  # keep refs so scheduled rpc coros aren't GC'd

            def quic_event_received(self, event: QuicEvent) -> None:
                if isinstance(event, ProtocolNegotiated):
                    self._http = H3Connection(self._quic, enable_webtransport=True)
                elif isinstance(event, ConnectionTerminated):
                    sessions.discard(self)
                    server.egress.disconnect(self)  # safety: cancel deadman + stop the robot
                    if self._drain_task is not None:
                        self._drain_task.cancel()
                if self._http is not None:
                    for h3 in self._http.handle_event(event):
                        self._h3(h3)

            def _h3(self, event: H3Event) -> None:
                if isinstance(event, HeadersReceived):
                    h = dict(event.headers)
                    if h.get(b":method") == b"CONNECT" and h.get(b":protocol") == b"webtransport":
                        self._session_id = event.stream_id
                        self._http.send_headers(
                            stream_id=event.stream_id,
                            headers=[
                                (b":status", b"200"),
                                (b"sec-webtransport-http3-draft", b"draft02"),
                            ],
                        )
                        self.outbox = PriorityOutbox()
                        self._drain_task = asyncio.ensure_future(self._drain())
                        sessions.add(self)
                        self.transmit()
                elif isinstance(event, WebTransportStreamDataReceived):
                    # browser's bidi control stream: newline-delimited JSON ops (client {op:"list"} → hello reply)
                    self.ctl_stream_id = event.stream_id
                    self._ctl_buf += event.data
                    while b"\n" in self._ctl_buf:
                        line, self._ctl_buf = self._ctl_buf.split(b"\n", 1)
                        if not line.strip():
                            continue
                        try:
                            self._on_control(json.loads(line))
                        except (ValueError, KeyError):
                            pass
                    if len(self._ctl_buf) > 65536:
                        self._ctl_buf = b""  # runaway non-newline-terminated input → drop

            def _on_control(self, m: dict) -> None:
                op = m.get("op")
                if op == "subscribe":
                    t = m["topic"]
                    self.subs.add(t)
                    if m.get("maxHz"):
                        self.rate[t] = float(m["maxHz"])
                    self._apply_qos(t, m)
                elif op == "unsubscribe":
                    t = m["topic"]
                    self.subs.discard(t)
                    self.rate.pop(t, None)
                    self.last.pop(t, None)
                    self.qos.pop(t, None)
                elif op == "rate":
                    self.rate[m["topic"]] = float(m.get("maxHz") or 0)
                elif op == "list":
                    self.send_control(
                        {
                            "op": "hello",
                            "topics": server.bus.topic_list(),
                            "label": "dimoscope/WT",
                            "rpc": server.egress.commands,
                        }
                    )
                # write-path control → the shared SafetyEgress (same clamp/deadman/whitelist as /ws)
                elif op == "teleop":
                    server.egress.teleop(
                        self,
                        float(m.get("linearX", 0)),
                        float(m.get("angularZ", 0)),
                        float(m.get("ttlMs", DEFAULT_TTL)),
                        asyncio.get_running_loop(),
                    )
                elif op == "stop":
                    server.egress.stop(self)
                elif op == "goal":
                    server.egress.goal(m.get("x", 0), m.get("y", 0), m.get("z", 0))
                elif op == "rpc":  # async → schedule, reply on the ctl stream; keep a ref (RUF006)
                    task = asyncio.ensure_future(self._do_rpc(m))
                    self._rpc_tasks.add(task)
                    task.add_done_callback(self._rpc_tasks.discard)

            async def _do_rpc(self, m: dict) -> None:
                res = await server.egress.rpc(
                    m.get("target"),
                    m.get("method"),
                    m.get("args") or [],
                    asyncio.get_running_loop(),
                )
                self.send_control({"op": "rpc-res", "id": m.get("id"), **res})

            def _apply_qos(self, topic: str, m: dict) -> None:
                if any(m.get(k) is not None for k in ("priority", "reliability", "depth")):
                    default = default_priority(topic, server.bus.topics.get(topic, ""))
                    self.qos[topic] = declared_to_class(
                        m.get("priority"), m.get("reliability"), m.get("depth"), default
                    )
                else:
                    self.qos.pop(topic, None)

            def send_control(self, obj: dict) -> None:
                if self._http is None or self.ctl_stream_id is None:
                    return
                try:
                    self._quic.send_stream_data(
                        self.ctl_stream_id, (json.dumps(obj) + "\n").encode(), end_stream=False
                    )
                    self.transmit()
                except Exception:
                    pass

            async def _drain(self) -> None:
                try:
                    while True:
                        item = await self.outbox.get()
                        self.send_frame(item)
                except asyncio.CancelledError:
                    pass

            def send_frame(self, payload: bytes) -> None:
                if self._http is None or self._session_id is None:
                    return
                try:
                    if len(payload) <= DATAGRAM_MAX:
                        self._http.send_datagram(self._session_id, payload)  # unreliable, no HoL
                    else:
                        sid = self._http.create_webtransport_stream(
                            self._session_id, is_unidirectional=True
                        )
                        self._quic.send_stream_data(sid, payload, end_stream=True)  # reliable
                    self.transmit()
                except Exception:
                    sessions.discard(self)

        # Dual-stack the IPv4 wildcard: on macOS `localhost` resolves to ::1 (IPv6) first, and QUIC/UDP
        # has no Happy-Eyeballs fallback, so an IPv4-only 0.0.0.0 bind gives the browser a bare
        # ERR_CONNECTION_REFUSED on [::1]:port. Binding :: (V6ONLY=0 default) serves both families;
        # explicit HOST overrides are honoured verbatim.
        bind_host = "::" if host == "0.0.0.0" else host
        await quic_serve(bind_host, port, configuration=config, create_protocol=_Protocol)
        logger.info(
            "webtransport up",
            url=f"https://{host}:{port}",
            transport="QUIC/UDP",
            cert_sha256=self.cert_hash[:16],
        )

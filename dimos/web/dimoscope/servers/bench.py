#!/usr/bin/env python3
# dimoscope research transports — SSE, HTTP long-poll, WebRTC DataChannel, WebTransport (QUIC).
#
# These exist to BENCHMARK delivery mechanisms against the default WebSocket data plane (e.g. TCP
# head-of-line blocking under loss). They were standalone re-transmitters that subscribed to the
# gateway over a WS hop; here they tap the shared Bus directly — same [f64 send-ms][LC02] frames, so
# the browser decodes every path through the identical frameToSample. All are READ-ONLY (control stays
# on the data WS, the safety boundary). They mount on distinct paths and all run at once, no flags.
from __future__ import annotations

import asyncio
import base64
from collections import deque
import struct
import time

from starlette.requests import Request
from starlette.responses import Response, StreamingResponse

from .bus import Bus, Sample

POLL_RING_MAX = 8192
POLL_WAIT_S = 10.0
DATAGRAM_MAX = 1100  # QUIC datagrams are path-MTU-capped; bigger frames go on a stream


def _frame(s: Sample) -> bytes:
    return struct.pack(">d", time.time() * 1000.0) + s.lc02  # [f64 gateway-send-ms][LC02]


def _wants(subs: set[str], topic: str) -> bool:
    return "*" in subs or topic in subs


def _subs_from_query(tp: str | None) -> set[str]:
    if tp is None or tp == "*":
        return {"*"}
    return {t for t in tp.split(",") if t}


# ── SSE ─────────────────────────────────────────────────────────────────────
class _SseClient:
    __slots__ = ("q", "subs")

    def __init__(self, subs: set[str]) -> None:
        self.q: asyncio.Queue = asyncio.Queue(maxsize=4096)
        self.subs = subs


class SsePlane:
    def __init__(self, bus: Bus) -> None:
        self.bus = bus
        self.clients: set[_SseClient] = set()
        bus.subscribe(self._on_sample)

    def _on_sample(self, s: Sample) -> None:
        if not self.clients:
            return
        line: bytes | None = None
        for c in self.clients:
            if not _wants(c.subs, s.topic):
                continue
            if line is None:
                line = b"data: " + base64.b64encode(_frame(s)) + b"\n\n"
            try:
                c.q.put_nowait(line)
            except asyncio.QueueFull:
                pass

    async def handle(self, request: Request) -> StreamingResponse:
        import json

        subs = _subs_from_query(request.query_params.get("topics"))
        client = _SseClient(subs)
        self.clients.add(client)
        hello = (
            "event: hello\ndata: "
            + json.dumps({"topics": self.bus.topic_list(), "label": "dimoscope/SSE"})
            + "\n\n"
        ).encode()

        async def gen():
            yield hello
            try:
                while True:
                    yield await client.q.get()
            finally:
                self.clients.discard(client)

        return StreamingResponse(
            gen(),
            media_type="text/event-stream",
            headers={"cache-control": "no-cache", "access-control-allow-origin": "*"},
        )


# ── HTTP long-poll ───────────────────────────────────────────────────────────
class PollPlane:
    def __init__(self, bus: Bus) -> None:
        self.bus = bus
        self.ring: deque = deque(maxlen=POLL_RING_MAX)  # (id, topic, frame)
        self.seq = 0
        self._waiters: set[asyncio.Future] = set()
        bus.subscribe(self._on_sample)

    def _on_sample(self, s: Sample) -> None:
        self.seq += 1
        self.ring.append((self.seq, s.topic, _frame(s)))
        for f in self._waiters:
            if not f.done():
                f.set_result(None)
        self._waiters.clear()

    def _collect(self, since: int, subs: set[str], max_n: int) -> list:
        out = []
        for eid, topic, frame in self.ring:
            if eid <= since or not _wants(subs, topic):
                continue
            out.append((eid, frame))
            if len(out) >= max_n:
                break
        return out

    async def handle(self, request: Request) -> Response:
        since = int(request.query_params.get("since", 0))
        subs = _subs_from_query(request.query_params.get("topics"))
        max_n = int(request.query_params.get("max", 512))
        hdr = {"content-type": "application/octet-stream", "access-control-allow-origin": "*"}
        # since < 0 → "start from head": skip history so a live client only gets new frames.
        if since < 0:
            head = self.ring[-1][0] if self.ring else 0
            return Response(struct.pack(">II", head, 0), headers=hdr)
        batch = self._collect(since, subs, max_n)
        if not batch:
            fut = asyncio.get_running_loop().create_future()
            self._waiters.add(fut)
            try:
                await asyncio.wait_for(fut, POLL_WAIT_S)
            except asyncio.TimeoutError:
                pass
            finally:
                self._waiters.discard(fut)
            batch = self._collect(since, subs, max_n)
        new_cursor = batch[-1][0] if batch else since
        body = bytearray(struct.pack(">II", new_cursor, len(batch)))
        for _eid, frame in batch:
            body += struct.pack(">I", len(frame)) + frame
        return Response(bytes(body), headers=hdr)


# ── WebRTC DataChannel ────────────────────────────────────────────────────────
try:
    from aiortc import RTCPeerConnection, RTCSessionDescription

    HAS_WEBRTC_DATA = True
except Exception:  # pragma: no cover
    HAS_WEBRTC_DATA = False


async def _await_ice(pc) -> None:
    """Block until ICE gathering completes (non-trickle). `fut` is function-local, so the
    icegatheringstatechange closure binds it cleanly (no loop-variable capture)."""
    if pc.iceGatheringState == "complete":
        return
    fut = asyncio.get_running_loop().create_future()

    @pc.on("icegatheringstatechange")
    def _on_change() -> None:
        if pc.iceGatheringState == "complete" and not fut.done():
            fut.set_result(None)

    await fut


class WebRtcDataPlane:
    def __init__(self, bus: Bus) -> None:
        self.bus = bus
        self.channels: set = set()
        self.pcs: dict = {}
        bus.subscribe(self._on_sample)

    def _on_sample(self, s: Sample) -> None:
        if not self.channels:
            return
        f = _frame(s)
        for ch in list(self.channels):
            try:
                if ch.readyState == "open":
                    ch.send(f)
            except Exception:
                self.channels.discard(ch)

    async def handle(self, ws) -> None:
        import json

        from fastapi import WebSocketDisconnect

        await ws.accept()
        if not HAS_WEBRTC_DATA:
            await ws.close()
            return
        pc = RTCPeerConnection()
        self.pcs[ws] = pc

        @pc.on("datachannel")
        def on_datachannel(channel) -> None:
            self.channels.add(channel)

            @channel.on("close")
            def _on_close() -> None:
                self.channels.discard(channel)

        try:
            while True:
                m = json.loads(await ws.receive_text())
                if m.get("op") == "offer":
                    await pc.setRemoteDescription(RTCSessionDescription(sdp=m["sdp"], type="offer"))
                    await pc.setLocalDescription(await pc.createAnswer())
                    await _await_ice(pc)  # non-trickle: gather then answer (instant on a LAN)
                    await ws.send_text(json.dumps({"op": "answer", "sdp": pc.localDescription.sdp}))
        except (WebSocketDisconnect, json.JSONDecodeError, RuntimeError):
            pass
        finally:
            await pc.close()
            self.pcs.pop(ws, None)


# ── WebTransport (HTTP/3 / QUIC) ──────────────────────────────────────────────
try:
    import datetime
    import hashlib
    import ipaddress
    import tempfile

    from aioquic.asyncio import QuicConnectionProtocol, serve as quic_serve
    from aioquic.h3.connection import H3_ALPN, H3Connection
    from aioquic.h3.events import H3Event, HeadersReceived
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
    """QUIC server on its own UDP port (QUIC can't share the HTTP TCP port); fed from the Bus."""

    def __init__(self, bus: Bus) -> None:
        self.bus = bus
        self.sessions: set = set()
        self.cert_hash = ""
        bus.subscribe(self._on_sample)

    def _on_sample(self, s: Sample) -> None:
        if not self.sessions:
            return
        f = _frame(s)
        for sess in list(self.sessions):
            sess.send_frame(f)

    async def start(self, host: str, port: int) -> None:
        if not HAS_WEBTRANSPORT:
            print("[webtransport] aioquic not available — disabled", flush=True)
            return
        certfile, keyfile, self.cert_hash = _make_cert()
        config = QuicConfiguration(
            is_client=False, alpn_protocols=H3_ALPN, max_datagram_frame_size=65536
        )
        config.load_cert_chain(certfile, keyfile)
        sessions = self.sessions

        class _Protocol(QuicConnectionProtocol):
            def __init__(self, *a, **k) -> None:
                super().__init__(*a, **k)
                self._http: H3Connection | None = None
                self._session_id: int | None = None

            def quic_event_received(self, event: QuicEvent) -> None:
                if isinstance(event, ProtocolNegotiated):
                    self._http = H3Connection(self._quic, enable_webtransport=True)
                elif isinstance(event, ConnectionTerminated):
                    sessions.discard(self)
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
                        sessions.add(self)
                        self.transmit()

            def send_frame(self, frame: bytes) -> None:
                if self._http is None or self._session_id is None:
                    return
                try:
                    if len(frame) <= DATAGRAM_MAX:
                        self._http.send_datagram(self._session_id, frame)  # unreliable, no HoL
                    else:
                        sid = self._http.create_webtransport_stream(
                            self._session_id, is_unidirectional=True
                        )
                        self._quic.send_stream_data(sid, frame, end_stream=True)  # reliable
                    self.transmit()
                except Exception:
                    sessions.discard(self)

        await quic_serve(host, port, configuration=config, create_protocol=_Protocol)
        print(
            f"[webtransport] https://{host}:{port} (QUIC/UDP) · cert-sha256={self.cert_hash[:16]}…",
            flush=True,
        )

#!/usr/bin/env python3
# WebTransport (HTTP/3 / QUIC) delivery for the data-path benchmark.
#
# A thin re-transmitter in FRONT of the existing gateway (mirrors servers/webrtc_data.py): it
# subscribes to gateway.ts over WebSocket (the same [f64 gateway-send-ms][LC02] frames) and fans each
# out over a WebTransport session — so the browser decodes via the identical frameToSample, no new wire
# format. WebTransport rides QUIC/UDP, so unlike WebSocket (TCP) it has **no head-of-line blocking**
# under loss. Size-routed:
#   frame ≤ 1100 B → DATAGRAM (unreliable/unordered — pose/imu, freshest-wins, no HoL)
#   frame  > 1100 B → a unidirectional STREAM per frame (reliable — lidar/camera; datagrams are MTU-capped)
#
#   gateway.ts :8090 ──WS(frames)──► webtransport.py ──QUIC datagram|stream──► browser
#
# RUN:  WEBTRANSPORT_PORT=8093 GATEWAY_URL=ws://localhost:8090 .venv/bin/python servers/webtransport.py
import asyncio
import datetime
import hashlib
import ipaddress
import os
import struct
import tempfile

import websockets
from aioquic.asyncio import QuicConnectionProtocol, serve
from aioquic.h3.connection import H3_ALPN, H3Connection
from aioquic.h3.events import DatagramReceived, H3Event, HeadersReceived, WebTransportStreamDataReceived
from aioquic.quic.configuration import QuicConfiguration
from aioquic.quic.events import ConnectionTerminated, ProtocolNegotiated, QuicEvent
from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.x509.oid import NameOID

HOST = os.environ.get("WEBTRANSPORT_HOST", "localhost")
PORT = int(os.environ.get("WEBTRANSPORT_PORT", 8093))
CERT_PORT = int(os.environ.get("WEBTRANSPORT_CERT_PORT", 8094))
GATEWAY_URL = os.environ.get("GATEWAY_URL", "ws://localhost:8090")
DATAGRAM_MAX = 1100  # QUIC datagrams are path-MTU-capped; bigger frames go on a stream

SESSIONS: set = set()  # active WebTransportProtocol instances with an accepted session
_CERT_SHA256 = ""


def make_cert() -> tuple[str, str, str]:
    """Self-signed ECDSA P-256 cert (validity ≤10 days, SAN localhost/127.0.0.1) — short-lived so the
    browser accepts it via serverCertificateHashes. Returns (certfile, keyfile, sha256-hex of the DER)."""
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


class WebTransportProtocol(QuicConnectionProtocol):
    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self._http: H3Connection | None = None
        self._session_id: int | None = None

    def quic_event_received(self, event: QuicEvent) -> None:
        if isinstance(event, ProtocolNegotiated):
            self._http = H3Connection(self._quic, enable_webtransport=True)
        elif isinstance(event, ConnectionTerminated):
            SESSIONS.discard(self)
        if self._http is not None:
            for h3_event in self._http.handle_event(event):
                self._h3_event_received(h3_event)

    def _h3_event_received(self, event: H3Event) -> None:
        if isinstance(event, HeadersReceived):
            h = dict(event.headers)
            if h.get(b":method") == b"CONNECT" and h.get(b":protocol") == b"webtransport":
                self._session_id = event.stream_id
                self._http.send_headers(  # type: ignore[union-attr]
                    stream_id=event.stream_id,
                    headers=[(b":status", b"200"), (b"sec-webtransport-http3-draft", b"draft02")],
                )
                SESSIONS.add(self)
                self.transmit()
                print(f"[webtransport] session up ({len(SESSIONS)} active)", flush=True)
        # incoming datagrams/streams ignored — this is a server→client read path.

    def send_frame(self, frame: bytes) -> None:
        if self._http is None or self._session_id is None:
            return
        try:
            if len(frame) <= DATAGRAM_MAX:
                self._http.send_datagram(self._session_id, frame)
            else:
                sid = self._http.create_webtransport_stream(self._session_id, is_unidirectional=True)
                self._quic.send_stream_data(sid, frame, end_stream=True)
            self.transmit()
        except Exception:
            SESSIONS.discard(self)


def _restamp(frame: bytes) -> bytes:
    if len(frame) < 8:
        return frame
    import time

    return struct.pack(">d", time.time() * 1000.0) + frame[8:]


def broadcast(frame: bytes) -> None:
    f = _restamp(frame)
    for s in list(SESSIONS):
        s.send_frame(f)


async def gateway_pump() -> None:
    """Subscribe to the gateway and pump its binary frames into broadcast(); reconnect if it drops."""
    while True:
        try:
            async with websockets.connect(GATEWAY_URL, max_size=2**24) as ws:
                print(f"[wt] gateway_pump connected → {GATEWAY_URL}", flush=True)
                await ws.send('{"op":"subscribe","topic":"*"}')
                await ws.send('{"op":"list"}')
                async for msg in ws:
                    if isinstance(msg, (bytes, bytearray)):
                        broadcast(bytes(msg))
        except Exception as e:
            print(f"[wt] gateway_pump error: {e!r}", flush=True)
            await asyncio.sleep(1.0)


async def cert_hash_server() -> None:
    """Tiny plain-HTTP endpoint so the browser can fetch the cert SHA-256 before connecting."""

    async def handle(reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        try:
            await reader.read(1024)  # drain the request line/headers
            body = _CERT_SHA256.encode()
            writer.write(
                b"HTTP/1.1 200 OK\r\nAccess-Control-Allow-Origin: *\r\n"
                b"Content-Type: text/plain\r\nContent-Length: " + str(len(body)).encode() + b"\r\n\r\n" + body
            )
            await writer.drain()
        finally:
            writer.close()

    srv = await asyncio.start_server(handle, HOST, CERT_PORT)
    async with srv:
        await srv.serve_forever()


async def main() -> None:
    global _CERT_SHA256
    certfile, keyfile, _CERT_SHA256 = make_cert()
    config = QuicConfiguration(is_client=False, alpn_protocols=H3_ALPN, max_datagram_frame_size=65536)
    config.load_cert_chain(certfile, keyfile)
    asyncio.create_task(gateway_pump())
    asyncio.create_task(cert_hash_server())
    print(
        f"[webtransport] https://{HOST}:{PORT}  ←  {GATEWAY_URL}  ·  cert-sha256={_CERT_SHA256}  "
        f"(hash at http://{HOST}:{CERT_PORT}/cert-hash)",
        flush=True,
    )
    await serve(HOST, PORT, configuration=config, create_protocol=WebTransportProtocol)
    await asyncio.Future()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass

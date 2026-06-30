"""Headless aioquic WebTransport client for the dimoscope QUIC endpoint (serve.py) — a browser
stand-in that opens a session, receives datagrams (small frames) + unidirectional streams (large
frames), and counts frames over DUR seconds. Verifies the full path (serve.py bus → QUIC) without a
browser and without the cert-hash dance (verify_mode=CERT_NONE). Each frame is [f64 BE send-ms][LC02], so we
also derive per-frame latency (now − send-ms) → p50/p95, which the loss bench (bench/loss.sh) uses to
show datagram latency stays flat under loss (no TCP head-of-line blocking). Run with a gateway +
bench_source up:
  WT_PORT=8093 DUR=3 .venv/bin/python bench/webtransport_client_probe.py
"""
import asyncio
import os
import ssl
import struct
import sys
import time

from aioquic.asyncio import QuicConnectionProtocol, connect
from aioquic.h3.connection import H3_ALPN, H3Connection
from aioquic.h3.events import DatagramReceived, HeadersReceived, WebTransportStreamDataReceived
from aioquic.quic.configuration import QuicConfiguration
from aioquic.quic.events import ProtocolNegotiated

HOST = os.environ.get("WT_HOST", "localhost")
PORT = int(os.environ.get("WT_PORT", 8093))
DUR = float(os.environ.get("DUR", 3))


def _pctl(xs: list[float], q: float) -> float:
    if not xs:
        return 0.0
    s = sorted(xs)
    return s[min(len(s) - 1, int(q * len(s)))]


def _lat_ms(frame: bytes) -> float | None:
    """now − the f64 BE send-ms prefix the server re-stamps at broadcast; None if no prefix."""
    if len(frame) < 8:
        return None
    return time.time() * 1000.0 - struct.unpack(">d", frame[:8])[0]


class ClientProto(QuicConnectionProtocol):
    def __init__(self, *a, **k) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*a, **k)
        self._http: H3Connection | None = None
        self.count = 0
        self.nbytes = 0
        self.dg = 0
        self.st = 0
        self.lat_dg: list[float] = []
        self.lat_st: list[float] = []
        self._streams: dict[int, bytes] = {}

    def quic_event_received(self, event) -> None:  # type: ignore[no-untyped-def]
        if isinstance(event, ProtocolNegotiated):
            self._http = H3Connection(self._quic, enable_webtransport=True)
            sid = self._quic.get_next_available_stream_id()
            self._http.send_headers(
                stream_id=sid,
                headers=[
                    (b":method", b"CONNECT"),
                    (b":protocol", b"webtransport"),
                    (b":scheme", b"https"),
                    (b":authority", f"{HOST}:{PORT}".encode()),
                    (b":path", b"/"),
                ],
            )
            self.transmit()
        if self._http is not None:
            for ev in self._http.handle_event(event):
                self._h3(ev)

    def _h3(self, ev) -> None:  # type: ignore[no-untyped-def]
        if isinstance(ev, HeadersReceived):
            return  # 200 → session established
        if isinstance(ev, DatagramReceived):
            self.count += 1
            self.dg += 1
            self.nbytes += len(ev.data)
            lat = _lat_ms(ev.data)
            if lat is not None:
                self.lat_dg.append(lat)
        elif isinstance(ev, WebTransportStreamDataReceived):
            buf = self._streams.get(ev.stream_id, b"") + ev.data
            if ev.stream_ended:
                self.count += 1
                self.st += 1
                self.nbytes += len(buf)
                lat = _lat_ms(buf)
                if lat is not None:
                    self.lat_st.append(lat)
                self._streams.pop(ev.stream_id, None)
            else:
                self._streams[ev.stream_id] = buf


async def run() -> int:
    config = QuicConfiguration(is_client=True, alpn_protocols=H3_ALPN, max_datagram_frame_size=65536)
    config.verify_mode = ssl.CERT_NONE
    async with connect(HOST, PORT, configuration=config, create_protocol=ClientProto) as proto:
        await asyncio.sleep(DUR)
        p: ClientProto = proto  # type: ignore[assignment]
        print(
            f"WEBTRANSPORT_PROBE count={p.count} (datagrams={p.dg} streams={p.st}) "
            f"bytes={p.nbytes} hz={p.count / DUR:.0f} "
            f"dgP50={_pctl(p.lat_dg, 0.50):.0f} dgP95={_pctl(p.lat_dg, 0.95):.0f} "
            f"stP50={_pctl(p.lat_st, 0.50):.0f} stP95={_pctl(p.lat_st, 0.95):.0f}"
        )
        return 0 if p.count > 0 else 1


if __name__ == "__main__":
    sys.exit(asyncio.run(run()))

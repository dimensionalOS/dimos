#!/usr/bin/env python3
# dimoscope gateway: the whole backend in one process. Serves the built frontend + every transport on one
# HTTP port (+ one UDP port for WebTransport/QUIC); ingest taps both LCM and Zenoh (gateway/bus.py).
# Run: python -m gateway
#
#   GET  /                 the built web app (app/dist)
#   WS   /ws               data plane (topics + teleop/goal/rpc)   ← @dimos/web default
#   WS   /media            camera: webrtc / webcodecs / jpeg
#   WS   /rtc              WebRTC DataChannel (bench)
#   GET  /sse              Server-Sent Events (bench)
#   GET  /poll             HTTP long-poll (bench)
#   GET  /cert             WebTransport cert SHA-256 (browser needs it before connecting)
#   QUIC :WT_PORT/UDP      WebTransport (bench)
from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
import os
from pathlib import Path

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import PlainTextResponse
from fastapi.staticfiles import StaticFiles

from dimos.utils.logging_config import setup_logger

from . import qos
from .bus import Bus
from .data import DataPlane
from .egress import SafetyEgress
from .media import MediaPlane
from .transports import PollPlane, SsePlane, WebRtcDataPlane, WebTransportServer

logger = setup_logger()

# 0.0.0.0 so another device on the LAN (e.g. a phone) can open the app; this also exposes teleop on the
# LAN (the data plane clamps velocity + runs a deadman). Set HOST=127.0.0.1 for loopback-only.
HOST = os.environ.get("HOST", "0.0.0.0")
PORT = int(os.environ.get("PORT", 8080))
WT_PORT = int(os.environ.get("WT_PORT", 8443))  # WebTransport QUIC/UDP (separate listener)
ZENOH_KEY = os.environ.get("ZENOH_KEY", "**")
LCM_HOST = os.environ.get("DIMOS_LCM_HOST", "239.255.76.67")
LCM_PORT = int(os.environ.get("DIMOS_LCM_PORT", 7667))
# app.py lives in gateway/, so the build + QoS rules sit one level up at the dimoscope root (parent.parent).
DIST = Path(os.environ.get("STATIC_DIR", Path(__file__).parent.parent / "app" / "dist"))
# Optional operator QoS map (topic-glob → scheduler lane) for topics the name/type heuristic can't
# classify. Absent = pure heuristic. Copy qos.rules.example.json to enable.
QOS_RULES = Path(os.environ.get("QOS_RULES", Path(__file__).parent.parent / "qos.rules.json"))


def build_app() -> FastAPI:
    qos.load_qos_rules(QOS_RULES)
    bus = Bus()
    egress = SafetyEgress()  # one teleop/goal/rpc trust boundary, shared by /ws + WebTransport
    data = DataPlane(bus, egress)
    media = MediaPlane(bus)
    sse = SsePlane(bus)
    poll = PollPlane(bus)
    rtc = WebRtcDataPlane(bus)
    wt = WebTransportServer(bus, egress)

    @asynccontextmanager
    async def lifespan(_app: FastAPI):
        bus.start_zenoh(ZENOH_KEY)
        await bus.start_lcm(LCM_HOST, LCM_PORT)
        egress.start()
        tasks = [
            asyncio.create_task(media.run_encoder()),
            asyncio.create_task(media.run_fanout()),
            asyncio.create_task(wt.start(HOST, WT_PORT)),
        ]
        logger.info("dimoscope up", url=f"http://localhost:{PORT}", transports="all")
        try:
            yield
        finally:
            for t in tasks:
                t.cancel()
            bus.close()

    app = FastAPI(lifespan=lifespan)

    # CORS: the SDK may be served from a different origin than the gateway (real-WAN: app on localhost,
    # gateway on the VPS by IP), so the browser can fetch /cert (the WebTransport cert hash) cross-origin.
    # WS/WebRTC/QUIC aren't subject to CORS; `*` is acceptable for a read-only + safe-teleop bench service.
    app.add_middleware(
        CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"]
    )

    # Data + media + bench websockets. Registered before the static mount so "/" doesn't shadow them.
    app.add_api_websocket_route("/ws", data.handle)
    app.add_api_websocket_route("/media", media.handle)
    app.add_api_websocket_route("/rtc", rtc.handle)
    app.add_api_route("/sse", sse.handle, methods=["GET"])
    app.add_api_route("/poll", poll.handle, methods=["GET"])
    app.add_api_route("/cert", lambda: PlainTextResponse(wt.cert_hash), methods=["GET"])
    app.add_api_route("/health", lambda: PlainTextResponse("ok"), methods=["GET"])

    # Frontend last: catch-all static serve of the Vite build (index.html at /).
    if DIST.is_dir():
        app.mount("/", StaticFiles(directory=str(DIST), html=True), name="app")
    else:

        @app.get("/")
        def _no_build() -> PlainTextResponse:
            return PlainTextResponse(
                f"No frontend build at {DIST}. Run `deno task build` first.", status_code=503
            )

    return app

#!/usr/bin/env python3
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

# dimoscope gateway: the Python brain of the backend. Serves the built frontend + every TCP transport on
# one HTTP port; ingest taps both LCM and Zenoh (gateway/bus.py). WebTransport and WebRTC (UDP) live in
# the native sidecar (gateway/wt-sidecar), fed over a unix socket (gateway/pipe.py).
# Run: deno task serve   (this gateway + the sidecar)
#
#   GET  /                 the built web app (app/dist)
#   WS   /ws               data plane (topics + teleop/goal/rpc)   ← @dimos/web default
#   WS   /media            camera: webrtc / webcodecs / jpeg
#   WS   /rtc              WebRTC signaling (SDP relay; the sidecar owns the DataChannel sessions)
#   GET  /sse              Server-Sent Events (bench)
#   GET  /poll             HTTP long-poll (bench)
#   GET  /cert             WebTransport cert SHA-256 (the sidecar's; browser needs it before connecting)
#   UDS  WT_PIPE           feed to the sidecar, which owns WebTransport :WT_PORT + WebRTC :RTC_PORT
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

from . import netem, qos
from .bus import Bus
from .data import DataPlane
from .egress import SafetyEgress
from .media import MediaPlane
from .pipe import PipePlane
from .transports import PollPlane, RtcSignalRelay, SsePlane

logger = setup_logger()

# 0.0.0.0 so another device on the LAN (e.g. a phone) can open the app; this also exposes teleop on the
# LAN (the data plane clamps velocity + runs a deadman). Set HOST=127.0.0.1 for loopback-only.
HOST = os.environ.get("HOST", "0.0.0.0")
PORT = int(os.environ.get("PORT", 8080))
# The native Rust sidecar (gateway/wt-sidecar) owns the WebTransport QUIC/UDP listener (:WT_PORT is its
# env); the gateway feeds it over this unix socket (gateway/pipe.py) and serves its cert hash at /cert.
WT_PIPE = os.environ.get("WT_PIPE", "/tmp/dimoscope-wt.sock")
WT_CERT_HASH_FILE = Path(os.environ.get("WT_CERT_HASH_FILE", "/tmp/dimoscope-wt-cert.hash"))
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
    pipe = PipePlane(bus, egress)
    rtc = RtcSignalRelay(pipe)  # /rtc = SDP relay; the sidecar owns the WebRTC sessions

    @asynccontextmanager
    async def lifespan(_app: FastAPI):
        bus.start_zenoh(ZENOH_KEY)
        await bus.start_lcm(LCM_HOST, LCM_PORT)
        egress.start()
        tasks = [
            asyncio.create_task(media.run_encoder()),
            asyncio.create_task(media.run_fanout()),
            asyncio.create_task(pipe.start(WT_PIPE)),
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

    def cert_hash() -> PlainTextResponse:
        # The sidecar writes its cert's SHA-256 hex to WT_CERT_HASH_FILE at startup, but the file
        # outlives the process — gate on a live pipe connection so a dead sidecar's stale hash can't
        # 200 (the client would burn a doomed connect). 503 = not up yet / not connected.
        if not pipe.connected:
            return PlainTextResponse("sidecar not connected", status_code=503)
        try:
            h = WT_CERT_HASH_FILE.read_text().strip()
        except OSError:
            h = ""
        return PlainTextResponse(h if h else "sidecar not up", status_code=200 if h else 503)

    app.add_api_route("/cert", cert_hash, methods=["GET"])
    app.add_api_route("/health", lambda: PlainTextResponse("ok"), methods=["GET"])
    # Browser-controlled network conditions (bench): OFF unless NETEM_CTL=1 (gateway/netem.py).
    app.add_api_route("/netem", netem.get_state, methods=["GET"])
    app.add_api_route("/netem", netem.set_profile, methods=["POST"])

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

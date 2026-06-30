#!/usr/bin/env python3
# dimoscope — the whole thing in ONE process.
#
# Serves the built frontend (the SDK consumer) AND every transport on one HTTP port (+ one UDP port
# for WebTransport, which rides QUIC and can't share TCP). Everything is on by default — no flags:
#
#   GET  /                 the built web app (app/dist)            ← the SDK that consumes the server
#   WS   /ws               data plane (topics + teleop/goal/rpc)   ← @dimos/topics default
#   WS   /media            camera: webrtc / webcodecs / jpeg
#   WS   /rtc              WebRTC DataChannel (bench)
#   GET  /sse              Server-Sent Events (bench)
#   GET  /poll             HTTP long-poll (bench)
#   GET  /cert             WebTransport cert SHA-256 (browser needs it before connecting)
#   QUIC :WT_PORT/UDP      WebTransport (bench)
#
# Ingest taps BOTH LCM and Zenoh at once (see servers/bus.py), so topics show up no matter which
# transport the robot/sim uses. Run:  uv run dimoscope   (or  python serve.py)
from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
import os
from pathlib import Path

from fastapi import FastAPI
from fastapi.responses import PlainTextResponse
from fastapi.staticfiles import StaticFiles
from servers.bench import PollPlane, SsePlane, WebRtcDataPlane, WebTransportServer
from servers.bus import Bus
from servers.data import DataPlane
from servers.media import MediaPlane
import uvicorn

# 0.0.0.0 so you can open the app from another device on the LAN (e.g. a phone, for camera tests).
# That also exposes teleop on the LAN — the data plane clamps velocity + runs a deadman, but set
# HOST=127.0.0.1 if you want it loopback-only.
HOST = os.environ.get("HOST", "0.0.0.0")
PORT = int(os.environ.get("PORT", 8080))
WT_PORT = int(os.environ.get("WT_PORT", 8443))  # WebTransport QUIC/UDP (separate listener)
ZENOH_KEY = os.environ.get("ZENOH_KEY", "**")
LCM_HOST = os.environ.get("DIMOS_LCM_HOST", "239.255.76.67")
LCM_PORT = int(os.environ.get("DIMOS_LCM_PORT", 7667))
DIST = Path(os.environ.get("STATIC_DIR", Path(__file__).parent / "app" / "dist"))


def build_app() -> FastAPI:
    bus = Bus()
    data = DataPlane(bus)
    media = MediaPlane(bus)
    sse = SsePlane(bus)
    poll = PollPlane(bus)
    rtc = WebRtcDataPlane(bus)
    wt = WebTransportServer(bus)

    @asynccontextmanager
    async def lifespan(_app: FastAPI):
        bus.start_zenoh(ZENOH_KEY)
        await bus.start_lcm(LCM_HOST, LCM_PORT)
        data.start_egress()
        tasks = [
            asyncio.create_task(media.run_encoder()),
            asyncio.create_task(media.run_fanout()),
            asyncio.create_task(wt.start(HOST, WT_PORT)),
        ]
        print(f"[dimoscope] http://localhost:{PORT}  ·  all transports live", flush=True)
        try:
            yield
        finally:
            for t in tasks:
                t.cancel()
            bus.close()

    app = FastAPI(lifespan=lifespan)

    # Data + media + bench websockets. Registered BEFORE the static mount so "/" doesn't shadow them.
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


def cli() -> None:
    uvicorn.run(build_app(), host=HOST, port=PORT, log_level="info")


if __name__ == "__main__":
    cli()

#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

"""Local dev broker for HostedTeleopModule end-to-end testing.

Stand-in for the production Cloudflare Worker broker. Implements the same
``/api/v1/sessions`` API spec, but bridges a robot ↔ operator pair using
aiortc on the broker side instead of routing through Cloudflare's SFU.

No internet, no Cloudflare account, no external dependencies — runs locally,
designed for laptop testing.

Architecture
------------
The broker creates one ``RTCPeerConnection`` per peer (robot, operator) and
forwards DataChannel messages between them by channel label. Effectively a
tiny in-process SFU. The four channels in your spec
(``cmd_unreliable``/``cmd_reliable``/``state_unreliable``/``state_reliable``)
will all bridge automatically — robot opens, operator opens same name,
broker matches them and forwards bytes.

Usage
-----
1. Start the broker::

       python -m dimos.teleop.quest_hosted.dev_broker

2. Run dimos pointing at this broker::

       dimos run teleop-hosted-xarm7 \\
         --hosted-arm-teleop-module.broker-url=http://localhost:8000 \\
         --hosted-arm-teleop-module.robot-id=test-robot \\
         --hosted-arm-teleop-module.robot-name="Test XArm7"

3. Open the operator HTML in a browser. Easiest path: this broker also
   serves it at ``http://localhost:8000/teleop``. Set the broker URL field
   in the UI to ``http://localhost:8000``, click "List robots", connect.

Networking notes
----------------
- For Quest-from-same-wifi: replace ``localhost`` with your laptop's LAN
  IP (e.g. ``http://192.168.1.10:8000``) and make sure your firewall lets
  port 8000 through.
- WebXR (immersive-ar/vr) requires HTTPS. ``localhost`` is exempt for
  desktop Chrome but **not** for the Quest browser — for Quest testing
  you'll need an HTTPS tunnel (e.g., ``cloudflared tunnel`` or ``ngrok``)
  pointing at this broker. For desktop Chrome the WebXR Emulator extension
  is enough to validate the data plane.

Limitations
-----------
- No auth (broker_api_key is ignored).
- One operator per session (most recent join wins).
- Process-local state (no KV / no persistence).
- Don't deploy this; it's the dev fixture. Production broker = Cloudflare
  Worker (separate task).
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import logging
from pathlib import Path
from typing import Any
import uuid

from aiortc import RTCPeerConnection, RTCSessionDescription
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse
from pydantic import BaseModel
import uvicorn

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
logger = logging.getLogger("dev_broker")

STATIC_DIR = Path(__file__).parent / "static"


# ─── Session state ──────────────────────────────────────────────────────────


@dataclass
class Session:
    session_id: str
    robot_id: str
    robot_name: str
    robot_pc: RTCPeerConnection
    robot_channels: dict[str, Any] = field(default_factory=dict)
    operator_pc: RTCPeerConnection | None = None
    operator_channels: dict[str, Any] = field(default_factory=dict)


_sessions: dict[str, Session] = {}


def _wire_forwarding(label: str, src_channel: Any, dst_channels: dict[str, Any]) -> None:
    """When src_channel receives a message, forward it to dst_channels[label]."""

    @src_channel.on("message")
    def _on_msg(data: Any) -> None:
        dst = dst_channels.get(label)
        if dst is not None and dst.readyState == "open":
            dst.send(data)


# ─── HTTP API ───────────────────────────────────────────────────────────────


app = FastAPI(title="DimOS Teleop Dev Broker")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


class SDPBody(BaseModel):
    sdp: str
    type: str


class RegisterBody(SDPBody):
    robot_id: str = ""
    robot_name: str = ""


@app.post("/api/v1/sessions")
async def register_robot(body: RegisterBody) -> dict[str, str]:
    """Robot registers — broker becomes the WebRTC peer answering its offer."""
    session_id = str(uuid.uuid4())
    robot_pc = RTCPeerConnection()

    session = Session(
        session_id=session_id,
        robot_id=body.robot_id or "unknown",
        robot_name=body.robot_name or body.robot_id or "unknown",
        robot_pc=robot_pc,
    )

    @robot_pc.on("datachannel")
    def _on_robot_dc(channel: Any) -> None:
        logger.info(f"[{session.robot_name}] robot channel: '{channel.label}'")
        session.robot_channels[channel.label] = channel
        _wire_forwarding(channel.label, channel, session.operator_channels)

    @robot_pc.on("connectionstatechange")
    async def _on_state() -> None:
        logger.info(f"[{session.robot_name}] robot PC: {robot_pc.connectionState}")

    await robot_pc.setRemoteDescription(RTCSessionDescription(sdp=body.sdp, type=body.type))
    answer = await robot_pc.createAnswer()
    await robot_pc.setLocalDescription(answer)

    _sessions[session_id] = session
    logger.info(f"[{session.robot_name}] registered, session_id={session_id}")

    return {
        "session_id": session_id,
        "sdp": robot_pc.localDescription.sdp,
        "type": robot_pc.localDescription.type,
    }


@app.delete("/api/v1/sessions/{session_id}")
async def deregister(session_id: str) -> dict[str, bool]:
    session = _sessions.pop(session_id, None)
    if not session:
        raise HTTPException(404, "Session not found")
    await session.robot_pc.close()
    if session.operator_pc:
        await session.operator_pc.close()
    logger.info(f"[{session.robot_name}] deregistered")
    return {"ok": True}


@app.post("/api/v1/sessions/{session_id}/heartbeat")
async def heartbeat(session_id: str) -> dict[str, bool]:
    if session_id not in _sessions:
        raise HTTPException(404, "Session not found")
    return {"ok": True}


@app.get("/api/v1/sessions")
async def list_sessions() -> list[dict[str, Any]]:
    return [
        {
            "session_id": s.session_id,
            "robot_id": s.robot_id,
            "robot_name": s.robot_name,
            "operator_connected": s.operator_pc is not None,
        }
        for s in _sessions.values()
    ]


@app.post("/api/v1/sessions/{session_id}/join")
async def operator_join(session_id: str, body: SDPBody) -> dict[str, str]:
    """Operator joins — broker becomes the WebRTC peer answering their offer."""
    session = _sessions.get(session_id)
    if not session:
        raise HTTPException(404, "Session not found")

    if session.operator_pc is not None:
        # Replace existing operator (most recent wins).
        await session.operator_pc.close()
        session.operator_channels.clear()

    operator_pc = RTCPeerConnection()
    session.operator_pc = operator_pc

    @operator_pc.on("datachannel")
    def _on_op_dc(channel: Any) -> None:
        logger.info(f"[{session.robot_name}] operator channel: '{channel.label}'")
        session.operator_channels[channel.label] = channel
        _wire_forwarding(channel.label, channel, session.robot_channels)

    @operator_pc.on("connectionstatechange")
    async def _on_state() -> None:
        logger.info(f"[{session.robot_name}] operator PC: {operator_pc.connectionState}")

    await operator_pc.setRemoteDescription(RTCSessionDescription(sdp=body.sdp, type=body.type))
    answer = await operator_pc.createAnswer()
    await operator_pc.setLocalDescription(answer)

    logger.info(f"[{session.robot_name}] operator joined")
    return {
        "sdp": operator_pc.localDescription.sdp,
        "type": operator_pc.localDescription.type,
    }


@app.post("/api/v1/sessions/{session_id}/leave")
async def operator_leave(session_id: str) -> dict[str, bool]:
    session = _sessions.get(session_id)
    if not session:
        raise HTTPException(404, "Session not found")
    if session.operator_pc:
        await session.operator_pc.close()
        session.operator_pc = None
        session.operator_channels.clear()
    logger.info(f"[{session.robot_name}] operator left")
    return {"ok": True}


@app.get("/")
async def index() -> dict[str, Any]:
    return {
        "service": "DimOS Teleop Dev Broker",
        "active_sessions": len(_sessions),
        "endpoints": [
            "POST   /api/v1/sessions",
            "DELETE /api/v1/sessions/:id",
            "POST   /api/v1/sessions/:id/heartbeat",
            "GET    /api/v1/sessions",
            "POST   /api/v1/sessions/:id/join",
            "POST   /api/v1/sessions/:id/leave",
        ],
        "operator_html": "/teleop",
    }


# Convenience: also serve the operator HTML so you can load it from the
# same origin during local testing.
@app.get("/teleop", response_class=HTMLResponse)
async def serve_operator_html() -> HTMLResponse:
    return HTMLResponse(content=(STATIC_DIR / "index.html").read_text())


# ─── Entry point ────────────────────────────────────────────────────────────


def main() -> None:
    parser = argparse.ArgumentParser(description="Local dev broker for hosted teleop")
    parser.add_argument(
        "--host",
        default="0.0.0.0",
        help="Bind address (default 0.0.0.0 — accessible from LAN)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8000,
        help="Port (default 8000)",
    )
    args = parser.parse_args()

    logger.info(f"Starting dev broker on http://{args.host}:{args.port}")
    logger.info("  • Point dimos with --hosted-arm-teleop-module.broker-url=...")
    logger.info(f"  • Operator HTML available at http://{args.host}:{args.port}/teleop")
    logger.info("  • Production: replace this with the Cloudflare Worker broker.")

    uvicorn.run(app, host=args.host, port=args.port, log_level="info")


if __name__ == "__main__":
    main()

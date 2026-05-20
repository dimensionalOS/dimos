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

2. Run dimos pointing at this broker. Module-config overrides go through
   ``-o`` / ``--option`` (repeat for multiples). Only ``broker_url`` needs
   overriding; ``robot_id``, ``robot_name``, ``broker_api_key`` come from
   your ``.env``::

       dimos run teleop-hosted-go2 teleop-benchmark \\
         -o hostedtwistteleopmodule.broker_url=http://localhost:8000

   …or xarm7 VR path::

       dimos run teleop-hosted-xarm7 \\
         -o hostedarmteleopmodule.broker_url=http://localhost:8000

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
CMD_CHANNEL_ID = 5
STATE_CHANNEL_ID = 7


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


def _create_negotiated_channels(
    pc: RTCPeerConnection,
    own_channels: dict[str, Any],
    peer_channels: dict[str, Any],
    log_prefix: str,
) -> None:
    """Open broker-side negotiated ``cmd_unreliable`` + ``state_reliable``.

    Mirrors the production broker's handshake — robot/operator each call
    ``createDataChannel(negotiated=True, id=CMD/STATE_CHANNEL_ID, ...)`` and
    the broker opens matching channels on its end of the same PC, then
    forwards bytes by label to the opposite peer.

    Populates *own_channels* so the opposite-peer forwarder can find these by
    label; sets up message handlers that publish to *peer_channels* when the
    opposite side's matching channel is up.
    """
    cmd = pc.createDataChannel(
        "cmd_unreliable",
        negotiated=True,
        id=CMD_CHANNEL_ID,
        ordered=False,
        maxRetransmits=0,
    )
    state = pc.createDataChannel(
        "state_reliable",
        negotiated=True,
        id=STATE_CHANNEL_ID,
        ordered=True,
    )
    for ch in (cmd, state):
        own_channels[ch.label] = ch
        _wire_forwarding(ch.label, ch, peer_channels)
        logger.info(f"{log_prefix} broker-side {ch.label} ready (sctp id={ch.id})")


# ─── HTTP API ───────────────────────────────────────────────────────────────


app = FastAPI(title="DimOS Teleop Dev Broker")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


# Request schemas — match the production broker API the robot module + HTML
# already speak. ``sdp_offer`` (string) goes both ways; the type is implied
# (always ``"offer"`` from clients, broker always answers).


class RegisterBody(BaseModel):
    robot_id: str = ""
    robot_name: str = ""
    sdp_offer: str


class JoinBody(BaseModel):
    role: str = "operator"
    sdp_offer: str


class AuthBody(BaseModel):
    email: str
    password: str


# ─── Auth stubs ─────────────────────────────────────────────────────────────
# Operator HTML expects /auth/login + /auth/register (prod broker has real
# auth backed by the dimensional-teleop stack). dev_broker accepts any
# credentials and returns a fake token — request handlers below ignore the
# Authorization header entirely.


@app.post("/api/v1/auth/login")
async def login(body: AuthBody) -> dict[str, str]:
    return {"token": "dev-token", "user_id": body.email}


@app.post("/api/v1/auth/register")
async def register(body: AuthBody) -> dict[str, str]:
    return {"token": "dev-token", "user_id": body.email}


# ─── API-key management stubs ───────────────────────────────────────────────
# Dashboard renders an "API Keys" section. dev_broker returns an empty list /
# echoes back a fake key — robots in dev are identified by free-form robot_id
# at register time, no key gating.


@app.get("/api/v1/keys")
async def list_keys() -> list[dict[str, Any]]:
    return []


@app.post("/api/v1/keys")
async def create_key(body: dict[str, Any]) -> dict[str, Any]:
    return {
        "id": "dev-key",
        "key": "dev-key-secret",
        "name": body.get("name", ""),
        "robot_id": body.get("robot_id", ""),
    }


@app.delete("/api/v1/keys/{key_id}")
async def delete_key(key_id: str) -> dict[str, bool]:
    return {"ok": True}


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

    # Open broker-side negotiated channels eagerly. on("datachannel") above
    # still catches anything else (e.g. the robot's _sctp_init throwaway).
    _create_negotiated_channels(
        robot_pc,
        session.robot_channels,
        session.operator_channels,
        log_prefix=f"[{session.robot_name}] robot",
    )

    await robot_pc.setRemoteDescription(RTCSessionDescription(sdp=body.sdp_offer, type="offer"))
    answer = await robot_pc.createAnswer()
    await robot_pc.setLocalDescription(answer)

    _sessions[session_id] = session
    logger.info(f"[{session.robot_name}] registered, session_id={session_id}")

    return {
        "session_id": session_id,
        "sdp_answer": robot_pc.localDescription.sdp,
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
async def heartbeat(session_id: str) -> dict[str, Any]:
    session = _sessions.get(session_id)
    if session is None:
        raise HTTPException(404, "Session not found")
    body: dict[str, Any] = {"ok": True}
    if session.operator_pc is not None:
        # Operator joined — robot module reads these and lazy-opens its
        # negotiated cmd_unreliable + state_reliable with matching SCTP ids.
        body["cmd_channel_subscriber_id"] = CMD_CHANNEL_ID
        body["state_channel_subscriber_id"] = STATE_CHANNEL_ID
    return body


@app.post("/api/v1/sessions/{session_id}/bridge-datachannel")
async def bridge_datachannel(session_id: str) -> dict[str, int]:
    """Operator HTML calls this after PC setup to learn which SCTP ids to bind
    its negotiated ``cmd_unreliable`` + ``state_reliable`` channels to.

    Mirrors the production broker; in dev these are constants because each PC
    has its own SCTP association.
    """
    if session_id not in _sessions:
        raise HTTPException(404, "Session not found")
    return {
        "cmd_channel_id": CMD_CHANNEL_ID,
        "state_channel_id": STATE_CHANNEL_ID,
    }


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
async def operator_join(session_id: str, body: JoinBody) -> dict[str, str]:
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

    _create_negotiated_channels(
        operator_pc,
        session.operator_channels,
        session.robot_channels,
        log_prefix=f"[{session.robot_name}] operator",
    )

    await operator_pc.setRemoteDescription(RTCSessionDescription(sdp=body.sdp_offer, type="offer"))
    answer = await operator_pc.createAnswer()
    await operator_pc.setLocalDescription(answer)

    logger.info(f"[{session.robot_name}] operator joined")
    return {"sdp_answer": operator_pc.localDescription.sdp}


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
            "POST   /api/v1/sessions/:id/bridge-datachannel",
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
    logger.info("  • Point dimos with -o hostedtwistteleopmodule.broker_url=...")
    logger.info(f"  • Operator HTML available at http://{args.host}:{args.port}/teleop")
    logger.info("  • Production: replace this with the Cloudflare Worker broker.")

    uvicorn.run(app, host=args.host, port=args.port, log_level="info")


if __name__ == "__main__":
    main()

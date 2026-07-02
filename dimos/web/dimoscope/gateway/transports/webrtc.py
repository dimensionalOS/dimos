#!/usr/bin/env python3
# /rtc — SDP signaling relay for the sidecar's WebRTC DataChannel plane (gateway/wt-sidecar/src/rtc.rs).
# The gateway carries no WebRTC stack here: a browser offer goes down the pipe as rtc-offer{rsid,sdp},
# the sidecar (which owns the muxed UDP :RTC_PORT socket) answers with rtc-answer{rsid,sdp|error},
# and this relay hands it back on the websocket. Same brain/muscle split as WebTransport.
from __future__ import annotations

import asyncio
import itertools
import json
from typing import TYPE_CHECKING

from fastapi import WebSocket, WebSocketDisconnect

if TYPE_CHECKING:  # runtime import would be circular: pipe → transports._common → this module
    from ..pipe import PipePlane

ANSWER_TIMEOUT_S = 10  # host-candidates only, no STUN — gathering is quick; a miss means a dead plane


class RtcSignalRelay:
    def __init__(self, pipe: "PipePlane") -> None:
        self.pipe = pipe
        self._rsid = itertools.count(1)
        self._pending: dict[int, asyncio.Future] = {}
        pipe.on_rtc_answer = self._on_answer

    def _on_answer(self, m: dict) -> None:
        fut = self._pending.pop(m.get("rsid"), None)
        if fut is not None and not fut.done():
            fut.set_result(m)

    async def handle(self, ws: WebSocket) -> None:
        # `ws: WebSocket` MUST be annotated — without the type, FastAPI treats it as a query-param
        # dependency and rejects the /rtc handshake with HTTP 403. /ws and /media do the same.
        await ws.accept()
        try:
            while True:
                m = json.loads(await ws.receive_text())
                if m.get("op") != "offer":
                    continue
                rsid = next(self._rsid)
                fut: asyncio.Future = asyncio.get_running_loop().create_future()
                self._pending[rsid] = fut
                if not self.pipe.rtc_offer(rsid, m["sdp"]):
                    self._pending.pop(rsid, None)
                    await ws.send_text(json.dumps({"op": "error", "error": "sidecar not connected"}))
                    continue
                try:
                    ans = await asyncio.wait_for(fut, timeout=ANSWER_TIMEOUT_S)
                except asyncio.TimeoutError:
                    self._pending.pop(rsid, None)
                    await ws.send_text(json.dumps({"op": "error", "error": "sidecar answer timeout"}))
                    continue
                if ans.get("error"):
                    await ws.send_text(json.dumps({"op": "error", "error": ans["error"]}))
                else:
                    await ws.send_text(json.dumps({"op": "answer", "sdp": ans.get("sdp")}))
        except (WebSocketDisconnect, json.JSONDecodeError, RuntimeError, KeyError):
            pass

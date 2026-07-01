#!/usr/bin/env python3
# WebRTC DataChannel bench transport — unordered/lossy [f64 send-ms][LC02] frames over an SCTP data
# channel (no TCP head-of-line blocking). Optional (needs aiortc). READ-ONLY. Mounts at /rtc.
from __future__ import annotations

import asyncio
import json

from fastapi import WebSocket, WebSocketDisconnect

from ..bus import Bus, Sample
from ._common import frame

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
        f = frame(s)
        for ch in list(self.channels):
            try:
                if ch.readyState == "open":
                    ch.send(f)
            except Exception:
                self.channels.discard(ch)

    async def handle(self, ws: WebSocket) -> None:
        # `ws: WebSocket` MUST be annotated — without the type, FastAPI treats it as a query-param
        # dependency, fails to resolve it, and rejects the /rtc handshake with HTTP 403 (the reason
        # WebRTC-data never connected). /ws and /media annotate their param the same way.
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

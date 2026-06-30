#!/usr/bin/env python3
# WebRTC DataChannel delivery for the data-path benchmark.
#
# A thin re-transmitter in FRONT of the existing Deno↔LCM gateway: it subscribes to the gateway over
# WebSocket (receiving the same [f64 gateway-send-ms][LC02] frames the WS/SSE/poll paths use) and fans
# each frame out over a WebRTC DataChannel to browser peers — so the browser decodes via the identical
# `frameToSample` path, no new wire format. Zero bus-tap/frame duplication:
#
#   gateway.ts :8090  ──WS(frames)──►  webrtc_data.py  ──DataChannel──►  browser
#
# Why it matters: DataChannels carry BINARY (no base64 tax like SSE) and can be unordered + lossy
# (the browser offers `{ordered:false, maxRetransmits:0}`), so under packet loss WebRTC has no TCP
# head-of-line blocking — the failure mode that sinks WS/SSE/poll on a lossy link. This is the
# mechanism the netsim-link bad-network test is built to expose.
#
# RUN:  WEBRTC_PORT=8093 GATEWAY_URL=ws://localhost:8090 .venv/bin/python servers/webrtc_data.py
import asyncio
import json
import os
import struct
import time

import websockets
from aiortc import RTCPeerConnection, RTCSessionDescription

GATEWAY_URL = os.environ.get("GATEWAY_URL", "ws://localhost:8090")
PORT = int(os.environ.get("WEBRTC_PORT", 8093))

# Open DataChannels that want frames. A channel is added on the peer's `datachannel` event and removed
# when it closes; broadcast() fans every gateway frame to all of them.
channels: set = set()
pcs: dict = {}


def _restamp(frame: bytes) -> bytes:
    """Overwrite the f64 gateway-send-ms prefix with our send time, so a WS-hop latency measurement
    reflects the DataChannel hop (not the upstream gateway hop). End-to-end mode uses the in-message
    publish ts and is unaffected by this."""
    if len(frame) < 8:
        return frame
    return struct.pack(">d", time.time() * 1000.0) + frame[8:]


def broadcast(frame: bytes) -> None:
    f = _restamp(frame)
    for ch in list(channels):
        try:
            if ch.readyState == "open":
                ch.send(f)
        except Exception:
            channels.discard(ch)


async def gateway_pump() -> None:
    """Subscribe to the gateway and pump its binary frames into broadcast(); reconnect if it drops."""
    while True:
        try:
            async with websockets.connect(GATEWAY_URL, max_size=2**24) as ws:
                await ws.send(json.dumps({"op": "subscribe", "topic": "*"}))
                await ws.send(json.dumps({"op": "list"}))
                async for msg in ws:
                    if isinstance(msg, (bytes, bytearray)):
                        broadcast(bytes(msg))
        except Exception:
            await asyncio.sleep(1.0)


async def signaling(ws) -> None:
    """One browser peer: accept its offer (it creates the DataChannel), answer (non-trickle ICE)."""
    pc = RTCPeerConnection()
    pcs[ws] = pc

    @pc.on("datachannel")
    def on_datachannel(channel) -> None:  # type: ignore[no-untyped-def]
        channels.add(channel)

        @channel.on("close")
        def _on_close() -> None:
            channels.discard(channel)

    @pc.on("connectionstatechange")
    async def _on_state() -> None:
        if pc.connectionState in ("failed", "closed"):
            for ch in [c for c in channels if getattr(c, "_pc", None) is pc]:
                channels.discard(ch)

    try:
        async for raw in ws:
            m = json.loads(raw)
            if m.get("op") == "offer":
                await pc.setRemoteDescription(RTCSessionDescription(sdp=m["sdp"], type="offer"))
                await pc.setLocalDescription(await pc.createAnswer())
                if pc.iceGatheringState != "complete":
                    fut = asyncio.get_running_loop().create_future()

                    @pc.on("icegatheringstatechange")
                    def _on_gather() -> None:
                        if pc.iceGatheringState == "complete" and not fut.done():
                            fut.set_result(None)

                    await fut
                await ws.send(json.dumps({"op": "answer", "sdp": pc.localDescription.sdp}))
    except Exception:
        pass
    finally:
        await pc.close()
        pcs.pop(ws, None)


async def main() -> None:
    asyncio.create_task(gateway_pump())
    async with websockets.serve(signaling, "localhost", PORT, max_size=2**24):
        print(f"[webrtc-data] ws://localhost:{PORT} (signaling)  ←  {GATEWAY_URL}", flush=True)
        await asyncio.Future()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass

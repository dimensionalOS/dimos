"""Headless aiortc client for servers/webrtc_data.py — a browser stand-in that connects, opens a
DataChannel, and counts frames received over DUR seconds. Verifies the full server path
(gateway → webrtc_data → DataChannel) e2e without a browser. Run with a gateway + bench_source up:
  WEBRTC_URL=ws://localhost:8093 DUR=2 .venv/bin/python bench/webrtc_client_probe.py
"""
import asyncio
import json
import os
import sys

import websockets
from aiortc import RTCPeerConnection, RTCSessionDescription


async def _gather(pc: RTCPeerConnection) -> None:
    if pc.iceGatheringState == "complete":
        return
    fut = asyncio.get_event_loop().create_future()

    @pc.on("icegatheringstatechange")
    def _on() -> None:
        if pc.iceGatheringState == "complete" and not fut.done():
            fut.set_result(None)

    await fut


async def run() -> int:
    url = os.environ.get("WEBRTC_URL", "ws://localhost:8093")
    dur = float(os.environ.get("DUR", 2))
    pc = RTCPeerConnection()
    count = 0
    nbytes = 0
    dc = pc.createDataChannel("bench", ordered=False, maxRetransmits=0)

    @dc.on("message")
    def _msg(data: bytes) -> None:
        nonlocal count, nbytes
        count += 1
        nbytes += len(data)

    async with websockets.connect(url, max_size=2**24) as ws:
        await pc.setLocalDescription(await pc.createOffer())
        await _gather(pc)
        await ws.send(json.dumps({"op": "offer", "sdp": pc.localDescription.sdp}))
        ans = json.loads(await ws.recv())
        await pc.setRemoteDescription(RTCSessionDescription(sdp=ans["sdp"], type="answer"))
        await asyncio.sleep(dur)

    print(f"WEBRTC_PROBE count={count} bytes={nbytes} hz={count / dur:.0f}")
    await pc.close()
    return 0 if count > 0 else 1


if __name__ == "__main__":
    sys.exit(asyncio.run(run()))

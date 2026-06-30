"""Loopback test for servers/webrtc_data.py — two aiortc peers establish a DataChannel and
webrtc_data.broadcast() reaches it. Verifies the DataChannel forwarding + restamp with real aiortc,
without needing a gateway/bus. Run:
  PYTHONPATH=servers /path/to/.venv/bin/python bench/webrtc_loopback_test.py
"""
import asyncio
import struct
import sys

import webrtc_data as wd
from aiortc import RTCPeerConnection


async def _gather(pc: RTCPeerConnection) -> None:
    """Non-trickle: wait for ICE gathering so localDescription carries host candidates (loopback)."""
    if pc.iceGatheringState == "complete":
        return
    fut = asyncio.get_event_loop().create_future()

    @pc.on("icegatheringstatechange")
    def _on() -> None:
        if pc.iceGatheringState == "complete" and not fut.done():
            fut.set_result(None)

    await fut


async def run() -> int:
    server = RTCPeerConnection()
    client = RTCPeerConnection()

    @server.on("datachannel")
    def on_dc(channel) -> None:  # the production registration path
        wd.channels.add(channel)

    opened: asyncio.Future = asyncio.get_event_loop().create_future()
    got: asyncio.Future = asyncio.get_event_loop().create_future()
    dc = client.createDataChannel("bench", ordered=False, maxRetransmits=0)

    @dc.on("open")
    def _open() -> None:
        if not opened.done():
            opened.set_result(None)

    @dc.on("message")
    def _msg(data: bytes) -> None:
        if not got.done():
            got.set_result(data)

    await client.setLocalDescription(await client.createOffer())
    await _gather(client)
    await server.setRemoteDescription(client.localDescription)
    await server.setLocalDescription(await server.createAnswer())
    await _gather(server)
    await client.setRemoteDescription(server.localDescription)

    await asyncio.wait_for(opened, 10)
    frame = struct.pack(">d", 111.0) + b"LC02-body-bytes"  # [f64 prefix][body]
    wd.broadcast(frame)
    data = bytes(await asyncio.wait_for(got, 5))

    ok = data[8:] == frame[8:] and data[:8] != frame[:8] and len(wd.channels) >= 1
    print("WEBRTC_LOOPBACK_OK" if ok else "WEBRTC_LOOPBACK_FAIL", f"({len(data)}B, body-preserved={data[8:]==frame[8:]}, restamped={data[:8]!=frame[:8]})")
    await client.close()
    await server.close()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(asyncio.run(run()))

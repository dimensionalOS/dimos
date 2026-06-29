#!/usr/bin/env python3
# dimos-web-gateway (Python ↔ Zenoh ↔ WebSocket) — the Zenoh alternate to the
# Bun↔LCM gateway. The ONLY practical way to put Zenoh behind the browser SDK
# (Bun/Node have no native Zenoh client). Speaks the *same* WS protocol, so the
# browser @dimos/topics SDK is byte-identical across gateways:
#   • subscribes a Zenoh key (default bench/**) on a peer session
#   • re-wraps each sample as an LC02 packet ("<topic>#<type>" + bare lcm_encode
#     payload) so the browser's decodeChannel/decode work unchanged
#   • JSON control (subscribe/unsubscribe/list) + per-client on-demand filtering
#
# RUN:  GATEWAY_PORT=8091 ZENOH_KEY='bench/**' .venv/bin/python servers/gateway_zenoh.py
import asyncio
import json
import os
import struct

import websockets
import zenoh

PORT = int(os.environ.get("GATEWAY_PORT", 8091))
KEY = os.environ.get("ZENOH_KEY", "bench/**")
LC02 = 0x4C433032

topics: dict[str, str] = {}
clients: dict[object, set[str]] = {}
_seq = 0


def make_lc02(channel: str, payload: bytes) -> bytes:
    global _seq
    _seq = (_seq + 1) & 0xFFFFFFFF
    cb = channel.encode()
    return struct.pack(">II", LC02, _seq) + cb + b"\x00" + payload


def topic_list() -> list[dict]:
    return [{"topic": t, "type": ty} for t, ty in topics.items()]


async def main() -> None:
    loop = asyncio.get_running_loop()
    out_q: asyncio.Queue = asyncio.Queue()

    def on_sample(sample) -> None:
        key = str(sample.key_expr)
        try:
            payload = sample.payload.to_bytes()
        except Exception:
            payload = bytes(sample.payload)
        base, _, typ = key.rpartition("/")  # bench/p0/geometry_msgs.PoseStamped
        topic = "/" + base
        if topic not in topics:
            topics[topic] = typ
        pkt = make_lc02(f"{topic}#{typ}", payload)
        loop.call_soon_threadsafe(out_q.put_nowait, (topic, pkt))

    session = zenoh.open(zenoh.Config())
    session.declare_subscriber(KEY, on_sample)
    print(f"[zgateway] zenoh sub '{KEY}'  ·  ws://localhost:{PORT}", flush=True)

    async def fanout() -> None:
        while True:
            topic, pkt = await out_q.get()
            for ws in list(clients.keys()):
                subs = clients.get(ws)
                if subs is not None and ("*" in subs or topic in subs):
                    try:
                        await ws.send(pkt)
                    except Exception:
                        clients.pop(ws, None)

    async def handler(ws) -> None:
        clients[ws] = set()
        await ws.send(json.dumps({"op": "hello", "topics": topic_list(), "label": "Python↔Zenoh"}))
        try:
            async for raw in ws:
                if isinstance(raw, bytes):
                    continue
                m = json.loads(raw)
                op = m.get("op")
                if op == "subscribe":
                    clients[ws].add(m["topic"])
                elif op == "unsubscribe":
                    clients[ws].discard(m["topic"])
                elif op == "list":
                    await ws.send(json.dumps({"op": "topics", "topics": topic_list()}))
        except Exception:
            pass
        finally:
            clients.pop(ws, None)

    async with websockets.serve(handler, "localhost", PORT, max_size=2**24):
        await fanout()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass

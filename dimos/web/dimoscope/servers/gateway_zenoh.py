#!/usr/bin/env python3
# dimos-web-gateway (Python ↔ Zenoh ↔ WebSocket) — the Zenoh alternate to the
# Bun↔LCM gateway. The ONLY practical way to put Zenoh behind the browser SDK
# (Bun/Node have no native Zenoh client). Speaks the *same* WS protocol, so the
# browser @dimos/topics SDK is byte-identical across gateways:
#   • subscribes a Zenoh key (default **) on a peer session
#   • re-wraps each sample as an LC02 packet ("<topic>#<type>" + bare lcm_encode
#     payload) so the browser's decodeChannel/decode work unchanged
#   • JSON control: subscribe/unsubscribe/list + teleop/stop/goal (safe publish)
#
# RUN:  GATEWAY_PORT=8090 ZENOH_KEY='**' .venv/bin/python servers/gateway_zenoh.py
import asyncio
import json
import os
import struct
import time

os.environ["DIMOS_TRANSPORT"] = "zenoh"  # this gateway publishes teleop/goal over Zenoh

import websockets
import zenoh

from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3

PORT = int(os.environ.get("GATEWAY_PORT", 8091))
KEY = os.environ.get("ZENOH_KEY", "bench/**")
LC02 = 0x4C433032
MAX_LIN = float(os.environ.get("TELEOP_MAX_LIN", 1.0))  # m/s clamp
MAX_ANG = float(os.environ.get("TELEOP_MAX_ANG", 1.5))  # rad/s clamp
DEFAULT_TTL = float(os.environ.get("TELEOP_TTL_MS", 400))  # deadman

topics: dict[str, str] = {}
clients: dict[object, set[str]] = {}
deadmen: dict[object, asyncio.TimerHandle] = {}
_seq = 0


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


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

    # Teleop + nav-goal publishers — reuse dimos transports so the zenoh key +
    # encoding match what the robot/planner subscribe to (verified).
    cmd = make_transport("/cmd_vel", Twist)
    cmd.start()  # → dimos/cmd_vel/geometry_msgs.Twist (GO2Connection.cmd_vel → .move())
    goal = make_transport("/clicked_point", PointStamped)
    goal.start()  # → dimos/clicked_point/geometry_msgs.PointStamped (nav goal)

    def publish_twist(lin: float, ang: float) -> None:
        cmd.publish(
            Twist(
                linear=Vector3(x=clamp(lin, -MAX_LIN, MAX_LIN), y=0.0, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=clamp(ang, -MAX_ANG, MAX_ANG)),
            )
        )

    def publish_goal(x: float, y: float, z: float) -> None:
        goal.publish(PointStamped(x=float(x), y=float(y), z=float(z), ts=time.time(), frame_id="world"))

    def arm_deadman(ws: object, ttl_ms: float) -> None:
        h = deadmen.pop(ws, None)
        if h:
            h.cancel()
        deadmen[ws] = loop.call_later(max(0.05, ttl_ms / 1000.0), lambda: publish_twist(0.0, 0.0))

    def cancel_deadman(ws: object) -> None:
        h = deadmen.pop(ws, None)
        if h:
            h.cancel()

    def on_sample(sample) -> None:
        key = str(sample.key_expr)
        try:
            payload = sample.payload.to_bytes()
        except Exception:
            payload = bytes(sample.payload)
        base, _, typ = key.rpartition("/")  # dimos/lidar/sensor_msgs.PointCloud2
        topic = "/" + base
        if topic not in topics:
            topics[topic] = typ
        pkt = make_lc02(f"{topic}#{typ}", payload)
        loop.call_soon_threadsafe(out_q.put_nowait, (topic, pkt))

    session = zenoh.open(zenoh.Config())
    session.declare_subscriber(KEY, on_sample)
    print(f"[zgateway] zenoh sub '{KEY}'  ·  ws://localhost:{PORT}  ·  teleop→/cmd_vel goal→/clicked_point", flush=True)

    async def fanout() -> None:
        while True:
            topic, pkt = await out_q.get()
            frame = struct.pack(">d", time.time() * 1000.0) + pkt  # [f64 gateway-send-ms][LC02]
            for ws in list(clients.keys()):
                subs = clients.get(ws)
                if subs is not None and ("*" in subs or topic in subs):
                    try:
                        await ws.send(frame)
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
                elif op == "teleop":
                    publish_twist(float(m.get("linearX", 0)), float(m.get("angularZ", 0)))
                    arm_deadman(ws, float(m.get("ttlMs", DEFAULT_TTL)))
                elif op == "stop":
                    cancel_deadman(ws)
                    publish_twist(0.0, 0.0)
                elif op == "goal":
                    publish_goal(m.get("x", 0), m.get("y", 0), m.get("z", 0))
        except Exception:
            pass
        finally:
            cancel_deadman(ws)
            publish_twist(0.0, 0.0)  # safety: stop the robot when an operator disconnects
            clients.pop(ws, None)

    async with websockets.serve(handler, "localhost", PORT, max_size=2**24):
        await fanout()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass

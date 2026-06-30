#!/usr/bin/env python3
# dimos-web-gateway (Python ↔ Zenoh ↔ WebSocket) — the Zenoh alternate to the
# Bun↔LCM gateway. The ONLY practical way to put Zenoh behind the browser SDK
# (Bun/Node have no native Zenoh client). Speaks the *same* WS protocol, so the
# browser @dimos/topics SDK is byte-identical across gateways:
#   • subscribes a Zenoh key (default **) on a peer session
#   • re-wraps each sample as an LC02 packet ("<topic>#<type>" + bare lcm_encode
#     payload) so the browser's decodeChannel/decode work unchanged
#   • JSON control: subscribe/unsubscribe/list + teleop/stop/goal + rpc (whitelisted)
#
# The CAMERA is a SEPARATE concern — encoded by the standalone media node
# (servers/media_server.py, :8092). This gateway is a thin data-relay + the
# teleop/goal/RPC trust boundary; it decodes nothing and imports no media libs.
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

from dimos.core.transport_factory import make_transport, rpc_backend
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3

PORT = int(os.environ.get("GATEWAY_PORT", 8091))
KEY = os.environ.get("ZENOH_KEY", "bench/**")
LC02 = 0x4C433032
MAX_LIN = float(os.environ.get("TELEOP_MAX_LIN", 1.0))  # m/s clamp
MAX_ANG = float(os.environ.get("TELEOP_MAX_ANG", 1.5))  # rad/s clamp
DEFAULT_TTL = float(os.environ.get("TELEOP_TTL_MS", 400))  # deadman


# RPC bridge: which dimos @rpc commands the browser may invoke. Server-side AUTHORITATIVE
# whitelist (the browser can never call a method not listed). Override with e.g.
# DIMOS_GATEWAY_RPC="GO2Connection/standup,GO2Connection/liedown".
def _parse_rpc_commands() -> list[dict]:
    env = os.environ.get("DIMOS_GATEWAY_RPC")
    if not env:
        return [
            {"target": "GO2Connection", "method": "standup", "label": "Stand up"},
            {"target": "GO2Connection", "method": "liedown", "label": "Lie down"},
            {"target": "BenchLoad", "method": "start_bench", "label": "Start bench"},
            {"target": "BenchLoad", "method": "stop_bench", "label": "Stop bench"},
        ]
    out = []
    for pair in env.split(","):
        pair = pair.strip()
        if "/" in pair:
            t, mth = pair.rsplit("/", 1)
            out.append({"target": t, "method": mth, "label": mth})
    return out


RPC_COMMANDS = _parse_rpc_commands()
RPC_WHITELIST = {(c["target"], c["method"]) for c in RPC_COMMANDS}

topics: dict[str, str] = {}
clients: dict[object, set[str]] = {}
deadmen: dict[object, asyncio.TimerHandle] = {}
_rpc = None  # dimos RPC client (rpc_backend()()), started in main() if available
HAS_RPC = False
_seq = 0


def _jsonable(v: object) -> object:
    """RPC results are arbitrary Python; keep JSON-safe primitives, stringify the rest."""
    return v if isinstance(v, (bool, int, float, str)) or v is None else str(v)


async def handle_rpc(ws: object, m: dict) -> None:
    """Bridge a browser {op:rpc} to a dimos @rpc method — whitelisted, off-loop (call_sync blocks)."""
    rid = m.get("id")
    target, method = m.get("target"), m.get("method")
    args = m.get("args") or []
    if (target, method) not in RPC_WHITELIST or _rpc is None:
        reason = "rpc unavailable" if _rpc is None else f"not allowed: {target}/{method}"
        await ws.send(json.dumps({"op": "rpc-res", "id": rid, "error": reason}))
        return
    loop = asyncio.get_running_loop()
    try:
        res = await loop.run_in_executor(
            None, lambda: _rpc.call_sync(f"{target}/{method}", (list(args), {}))[0]
        )
        await ws.send(json.dumps({"op": "rpc-res", "id": rid, "res": _jsonable(res)}))
    except Exception as e:
        await ws.send(json.dumps({"op": "rpc-res", "id": rid, "error": str(e)}))


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

    # RPC bridge client — same backend as the robot (zenoh), so dimos/rpc/<Module>/<method> match.
    global _rpc, HAS_RPC
    try:
        _rpc = rpc_backend()()
        _rpc.start()
        HAS_RPC = True
    except Exception as e:
        print(f"[zgateway] rpc bridge unavailable: {e}", flush=True)

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
        if topic.startswith("/dimos/"):  # canonical name: LCM has no prefix, so neither should the browser
            topic = topic[len("/dimos") :]  # /dimos/lidar -> /lidar
        if topic not in topics:
            topics[topic] = typ
        pkt = make_lc02(f"{topic}#{typ}", payload)
        loop.call_soon_threadsafe(out_q.put_nowait, (topic, pkt))

    session = zenoh.open(zenoh.Config())
    session.declare_subscriber(KEY, on_sample)
    print(f"[zgateway] zenoh sub '{KEY}'  ·  ws://localhost:{PORT}  ·  teleop→/cmd_vel goal→/clicked_point", flush=True)
    print(f"[zgateway] rpc bridge: {('on, ' + str(len(RPC_COMMANDS)) + ' cmds') if HAS_RPC else 'off'}", flush=True)

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
        await ws.send(
            json.dumps(
                {
                    "op": "hello",
                    "topics": topic_list(),
                    "label": "Python↔Zenoh",
                    "rpc": RPC_COMMANDS if HAS_RPC else [],
                }
            )
        )
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
                elif op == "rpc":
                    await handle_rpc(ws, m)
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

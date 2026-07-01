#!/usr/bin/env python3
# dimoscope data plane — the WebSocket the browser SDK (@dimos/topics) talks to.
#
# Speaks the gateway control protocol verbatim (the same one gateway_zenoh.py / the Deno gateway
# used), so the SDK is byte-identical:
#   • hello{topics,label,rpc}              on connect
#   • subscribe / unsubscribe / list / rate   on-demand topic filtering + per-topic downsample
#   • teleop / stop / goal                 structured + SAFE (velocity clamp + TTL deadman + stop-on-disconnect)
#   • rpc                                  whitelisted dimos @rpc bridge
# Reads samples from the shared Bus (LCM+Zenoh merged) — it taps no bus of its own. Each client gets
# its own queue + writer task, so one slow browser can't head-of-line-block the others.
#
# EGRESS goes to BOTH backends: teleop Twist / nav goal PointStamped are published over Zenoh *and*
# LCM (each via dimos make_transport with an explicit per-backend config), so whichever transport the
# robot listens on receives them; the other publish is harmless. RPC uses the service's default backend
# (DIMOS_TRANSPORT) — the robot answering RPC runs one transport, matched by how you launch the service.
from __future__ import annotations

import asyncio
import json
import os
import struct
import time

from fastapi import WebSocket, WebSocketDisconnect

from .bus import Bus, Sample
from .qos_sched import PriorityOutbox, declared_to_class, default_priority

# EGRESS_KBPS>0 → pace each client's writer to this bitrate (egress shaping to the client's link budget).
# This keeps the backlog in the priority outbox — where the scheduler enforces priority — instead of
# letting it bloat the kernel/proxy buffers downstream, which is the only way gateway-egress priority
# actually bites on a constrained link (otherwise the queue lives in the socket buffer, drained FIFO).
EGRESS_KBPS = float(os.environ.get("EGRESS_KBPS", "0"))

MAX_LIN = 1.0  # m/s clamp
MAX_ANG = 1.5  # rad/s clamp
DEFAULT_TTL = 400.0  # deadman timeout (ms)

# RPC bridge: which dimos @rpc commands the browser may invoke (server-side AUTHORITATIVE whitelist).
RPC_COMMANDS = [
    {"target": "GO2Connection", "method": "standup", "label": "Stand up"},
    {"target": "GO2Connection", "method": "liedown", "label": "Lie down"},
    {"target": "BenchLoad", "method": "start_bench", "label": "Start bench"},
    {"target": "BenchLoad", "method": "stop_bench", "label": "Stop bench"},
]
RPC_WHITELIST = {(c["target"], c["method"]) for c in RPC_COMMANDS}


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def _jsonable(v: object) -> object:
    return v if isinstance(v, (bool, int, float, str)) or v is None else str(v)


class _Client:
    __slots__ = ("deadman", "last", "q", "qos", "rate", "subs")

    def __init__(self) -> None:
        self.subs: set[str] = set()  # subscribed topics; "*" = all
        self.rate: dict[str, float] = {}  # topic -> maxHz (0/absent = unlimited)
        self.last: dict[str, float] = {}  # topic -> last forward time (ms)
        self.qos: dict[
            str, tuple
        ] = {}  # topic -> (rank, conflate, depth) client override (else default)
        self.q = PriorityOutbox()  # per-client priority + conflation outbox
        self.deadman: asyncio.TimerHandle | None = None


class DataPlane:
    """Holds the egress publishers + per-client state; one instance, created by serve.py."""

    def __init__(self, bus: Bus) -> None:
        self.bus = bus
        self.clients: dict[WebSocket, _Client] = {}
        self._cmd: list = []  # teleop Twist publishers (one per backend)
        self._goal: list = []  # nav goal PointStamped publishers (one per backend)
        self._rpc = None
        self.has_rpc = False
        bus.subscribe(self._on_sample)
        bus.on_new_topic(self._on_new_topic)

    # ── egress: publish teleop + goal to BOTH backends ──────────────────────
    def start_egress(self) -> None:
        from dimos.core.global_config import global_config
        from dimos.core.transport_factory import make_transport, rpc_backend
        from dimos.msgs.geometry_msgs.PointStamped import PointStamped
        from dimos.msgs.geometry_msgs.Twist import Twist
        from dimos.msgs.geometry_msgs.Vector3 import Vector3

        self._Twist, self._Point, self._Vector3 = Twist, PointStamped, Vector3
        for backend in ("zenoh", "lcm"):
            try:
                g = global_config.model_copy(update={"transport": backend})
                cmd = make_transport("/cmd_vel", Twist, g=g)
                cmd.start()
                goal = make_transport("/clicked_point", PointStamped, g=g)
                goal.start()
                self._cmd.append(cmd)
                self._goal.append(goal)
            except Exception as e:
                print(f"[data] egress backend '{backend}' unavailable: {e}", flush=True)
        # RPC bridge on the service's default backend (matches the robot's transport).
        try:
            self._rpc = rpc_backend()()
            self._rpc.start()
            self.has_rpc = True
        except Exception as e:
            print(f"[data] rpc bridge unavailable: {e}", flush=True)
        print(
            f"[data] egress backends={len(self._cmd)} · rpc={'on' if self.has_rpc else 'off'}",
            flush=True,
        )

    def _publish_twist(self, lin: float, ang: float) -> None:
        if not self._cmd:
            return  # no egress backend (e.g. dimos transport unavailable)
        Vector3 = self._Vector3
        msg = self._Twist(
            linear=Vector3(x=_clamp(lin, -MAX_LIN, MAX_LIN), y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=_clamp(ang, -MAX_ANG, MAX_ANG)),
        )
        for t in self._cmd:
            try:
                t.publish(msg)
            except Exception:
                pass

    def _publish_goal(self, x: float, y: float, z: float) -> None:
        if not self._goal:
            return
        msg = self._Point(x=float(x), y=float(y), z=float(z), ts=time.time(), frame_id="world")
        for t in self._goal:
            try:
                t.publish(msg)
            except Exception:
                pass

    # ── bus fan-out (loop thread, must stay cheap) ──────────────────────────
    def _on_sample(self, s: Sample) -> None:
        if not self.clients:
            return
        now = time.time() * 1000.0
        default = default_priority(
            s.topic, s.type
        )  # server default; clients may override per subscription
        frame: bytes | None = None
        for st in self.clients.values():
            if not ("*" in st.subs or s.topic in st.subs):
                continue
            hz = st.rate.get(s.topic, 0)
            if hz > 0:
                if now - st.last.get(s.topic, 0) < 1000.0 / hz:
                    continue  # downsample
                st.last[s.topic] = now
            if frame is None:
                frame = struct.pack(">d", now) + s.lc02  # [f64 gateway-send-ms][LC02]
            prio, conflate, depth = st.qos.get(
                s.topic, default
            )  # client override else server default
            st.q.put_data(
                s.topic, prio, conflate, depth, frame
            )  # priority outbox; never blocks/raises

    def _on_new_topic(self, topic: str, typ: str) -> None:
        msg = json.dumps({"op": "topic", "topic": topic, "type": typ})
        for st in self.clients.values():
            st.q.put_control(msg)  # control → top priority, generously buffered

    # ── per-client connection ───────────────────────────────────────────────
    async def handle(self, ws: WebSocket) -> None:
        await ws.accept()
        st = _Client()
        self.clients[ws] = st
        loop = asyncio.get_running_loop()
        await ws.send_text(
            json.dumps(
                {
                    "op": "hello",
                    "topics": self.bus.topic_list(),
                    "label": "dimoscope",
                    "rpc": RPC_COMMANDS if self.has_rpc else [],
                }
            )
        )
        writer = asyncio.create_task(self._writer(ws, st))
        try:
            while True:
                m = json.loads(await ws.receive_text())
                await self._on_control(ws, st, m, loop)
        except (WebSocketDisconnect, json.JSONDecodeError, RuntimeError):
            pass
        finally:
            if st.deadman:
                st.deadman.cancel()
            self._publish_twist(0.0, 0.0)  # safety: stop the robot on operator disconnect
            self.clients.pop(ws, None)
            writer.cancel()

    async def _writer(self, ws: WebSocket, st: _Client) -> None:
        try:
            while True:
                item = await st.q.get()
                if isinstance(item, (bytes, bytearray)):
                    await ws.send_bytes(item)
                else:
                    await ws.send_text(item)
                if (
                    EGRESS_KBPS > 0
                ):  # pace egress → the backlog (and thus priority) stays in the outbox
                    await asyncio.sleep(len(item) * 8 / (EGRESS_KBPS * 1000.0))
        except (WebSocketDisconnect, RuntimeError, asyncio.CancelledError):
            pass

    async def _on_control(self, ws, st: _Client, m: dict, loop) -> None:
        op = m.get("op")
        if op == "subscribe":
            st.subs.add(m["topic"])
            if m.get("maxHz"):
                st.rate[m["topic"]] = float(m["maxHz"])
            self._apply_qos(
                st, m["topic"], m
            )  # client-declared priority/reliability/depth (overrides default)
        elif op == "unsubscribe":
            st.subs.discard(m["topic"])
            st.rate.pop(m["topic"], None)
            st.last.pop(m["topic"], None)
            st.qos.pop(m["topic"], None)
        elif op == "rate":
            st.rate[m["topic"]] = float(m.get("maxHz") or 0)
        elif op == "list":
            st.q.put_control(json.dumps({"op": "topics", "topics": self.bus.topic_list()}))
        elif op == "teleop":
            self._publish_twist(float(m.get("linearX", 0)), float(m.get("angularZ", 0)))
            self._arm_deadman(st, float(m.get("ttlMs", DEFAULT_TTL)), loop)
        elif op == "stop":
            self._cancel_deadman(st)
            self._publish_twist(0.0, 0.0)
        elif op == "goal":
            self._publish_goal(m.get("x", 0), m.get("y", 0), m.get("z", 0))
        elif op == "rpc":
            await self._handle_rpc(st, m, loop)

    def _apply_qos(self, st: _Client, topic: str, m: dict) -> None:
        """Store the client's declared QoS override for `topic` (priority/reliability/depth merged onto
        the server default), or clear it back to the default if the client declared none."""
        if any(m.get(k) is not None for k in ("priority", "reliability", "depth")):
            default = default_priority(topic, self.bus.topics.get(topic, ""))
            st.qos[topic] = declared_to_class(
                m.get("priority"), m.get("reliability"), m.get("depth"), default
            )
        else:
            st.qos.pop(topic, None)

    def _arm_deadman(self, st: _Client, ttl_ms: float, loop) -> None:
        if st.deadman:
            st.deadman.cancel()
        st.deadman = loop.call_later(
            max(0.05, ttl_ms / 1000.0), lambda: self._publish_twist(0.0, 0.0)
        )

    def _cancel_deadman(self, st: _Client) -> None:
        if st.deadman:
            st.deadman.cancel()
            st.deadman = None

    async def _handle_rpc(self, st: _Client, m: dict, loop) -> None:
        rid, target, method = m.get("id"), m.get("target"), m.get("method")
        args = m.get("args") or []
        if (target, method) not in RPC_WHITELIST or self._rpc is None:
            reason = "rpc unavailable" if self._rpc is None else f"not allowed: {target}/{method}"
            st.q.put_control(json.dumps({"op": "rpc-res", "id": rid, "error": reason}))
            return
        try:
            res = await loop.run_in_executor(
                None, lambda: self._rpc.call_sync(f"{target}/{method}", (list(args), {}))[0]
            )
            st.q.put_control(json.dumps({"op": "rpc-res", "id": rid, "res": _jsonable(res)}))
        except Exception as e:
            st.q.put_control(json.dumps({"op": "rpc-res", "id": rid, "error": str(e)}))

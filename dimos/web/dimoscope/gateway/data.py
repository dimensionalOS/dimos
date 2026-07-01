#!/usr/bin/env python3
# dimoscope data plane: the WebSocket the browser SDK (@dimos/web) talks to. Speaks the gateway control
# protocol (hello/subscribe/unsubscribe/list/rate/teleop/stop/goal/rpc). teleop/goal are safe (velocity
# clamp + TTL deadman + stop-on-disconnect); rpc is a whitelisted bridge. Reads from the shared Bus
# (LCM+Zenoh merged); each client gets its own queue + writer task so one slow browser can't
# head-of-line-block the others.
from __future__ import annotations

import asyncio
import json
import os
import struct
import time

from fastapi import WebSocket, WebSocketDisconnect

from dimos.utils.logging_config import setup_logger

from .bus import Bus, Sample
from .egress import DEFAULT_TTL, SafetyEgress
from .qos import PriorityOutbox, declared_to_class, default_priority

logger = setup_logger()

# EGRESS_KBPS>0 → pace each client's writer to this bitrate. This keeps the backlog in the priority outbox
# (where the scheduler enforces priority) instead of bloating downstream kernel/proxy buffers — the only
# way gateway-egress priority actually bites on a constrained link (else the queue lives in the socket
# buffer, drained FIFO).
EGRESS_KBPS = float(os.environ.get("EGRESS_KBPS", "0"))

# Velocity clamp + TTL deadman + stop-on-disconnect + the @rpc whitelist live in the shared SafetyEgress
# (gateway/egress.py) so /ws and WebTransport share one trust boundary.


class _Client:
    __slots__ = ("last", "q", "qos", "rate", "subs")

    def __init__(self) -> None:
        self.subs: set[str] = set()  # subscribed topics; "*" = all
        self.rate: dict[str, float] = {}  # topic -> maxHz (0/absent = unlimited)
        self.last: dict[str, float] = {}  # topic -> last forward time (ms)
        self.qos: dict[
            str, tuple
        ] = {}  # topic -> (rank, conflate, depth) client override (else default)
        self.q = PriorityOutbox()  # per-client priority + conflation outbox


class DataPlane:
    """The /ws data plane: per-client on-demand subs + priority outbox. Write-path control (teleop/goal/
    rpc) is delegated to the shared SafetyEgress so /ws and WebTransport share one trust boundary."""

    def __init__(self, bus: Bus, egress: SafetyEgress) -> None:
        self.bus = bus
        self.egress = egress
        self.clients: dict[WebSocket, _Client] = {}
        bus.subscribe(self._on_sample)
        bus.on_new_topic(self._on_new_topic)

    # bus fan-out (loop thread, keep cheap)
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
                    "rpc": self.egress.commands,
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
            self.egress.disconnect(ws)  # safety: deadman-cancel + stop the robot on disconnect
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
        elif op == "ping":  # clock-sync probe: echo our clock so the client can estimate skew
            st.q.put_control(
                json.dumps({"op": "pong", "id": m.get("id"), "serverTs": time.time() * 1000.0})
            )
        elif op == "teleop":
            self.egress.teleop(
                ws,
                float(m.get("linearX", 0)),
                float(m.get("angularZ", 0)),
                float(m.get("ttlMs", DEFAULT_TTL)),
                loop,
            )
        elif op == "stop":
            self.egress.stop(ws)
        elif op == "goal":
            self.egress.goal(m.get("x", 0), m.get("y", 0), m.get("z", 0))
        elif op == "rpc":
            res = await self.egress.rpc(m.get("target"), m.get("method"), m.get("args") or [], loop)
            st.q.put_control(json.dumps({"op": "rpc-res", "id": m.get("id"), **res}))

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

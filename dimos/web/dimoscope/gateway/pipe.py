#!/usr/bin/env python3
# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Pipe plane: feeds the native sidecar (gateway/wt-sidecar — WebTransport :8443 + WebRTC :8444)
# over a local unix socket. The gateway keeps the single bus tap, SafetyEgress and QoS config; the
# sidecar owns the wire sessions and per-session scheduling. The gateway LISTENS; the sidecar
# connects (and reconnects).
#
# Framing: [u32be len][u8 kind][payload]
#   kind 1 = DATA (gateway→sidecar): raw Sample.lc02 — topic/type live inside the LC02 channel; the
#            sidecar stamps the [f64be send-ms] prefix at its ingress (mirror of _common.frame timing).
#   kind 2 = JSON (both directions):
#     downstream  hello{topics,label,rpc} · topic{topic,type} · rpc-res{sid,id,res|error} ·
#                 rtc-offer{rsid,sdp}   (SDP relayed from the /rtc websocket)
#     upstream    subs{topics[]} · teleop{sid,linearX,angularZ,ttlMs} · stop{sid} · goal{x,y,z} ·
#                 rpc{sid,id,target,method,args} · disconnect{sid} · rtc-answer{rsid,sdp|error}
#
# Safety invariants: disconnect{sid} → deadman-cancel + zero twist for that session (egress key
# (self, sid)); the pipe connection dropping → egress.disconnect for ALL its sids (the robot never keeps
# driving on a dead sidecar).
from __future__ import annotations

import asyncio
import json
import struct

from dimos.utils.logging_config import setup_logger

from .bus import Bus, Sample
from .egress import DEFAULT_TTL, SafetyEgress
from .transports._common import wants

logger = setup_logger()

KIND_DATA = 1
KIND_JSON = 2
LABEL = "dimoscope"  # base label; the sidecar appends the wire tag (WT-rs / rtc-rs) per session
QUEUE_MAX = 4096  # frames buffered toward the sidecar; localhost UDS — should never fill


class PipePlane:
    """One-listener UDS server; at most one sidecar connection at a time (a new one replaces the old)."""

    def __init__(self, bus: Bus, egress: SafetyEgress) -> None:
        self.bus = bus
        self.egress = egress
        self._conn: _Conn | None = None
        # rtc-answer{rsid,sdp|error} handler, installed by the /rtc signaling relay.
        self.on_rtc_answer: callable | None = None
        bus.subscribe(self._on_sample)
        bus.on_new_topic(self._on_new_topic)

    @property
    def connected(self) -> bool:
        """A live sidecar is on the pipe — gates /cert so a stale hash file can't 200 while
        nothing is listening on :WT_PORT."""
        return self._conn is not None

    def rtc_offer(self, rsid: int, sdp: str) -> bool:
        """Relay a browser SDP offer to the sidecar's WebRTC plane. False = no sidecar on the pipe."""
        conn = self._conn
        if conn is None:
            return False
        conn.send_json({"op": "rtc-offer", "rsid": rsid, "sdp": sdp})
        return True

    async def start(self, path: str) -> None:
        import contextlib
        import os

        with contextlib.suppress(FileNotFoundError):
            os.unlink(path)
        server = await asyncio.start_unix_server(self._on_connect, path=path)
        logger.info("wt pipe listening", path=path)
        async with server:
            await server.serve_forever()

    # bus fan-out (loop thread, keep cheap: one filter + one enqueue)
    def _on_sample(self, s: Sample) -> None:
        conn = self._conn
        if conn is not None and wants(conn.subs, s.topic):
            conn.send_data(s.lc02)

    def _on_new_topic(self, topic: str, typ: str) -> None:
        conn = self._conn
        if conn is not None:
            conn.send_json({"op": "topic", "topic": topic, "type": typ})

    async def _on_connect(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        if self._conn is not None:
            logger.warning("wt pipe: new sidecar connection replaces the old one")
            self._conn.close()
        conn = _Conn(self, reader, writer)
        self._conn = conn
        conn.send_json(
            {
                "op": "hello",
                "topics": self.bus.topic_list(),
                "label": LABEL,
                "rpc": self.egress.commands,
            }
        )
        try:
            await conn.run()
        finally:
            if self._conn is conn:
                self._conn = None
            conn.drop_all_sids()  # safety: dead sidecar must not keep the robot driving
            conn.close()


class _Conn:
    def __init__(
        self, plane: PipePlane, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        self.plane = plane
        self.reader = reader
        self.writer = writer
        # announced sub-union; on-demand — nothing flows until announced
        self.subs: set[str] = set()
        self.sids: set[str] = set()  # sessions that touched the write path (teleop/stop/rpc)
        self._q: asyncio.Queue[bytes] = asyncio.Queue(maxsize=QUEUE_MAX)
        self._dropped = 0
        self._writer_task = asyncio.ensure_future(self._drain())
        self._rpc_tasks: set = set()  # keep refs so scheduled rpc coros aren't GC'd

    # enqueue (never blocks the bus callback); overflow → drop-oldest, logged
    def _enqueue(self, frame: bytes) -> None:
        try:
            self._q.put_nowait(frame)
        except asyncio.QueueFull:
            try:
                self._q.get_nowait()
            except asyncio.QueueEmpty:
                pass
            try:
                self._q.put_nowait(frame)
            except asyncio.QueueFull:
                pass
            self._dropped += 1
            if self._dropped % 1000 == 1:
                logger.warning("wt pipe backpressure — dropping oldest", dropped=self._dropped)

    def send_data(self, lc02: bytes) -> None:
        self._enqueue(struct.pack(">IB", len(lc02) + 1, KIND_DATA) + lc02)

    def send_json(self, obj: dict) -> None:
        payload = json.dumps(obj).encode()
        self._enqueue(struct.pack(">IB", len(payload) + 1, KIND_JSON) + payload)

    async def _drain(self) -> None:
        try:
            while True:
                self.writer.write(await self._q.get())
                await self.writer.drain()
        except (asyncio.CancelledError, ConnectionError, OSError):
            pass

    async def run(self) -> None:
        try:
            while True:
                head = await self.reader.readexactly(5)
                length, kind = struct.unpack(">IB", head)
                body = await self.reader.readexactly(length - 1)
                if kind == KIND_JSON:
                    try:
                        self._on_op(json.loads(body))
                    except (ValueError, KeyError):
                        logger.warning("wt pipe: bad op", body=body[:120])
        except (asyncio.IncompleteReadError, ConnectionError, OSError):
            pass

    def _on_op(self, m: dict) -> None:
        egress = self.plane.egress
        op = m.get("op")
        if op == "subs":  # the sidecar's sub-union across all its sessions
            self.subs = set(m.get("topics") or [])
        elif op == "teleop":
            self.sids.add(m["sid"])
            egress.teleop(
                (self, m["sid"]),
                float(m.get("linearX", 0)),
                float(m.get("angularZ", 0)),
                float(m.get("ttlMs", DEFAULT_TTL)),
                asyncio.get_running_loop(),
            )
        elif op == "stop":
            self.sids.add(m["sid"])
            egress.stop((self, m["sid"]))
        elif op == "goal":
            egress.goal(m.get("x", 0), m.get("y", 0), m.get("z", 0))
        elif op == "rpc":
            task = asyncio.ensure_future(self._do_rpc(m))
            self._rpc_tasks.add(task)
            task.add_done_callback(self._rpc_tasks.discard)
        elif op == "disconnect":
            sid = m.get("sid")
            self.sids.discard(sid)
            egress.disconnect((self, sid))
        elif op == "rtc-answer":
            cb = self.plane.on_rtc_answer
            if cb is not None:
                cb(m)

    async def _do_rpc(self, m: dict) -> None:
        res = await self.plane.egress.rpc(
            m.get("target"), m.get("method"), m.get("args") or [], asyncio.get_running_loop()
        )
        self.send_json({"op": "rpc-res", "sid": m.get("sid"), "id": m.get("id"), **res})

    def drop_all_sids(self) -> None:
        for sid in self.sids:
            self.plane.egress.disconnect((self, sid))
        self.sids.clear()

    def close(self) -> None:
        self._writer_task.cancel()
        try:
            self.writer.close()
        except Exception:
            pass

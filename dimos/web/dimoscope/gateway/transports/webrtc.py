#!/usr/bin/env python3
# WebRTC DataChannel transport — the same read-path machinery as /ws and the WT sidecar (on-demand
# subscriptions, per-client PriorityOutbox, 1100 B size routing), mapped onto SCTP's primitives so the
# transports compare fairly:
#
#   ctl   ordered+reliable    one JSON per message: subscribe/unsubscribe/list/ping in · hello/topic/
#                             topics/pong out (data.py's read-only control subset — no teleop/rpc)
#   pose  unordered+lossy     [f64 send-ms][LC02] frames ≤ DATAGRAM_MAX, one per message (the QUIC-
#                             datagram analogue; dropped instead of queued when the buffer is full)
#   bulk  ordered+reliable    [u32be len][frame] chunked into ≤CHUNK messages — ordered+reliable, so
#                             the client concatenates messages and length-prefix-parses exactly like
#                             the WT bulk stream (SCTP maxMessageSize is far below a 1 MB frame)
#
# Two outboxes + two drain tasks per client: a bulk send waiting on bufferedAmount must never block
# pose (the WT sidecar's split-drain lesson). Signaling stays a WS at /rtc; ICE uses a public STUN
# server on both sides — no TURN, a relay would benchmark the relay.
from __future__ import annotations

import asyncio
import json
import struct
import time

from fastapi import WebSocket, WebSocketDisconnect

from ..bus import Bus, Sample
from ..qos import PriorityOutbox, declared_to_class, default_priority
from ._common import frame, wants

try:
    from aiortc import RTCConfiguration, RTCIceServer, RTCPeerConnection, RTCSessionDescription

    HAS_WEBRTC_DATA = True
except Exception:  # pragma: no cover
    HAS_WEBRTC_DATA = False

DATAGRAM_MAX = 1100  # same size split as the WT sidecar: ≤ → pose channel, > → bulk channel
CHUNK = 60_000  # bulk chunk size; safely under the 64 KB SCTP message-size interop floor
POSE_BUF_MAX = 64_000  # pose sends are dropped (not queued) beyond this — datagram semantics
BULK_BUF_HIGH = 512_000  # bulk drain waits below this; backlog then accumulates in the outbox
STUN = "stun:stun.l.google.com:19302"


class RtcSession:
    """Per-client read-path state, transport-agnostic (aiortc wiring stays in the plane): the same
    subs/rate/qos handling as data.py, with the frame stream split across two outboxes by size."""

    def __init__(self) -> None:
        self.subs: set[str] = set()
        self.rate: dict[str, float] = {}
        self.last: dict[str, float] = {}
        self.qos: dict[str, tuple] = {}
        self.pose_q = PriorityOutbox()
        self.bulk_q = PriorityOutbox()

    def offer_sample(self, topic: str, default: tuple, framed: bytes, now: float) -> None:
        if not wants(self.subs, topic):
            return  # on-demand: unsubscribed topics never transit
        hz = self.rate.get(topic, 0)
        if hz > 0:
            if now - self.last.get(topic, 0) < 1000.0 / hz:
                return  # per-topic downsample
            self.last[topic] = now
        prio, conflate, depth = self.qos.get(topic, default)
        # framed = [8-byte stamp][LC02]; route on the full frame size like the sidecar
        q = self.pose_q if len(framed) <= DATAGRAM_MAX else self.bulk_q
        q.put_data(topic, prio, conflate, depth, framed)

    def on_control(self, m: dict, topics: dict[str, str]) -> dict | None:
        """Apply one control op; returns the reply for the ctl channel (or None). The read-only
        subset of data.py's `_on_control` — teleop/goal/rpc stay on the duplex transports."""
        op = m.get("op")
        if op == "subscribe":
            t = m["topic"]
            self.subs.add(t)
            if m.get("maxHz"):
                self.rate[t] = float(m["maxHz"])
            if any(m.get(k) is not None for k in ("priority", "reliability", "depth")):
                default = default_priority(t, topics.get(t, ""))
                self.qos[t] = declared_to_class(
                    m.get("priority"), m.get("reliability"), m.get("depth"), default
                )
            else:
                self.qos.pop(t, None)
        elif op == "unsubscribe":
            t = m["topic"]
            self.subs.discard(t)
            self.rate.pop(t, None)
            self.last.pop(t, None)
            self.qos.pop(t, None)
        elif op == "list":
            return {
                "op": "topics",
                "topics": [{"topic": t, "type": ty} for t, ty in topics.items()],
            }
        elif op == "ping":  # clock-sync probe, same shape as data.py — keeps latency rows corrected
            return {"op": "pong", "id": m.get("id"), "serverTs": time.time() * 1000.0}
        return None


def bulk_chunks(framed: bytes) -> list[bytes]:
    """[u32be len][frame], split into ≤CHUNK messages. The channel is ordered+reliable, so the client
    treats the concatenation as a byte stream (identical parser to the WT bulk lane)."""
    buf = struct.pack(">I", len(framed)) + framed
    return [buf[i : i + CHUNK] for i in range(0, len(buf), CHUNK)]


async def _await_ice(pc) -> None:
    """Block until ICE gathering completes (non-trickle)."""
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
        self.sessions: dict = {}  # ws -> RtcSession
        self._ctl: dict = {}  # ws -> ctl channel
        bus.subscribe(self._on_sample)
        bus.on_new_topic(self._on_new_topic)

    # bus fan-out (loop thread, keep cheap: frame once, enqueue per session)
    def _on_sample(self, s: Sample) -> None:
        if not self.sessions:
            return
        now = time.time() * 1000.0
        default = default_priority(s.topic, s.type)
        framed: bytes | None = None
        for sess in list(self.sessions.values()):
            if not wants(sess.subs, s.topic):
                continue
            if framed is None:
                framed = frame(s)  # [f64 gateway-send-ms][LC02], stamped once
            sess.offer_sample(s.topic, default, framed, now)

    def _on_new_topic(self, topic: str, typ: str) -> None:
        msg = json.dumps({"op": "topic", "topic": topic, "type": typ})
        for ch in list(self._ctl.values()):
            self._ctl_send(ch, msg)

    @staticmethod
    def _ctl_send(ch, text: str) -> None:
        try:
            if ch.readyState == "open":
                ch.send(text)
        except Exception:
            pass

    async def handle(self, ws: WebSocket) -> None:
        # `ws: WebSocket` MUST be annotated — without the type, FastAPI treats it as a query-param
        # dependency and rejects the /rtc handshake with HTTP 403. /ws and /media do the same.
        await ws.accept()
        if not HAS_WEBRTC_DATA:
            await ws.close()
            return
        pc = RTCPeerConnection(RTCConfiguration(iceServers=[RTCIceServer(urls=STUN)]))
        sess = RtcSession()
        self.sessions[ws] = sess
        drains: list[asyncio.Task] = []

        @pc.on("datachannel")
        def on_datachannel(channel) -> None:
            if channel.label == "ctl":
                self._ctl[ws] = channel
                self._ctl_send(
                    channel,
                    json.dumps(
                        {
                            "op": "hello",
                            "topics": self.bus.topic_list(),
                            "label": "dimoscope/rtc",
                            "rpc": [],  # read-only plane: teleop/goal/rpc stay on /ws + WT
                        }
                    ),
                )

                @channel.on("message")
                def on_ctl(msg) -> None:
                    try:
                        reply = sess.on_control(json.loads(msg), self.bus.topics)
                    except (ValueError, KeyError):
                        return
                    if reply is not None:
                        self._ctl_send(channel, json.dumps(reply))

            elif channel.label == "pose":
                drains.append(asyncio.ensure_future(self._drain_pose(channel, sess)))
            elif channel.label == "bulk":
                drains.append(asyncio.ensure_future(self._drain_bulk(channel, sess)))

        try:
            while True:
                m = json.loads(await ws.receive_text())
                if m.get("op") == "offer":
                    await pc.setRemoteDescription(RTCSessionDescription(sdp=m["sdp"], type="offer"))
                    await pc.setLocalDescription(await pc.createAnswer())
                    await _await_ice(pc)  # non-trickle: gather, then answer
                    await ws.send_text(json.dumps({"op": "answer", "sdp": pc.localDescription.sdp}))
        except (WebSocketDisconnect, json.JSONDecodeError, RuntimeError):
            pass
        finally:
            for t in drains:
                t.cancel()
            self.sessions.pop(ws, None)
            self._ctl.pop(ws, None)
            await pc.close()

    async def _drain_pose(self, ch, sess: RtcSession) -> None:
        """Datagram semantics: never wait on the SCTP buffer — a full buffer drops the frame, so a
        stalled association can't turn pose into a queue of stale samples."""
        try:
            while True:
                framed = await sess.pose_q.get()
                if ch.readyState != "open":
                    return
                if ch.bufferedAmount > POSE_BUF_MAX:
                    continue  # drop: freshness over completeness
                ch.send(framed)
        except asyncio.CancelledError:
            pass
        except Exception:
            pass

    async def _drain_bulk(self, ch, sess: RtcSession) -> None:
        """Reliable lane with real backpressure: wait below BULK_BUF_HIGH before each chunk so the
        backlog accumulates in the outbox (where conflation keeps it fresh), not in aiortc's buffer."""
        low = asyncio.Event()
        ch.bufferedAmountLowThreshold = BULK_BUF_HIGH // 2

        @ch.on("bufferedamountlow")
        def _on_low() -> None:
            low.set()

        try:
            while True:
                framed = await sess.bulk_q.get()
                for chunk in bulk_chunks(framed):
                    while ch.bufferedAmount > BULK_BUF_HIGH:
                        low.clear()
                        await low.wait()
                    if ch.readyState != "open":
                        return
                    ch.send(chunk)
        except asyncio.CancelledError:
            pass
        except Exception:
            pass

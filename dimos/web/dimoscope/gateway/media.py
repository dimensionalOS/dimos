#!/usr/bin/env python3
# dimoscope media plane — camera Image → WebRTC / WebCodecs / JPEG, served on /media.
#
# Reads camera frames from the shared Bus (LCM+Zenoh merged) rather than opening its own Zenoh peer —
# so there is ONE bus session for the whole service. Encoding is CPU-heavy (PyAV/libx264, aiortc), so it
# runs in a single-worker executor
# (NOT on the event loop, and never two encoders at once) and hands NAL chunks back to the loop.
#
# Two delivery paths (the browser negotiates; both encode ONCE and fan out to N viewers):
#   • WebRTC  (aiortc)             — recvonly offer/answer; browser HW-decodes a <video>.
#   • WebCodecs (libx264 Annex-B)  — {video-config} JSON then binary
#       [u8 flags(bit0=keyframe)][u64 ts_us BE][u16 topic_len BE][topic utf8][H.264 Annex-B NAL]
# JPEG is the floor — decoded in the browser straight off the data plane, so it needs nothing here.
from __future__ import annotations

import asyncio
from concurrent.futures import ThreadPoolExecutor
import json
import struct
import time

from fastapi import WebSocket, WebSocketDisconnect

from .bus import Bus, Sample

# WebRTC (optional): re-encode the camera Image as a video track via aiortc.
try:
    from aiortc import RTCPeerConnection, RTCSessionDescription

    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.teleop.quest_hosted.video_track import CameraVideoTrack

    HAS_WEBRTC = True
except Exception:  # pragma: no cover - depends on optional aiortc/av install
    HAS_WEBRTC = False

# WebCodecs (optional): encode the camera Image to raw H.264 (Annex-B) with PyAV.
try:
    from fractions import Fraction

    import av

    from dimos.msgs.sensor_msgs.Image import Image

    try:
        from dimos.teleop.quest_hosted.video_track import _AV_FORMAT_MAP as _AV_FMT
    except Exception:
        _AV_FMT = {}
    HAS_WEBCODECS = True
except Exception:  # pragma: no cover - depends on optional av install
    HAS_WEBCODECS = False

MEDIA_KINDS = (
    (["webcodecs"] if HAS_WEBCODECS else []) + (["webrtc"] if HAS_WEBRTC else []) + ["jpeg"]
)


class MediaPlane:
    def __init__(self, bus: Bus) -> None:
        self.bus = bus
        # WebRTC: camera topic -> CameraVideoTracks wanting its frames; ws -> live PCs.
        self.webrtc_tracks: dict[str, set] = {}
        self.webrtc_pcs: dict[object, list] = {}
        # WebCodecs: camera topic -> set of ws wanting H.264 chunks; one encoder per topic.
        self.webcodecs_subs: dict[str, set] = {}
        self.encoders: dict[str, object] = {}
        self.force_key: set[str] = set()
        self._in_q: asyncio.Queue = asyncio.Queue(maxsize=256)  # (topic, payload) → encode worker
        self._video_q: asyncio.Queue = asyncio.Queue()  # (topic, nal, is_key, ts_us) → fanout
        self._exec = ThreadPoolExecutor(
            max_workers=1
        )  # serialise encode → encoders stay single-thread
        bus.subscribe(self._on_sample)

    # ── bus tap (loop thread, cheap): only camera topics with live viewers ──
    def _on_sample(self, s: Sample) -> None:
        if "Image" not in (s.type or ""):  # camera frames are sensor_msgs.Image
            return
        trs = self.webrtc_tracks.get(s.topic) if HAS_WEBRTC else None
        wc = self.webcodecs_subs.get(s.topic) if HAS_WEBCODECS else None
        if not (trs or wc):
            return
        try:
            self._in_q.put_nowait((s.topic, s.payload))
        except asyncio.QueueFull:
            pass  # camera is freshest-wins; drop under backpressure

    # ── background tasks (started in app.py's lifespan) ─────────────────────
    async def run_encoder(self) -> None:
        loop = asyncio.get_running_loop()
        while True:
            topic, payload = await self._in_q.get()
            await loop.run_in_executor(self._exec, self._process, topic, payload, loop)

    def _process(self, topic: str, payload: bytes, loop) -> None:
        """Executor thread: decode the Image once, feed WebRTC tracks + the WebCodecs encoder."""
        try:
            img = Image.lcm_decode(payload)
        except Exception:
            return
        trs = self.webrtc_tracks.get(topic)
        if trs:
            for tr in list(trs):
                tr.set_latest(img)
        if self.webcodecs_subs.get(topic):
            self._encode_webcodecs(topic, img, loop)

    def _encode_webcodecs(self, topic: str, img, loop) -> None:
        try:
            data = img.data
            h, w = int(data.shape[0]), int(data.shape[1])
            enc = self.encoders.get(topic)
            if enc is None:
                enc = av.CodecContext.create("libx264", "w")
                enc.width, enc.height, enc.pix_fmt = w, h, "yuv420p"
                enc.framerate = Fraction(
                    30, 1
                )  # real fps → correct rate control (else blocky garbage)
                enc.time_base = Fraction(1, 1_000_000)  # pts in µs
                enc.options = {
                    "tune": "zerolatency",
                    "preset": "veryfast",
                    "profile": "baseline",  # avc1.42e0 — universal hardware decode
                    "crf": "23",
                    "g": "30",  # IDR cadence
                    "bf": "0",  # no B-frames → lower latency
                    "x264-params": "repeat-headers=1",  # SPS/PPS before every IDR → late joiners decode
                }
                self.encoders[topic] = enc
            frame = av.VideoFrame.from_ndarray(
                data, format=_AV_FMT.get(getattr(img, "format", None), "bgr24")
            )
            frame.pts = int(time.time() * 1_000_000)
            frame.time_base = Fraction(1, 1_000_000)
            if topic in self.force_key:
                try:
                    frame.pict_type = av.video.frame.PictureType.I
                except Exception:
                    pass
                self.force_key.discard(topic)
            for pkt in enc.encode(frame):
                buf = bytes(pkt)
                if buf:
                    ts = int(pkt.pts) if pkt.pts is not None else int(time.time() * 1_000_000)
                    loop.call_soon_threadsafe(
                        self._video_q.put_nowait, (topic, buf, bool(pkt.is_keyframe), ts)
                    )
        except Exception:
            pass  # an encode hiccup must never disturb other viewers

    async def run_fanout(self) -> None:
        # WebCodecs chunk wire: [u8 flags(bit0=key)][u64 ts_us BE][u16 topic_len BE][topic][NAL]
        while True:
            topic, buf, is_key, ts_us = await self._video_q.get()
            subs = self.webcodecs_subs.get(topic)
            if not subs:
                continue
            tb = topic.encode()
            head = struct.pack(">BQH", 1 if is_key else 0, ts_us & 0xFFFFFFFFFFFFFFFF, len(tb))
            frame = head + tb + buf
            for ws in list(subs):
                try:
                    await ws.send_bytes(frame)
                except Exception:
                    subs.discard(ws)

    # ── WebRTC helpers ──────────────────────────────────────────────────────
    def _add_track(self, topic: str, track) -> None:
        self.webrtc_tracks.setdefault(topic, set()).add(track)

    def _drop_track(self, topic: str, track) -> None:
        s = self.webrtc_tracks.get(topic)
        if s is not None:
            s.discard(track)
            if not s:
                self.webrtc_tracks.pop(topic, None)

    async def _close_pcs(self, ws) -> None:
        for pc, track, topic in self.webrtc_pcs.pop(ws, []):
            self._drop_track(topic, track)
            try:
                await pc.close()
            except Exception:
                pass

    async def _handle_webrtc_offer(self, ws, m: dict) -> None:
        topic, sdp = m.get("topic"), m.get("sdp")
        if not topic or not sdp:
            return
        await self._close_pcs(ws)
        track = CameraVideoTrack()
        pc = RTCPeerConnection()
        self._add_track(topic, track)
        self.webrtc_pcs.setdefault(ws, []).append((pc, track, topic))
        pc.addTrack(track)

        @pc.on("connectionstatechange")
        async def _on_state() -> None:
            if pc.connectionState == "connected":
                track.arm()  # start the operator's video at "now", not robot boot
            elif pc.connectionState in ("failed", "closed"):
                self._drop_track(topic, track)

        await pc.setRemoteDescription(RTCSessionDescription(sdp=sdp, type="offer"))
        await pc.setLocalDescription(await pc.createAnswer())
        if pc.iceGatheringState != "complete":
            fut = asyncio.get_running_loop().create_future()

            @pc.on("icegatheringstatechange")
            def _on_gathering() -> None:
                if pc.iceGatheringState == "complete" and not fut.done():
                    fut.set_result(None)

            await fut
        await ws.send_text(json.dumps({"op": "webrtc-answer", "sdp": pc.localDescription.sdp}))

    # ── per-client connection ───────────────────────────────────────────────
    async def handle(self, ws: WebSocket) -> None:
        await ws.accept()
        await ws.send_text(json.dumps({"op": "hello", "label": "media", "media": MEDIA_KINDS}))
        try:
            while True:
                m = json.loads(await ws.receive_text())
                op = m.get("op")
                if op == "webrtc-offer" and HAS_WEBRTC:
                    await self._handle_webrtc_offer(ws, m)
                elif op == "webrtc-stop":
                    await self._close_pcs(ws)
                elif op == "webcodecs-start" and HAS_WEBCODECS:
                    t = m.get("topic")
                    if t:
                        self.webcodecs_subs.setdefault(t, set()).add(ws)
                        self.force_key.add(t)  # emit an IDR so this viewer starts fast
                        await ws.send_text(
                            json.dumps({"op": "video-config", "topic": t, "codec": "avc1.42E01F"})
                        )
                elif op == "webcodecs-stop":
                    s = self.webcodecs_subs.get(m.get("topic"))
                    if s is not None:
                        s.discard(ws)
        except (WebSocketDisconnect, json.JSONDecodeError, RuntimeError):
            pass
        finally:
            await self._close_pcs(ws)
            for s in self.webcodecs_subs.values():
                s.discard(ws)

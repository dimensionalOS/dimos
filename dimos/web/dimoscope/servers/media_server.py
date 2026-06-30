#!/usr/bin/env python3
# dimos-media-server (Zenoh camera → WebRTC / WebCodecs) — the standalone media node.
#
# Split out of the data gateway: encoding the camera is CPU-heavy (PyAV/libx264, aiortc) and has a
# different lifecycle than the latency-sensitive byte-relay + the teleop/RPC trust boundary. This is
# its OWN Zenoh peer, so the camera reaches the browser regardless of which DATA transport carries the
# topics (Bun↔LCM / Python↔Zenoh / zenoh-ts). It publishes NOTHING to the bus — read-only camera.
#
# Two delivery paths (the browser picks via capability negotiation; both encode ONCE, fan out to N):
#   • WebRTC  (aiortc)             — recvonly offer/answer; browser HW-decodes a <video>.
#   • WebCodecs (libx264 Annex-B)  — {video-config} JSON then binary
#       [u8 flags(bit0=keyframe)][u64 ts_us BE][u16 topic_len BE][topic utf8][H.264 Annex-B NAL]
#
# RUN:  MEDIA_PORT=8092 MEDIA_KEY='**' .venv/bin/python servers/media_server.py
import asyncio
import json
import os
import struct
import time

os.environ["DIMOS_TRANSPORT"] = "zenoh"  # this node is a Zenoh peer (reads the camera off the bus)

import websockets
import zenoh

# WebRTC (optional): re-encode the camera Image as a video track via aiortc, reusing the same
# CameraVideoTrack the hosted-teleop module uses. Degrades to jpeg (data-plane floor) if absent.
try:
    from aiortc import RTCPeerConnection, RTCSessionDescription

    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.teleop.quest_hosted.video_track import CameraVideoTrack

    HAS_WEBRTC = True
except Exception:  # pragma: no cover - depends on optional aiortc/av install
    HAS_WEBRTC = False

# WebCodecs (optional): encode the camera Image to raw H.264 (Annex-B) with PyAV and stream the NAL
# chunks over the WS; the browser hardware-decodes them with VideoDecoder. No ICE/SDP.
try:
    import av
    from fractions import Fraction

    from dimos.msgs.sensor_msgs.Image import Image  # noqa: F811 (also imported above for webrtc)

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

PORT = int(os.environ.get("MEDIA_PORT", 8092))
KEY = os.environ.get("MEDIA_KEY", "**")

# WebRTC media: camera topic -> the CameraVideoTracks wanting its frames; ws -> live PCs.
webrtc_tracks: dict[str, set] = {}
webrtc_pcs: dict[object, list] = {}
# WebCodecs media: camera topic -> set of ws wanting H.264 chunks; one av encoder per topic
# (encode-once → fan out). force_key holds topics that must emit an IDR on the next frame so a
# late-joining viewer starts decoding immediately.
webcodecs_subs: dict[str, set] = {}
webcodecs_encoders: dict[str, object] = {}
webcodecs_force_key: set[str] = set()


def _add_track(topic: str, track: object) -> None:
    webrtc_tracks.setdefault(topic, set()).add(track)


def _drop_track(topic: str, track: object) -> None:
    s = webrtc_tracks.get(topic)
    if s is not None:
        s.discard(track)
        if not s:
            webrtc_tracks.pop(topic, None)


async def _close_pcs(ws: object) -> None:
    for pc, track, topic in webrtc_pcs.pop(ws, []):
        _drop_track(topic, track)
        try:
            await pc.close()
        except Exception:
            pass


def _encode_webcodecs(topic: str, img, loop, video_q) -> None:
    """Encode one camera frame to H.264 (Annex-B) and enqueue its NAL packet(s) for fan-out.

    Runs in the Zenoh callback thread; a topic's encoder is only ever touched here, so no lock is
    needed. Sending is handed to the event loop via call_soon_threadsafe (video_fanout drains it).
    """
    try:
        data = img.data
        h, w = int(data.shape[0]), int(data.shape[1])
        enc = webcodecs_encoders.get(topic)
        if enc is None:
            enc = av.CodecContext.create("libx264", "w")
            enc.width, enc.height, enc.pix_fmt = w, h, "yuv420p"
            # libx264 rate control needs the REAL fps: without it the per-frame bit budget is
            # mis-derived (the µs time_base implies an absurd fps) so every frame is starved →
            # max QP → blocky/washed-out garbage. This was the bug: framerate was never set.
            enc.framerate = Fraction(30, 1)
            enc.time_base = Fraction(1, 1_000_000)  # pts stay in µs
            enc.options = {
                "tune": "zerolatency",
                "preset": "veryfast",  # latency/quality balance
                "profile": "baseline",  # avc1.42e0 — universal hardware decode
                "crf": os.environ.get("WEBCODECS_CRF", "23"),  # constant-quality (replaces ABR bit_rate)
                "g": "30",  # IDR cadence (force_key gets late joiners going sooner)
                "bf": "0",  # no B-frames → lower latency, simpler decode order
                "x264-params": "repeat-headers=1",  # SPS/PPS before every IDR → late joiners decode
            }
            webcodecs_encoders[topic] = enc
        frame = av.VideoFrame.from_ndarray(data, format=_AV_FMT.get(getattr(img, "format", None), "bgr24"))
        frame.pts = int(time.time() * 1_000_000)
        frame.time_base = Fraction(1, 1_000_000)
        if topic in webcodecs_force_key:
            try:
                frame.pict_type = av.video.frame.PictureType.I
            except Exception:
                pass
            webcodecs_force_key.discard(topic)
        for pkt in enc.encode(frame):
            buf = bytes(pkt)
            if buf:
                ts = int(pkt.pts) if pkt.pts is not None else int(time.time() * 1_000_000)
                loop.call_soon_threadsafe(video_q.put_nowait, (topic, buf, bool(pkt.is_keyframe), ts))
    except Exception:
        pass  # an encode hiccup must never disturb other viewers


async def handle_webrtc_offer(ws: object, m: dict) -> None:
    """Browser → camera: answer a recvonly offer with a CameraVideoTrack for `topic`.

    One active camera PC per client. Non-trickle ICE: we gather then answer — instant on a LAN.
    on_sample() decodes the matching Image and feeds the track's frames.
    """
    topic = m.get("topic")
    sdp = m.get("sdp")
    if not topic or not sdp:
        return
    await _close_pcs(ws)  # drop any prior camera PC for this client
    track = CameraVideoTrack()
    pc = RTCPeerConnection()
    _add_track(topic, track)
    webrtc_pcs.setdefault(ws, []).append((pc, track, topic))
    pc.addTrack(track)

    @pc.on("connectionstatechange")
    async def _on_state() -> None:
        if pc.connectionState == "connected":
            track.arm()  # start the operator's video at "now", not robot boot
        elif pc.connectionState in ("failed", "closed"):
            _drop_track(topic, track)

    await pc.setRemoteDescription(RTCSessionDescription(sdp=sdp, type="offer"))
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    if pc.iceGatheringState != "complete":
        fut = asyncio.get_running_loop().create_future()

        @pc.on("icegatheringstatechange")
        def _on_gathering() -> None:
            if pc.iceGatheringState == "complete" and not fut.done():
                fut.set_result(None)

        await fut
    await ws.send(json.dumps({"op": "webrtc-answer", "sdp": pc.localDescription.sdp}))


async def main() -> None:
    loop = asyncio.get_running_loop()
    video_q: asyncio.Queue = asyncio.Queue()  # (topic, nal_bytes, is_key, ts_us) → video_fanout

    def on_sample(sample) -> None:
        key = str(sample.key_expr)
        try:
            payload = sample.payload.to_bytes()
        except Exception:
            payload = bytes(sample.payload)
        base, _, _typ = key.rpartition("/")  # dimos/color_image/sensor_msgs.Image
        topic = "/" + base
        # Decode the camera Image ONCE, then feed whichever consumers want it (only when subscribed).
        trs = webrtc_tracks.get(topic) if HAS_WEBRTC else None
        wc = webcodecs_subs.get(topic) if HAS_WEBCODECS else None
        if not (trs or wc):
            return
        try:
            img = Image.lcm_decode(payload)
        except Exception:
            return
        if trs:
            for tr in list(trs):
                tr.set_latest(img)
        if wc:
            _encode_webcodecs(topic, img, loop, video_q)

    session = zenoh.open(zenoh.Config())
    session.declare_subscriber(KEY, on_sample)
    print(f"[media] zenoh sub '{KEY}'  ·  ws://localhost:{PORT}  ·  {'+'.join(MEDIA_KINDS)}", flush=True)

    async def video_fanout() -> None:
        # WebCodecs chunks ride their own binary frame (a webcodecs ws only ever receives these):
        #   [u8 flags(bit0=keyframe)][u64 ts_us BE][u16 topic_len BE][topic utf8][H.264 Annex-B NAL]
        while True:
            topic, buf, is_key, ts_us = await video_q.get()
            subs = webcodecs_subs.get(topic)
            if not subs:
                continue
            tb = topic.encode()
            head = struct.pack(">BQH", 1 if is_key else 0, ts_us & 0xFFFFFFFFFFFFFFFF, len(tb))
            frame = head + tb + buf
            for ws in list(subs):
                try:
                    await ws.send(frame)
                except Exception:
                    subs.discard(ws)

    async def handler(ws) -> None:
        # Adapters ignore this hello (they only act on webrtc-answer / video-config), but it's a
        # useful probe (curl-able) of what the media node serves.
        await ws.send(json.dumps({"op": "hello", "label": "media", "media": MEDIA_KINDS}))
        try:
            async for raw in ws:
                if isinstance(raw, bytes):
                    continue
                m = json.loads(raw)
                op = m.get("op")
                if op == "webrtc-offer" and HAS_WEBRTC:
                    await handle_webrtc_offer(ws, m)
                elif op == "webrtc-stop":
                    await _close_pcs(ws)
                elif op == "webcodecs-start" and HAS_WEBCODECS:
                    t = m.get("topic")
                    if t:
                        webcodecs_subs.setdefault(t, set()).add(ws)
                        webcodecs_force_key.add(t)  # emit an IDR so this viewer starts fast
                        await ws.send(json.dumps({"op": "video-config", "topic": t, "codec": "avc1.42E01F"}))
                elif op == "webcodecs-stop":
                    s = webcodecs_subs.get(m.get("topic"))
                    if s is not None:
                        s.discard(ws)
        except Exception:
            pass
        finally:
            await _close_pcs(ws)
            for s in webcodecs_subs.values():
                s.discard(ws)

    async with websockets.serve(handler, "localhost", PORT, max_size=2**24):
        await video_fanout()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass

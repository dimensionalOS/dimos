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

from dimos.core.transport_factory import make_transport, rpc_backend
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3

# WebRTC media plane (optional): re-encode the camera Image stream as a video track via aiortc,
# reusing the same CameraVideoTrack the hosted-teleop module uses. If aiortc isn't installed the
# gateway still runs — it just advertises jpeg-only and the browser uses the Image-topic floor.
try:
    from aiortc import RTCPeerConnection, RTCSessionDescription

    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.teleop.quest_hosted.video_track import CameraVideoTrack

    HAS_WEBRTC = True
except Exception:  # pragma: no cover - depends on optional aiortc/av install
    HAS_WEBRTC = False

# WebCodecs media plane (optional): encode the camera Image to raw H.264 (Annex-B) with PyAV and
# stream the NAL chunks over the gateway WS; the browser hardware-decodes them with VideoDecoder.
# Same codec as WebRTC but no ICE/SDP — it rides the WS (works behind any data transport), gives the
# browser frame-level access, and ONE encoder per topic fans out to N viewers. Degrades to jpeg if
# PyAV isn't available.
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
# WebRTC media: camera topic -> the CameraVideoTracks wanting its frames; ws -> live PCs.
webrtc_tracks: dict[str, set] = {}
webrtc_pcs: dict[object, list] = {}
# WebCodecs media: camera topic -> set of ws wanting H.264 chunks; one av encoder per topic
# (encode-once → fan out). force_key holds topics that must emit an IDR on the next frame so a
# late-joining viewer starts decoding immediately.
webcodecs_subs: dict[str, set] = {}
webcodecs_encoders: dict[str, object] = {}
webcodecs_force_key: set[str] = set()
_rpc = None  # dimos RPC client (rpc_backend()()), started in main() if available
HAS_RPC = False
_seq = 0


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
            enc.time_base = Fraction(1, 1_000_000)
            enc.bit_rate = int(os.environ.get("WEBCODECS_BITRATE", 2_500_000))
            enc.options = {
                "tune": "zerolatency",
                "profile": "baseline",  # avc1.42e0 — universal hardware decode
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
        pass  # an encode hiccup must never disturb the data plane


async def handle_webrtc_offer(ws: object, m: dict) -> None:
    """Browser → camera: answer a recvonly offer with a CameraVideoTrack for `topic`.

    One active camera PC per client (slice scope). Non-trickle ICE: we gather then answer —
    instant on a LAN. on_sample() decodes the matching Image and feeds the track's frames.
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
    video_q: asyncio.Queue = asyncio.Queue()  # (topic, nal_bytes, is_key, ts_us) → video_fanout

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
        if topic not in topics:
            topics[topic] = typ
        # Media plane: decode the camera Image ONCE, then feed whichever consumers want it —
        # WebRTC tracks (aiortc re-encode) and/or the WebCodecs H.264 encoder (raw NAL over WS).
        trs = webrtc_tracks.get(topic) if HAS_WEBRTC else None
        wc = webcodecs_subs.get(topic) if HAS_WEBCODECS else None
        if trs or wc:
            try:
                img = Image.lcm_decode(payload)
            except Exception:
                img = None
            if img is not None:
                if trs:
                    for tr in list(trs):
                        tr.set_latest(img)
                if wc:
                    _encode_webcodecs(topic, img, loop, video_q)
        pkt = make_lc02(f"{topic}#{typ}", payload)
        loop.call_soon_threadsafe(out_q.put_nowait, (topic, pkt))

    session = zenoh.open(zenoh.Config())
    session.declare_subscriber(KEY, on_sample)
    print(f"[zgateway] zenoh sub '{KEY}'  ·  ws://localhost:{PORT}  ·  teleop→/cmd_vel goal→/clicked_point", flush=True)
    print(f"[zgateway] media: {'+'.join(MEDIA_KINDS)}", flush=True)
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

    async def video_fanout() -> None:
        # WebCodecs chunks ride their own binary frame so they never collide with LC02 data frames
        # (a webcodecs ws never subscribes data topics, so it only ever receives these):
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
        clients[ws] = set()
        await ws.send(
            json.dumps(
                {
                    "op": "hello",
                    "topics": topic_list(),
                    "label": "Python↔Zenoh",
                    "media": MEDIA_KINDS,
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
                elif op == "webrtc-offer" and HAS_WEBRTC:
                    await handle_webrtc_offer(ws, m)
                elif op == "webrtc-stop":
                    await _close_pcs(ws)
                elif op == "rpc":
                    await handle_rpc(ws, m)
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
            cancel_deadman(ws)
            publish_twist(0.0, 0.0)  # safety: stop the robot when an operator disconnects
            await _close_pcs(ws)
            for s in webcodecs_subs.values():
                s.discard(ws)
            clients.pop(ws, None)

    async with websockets.serve(handler, "localhost", PORT, max_size=2**24):
        await asyncio.gather(fanout(), video_fanout())


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass

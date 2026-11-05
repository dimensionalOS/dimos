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

"""Direct WebRTC H.264 video for topic routes.

One aiortc peer per viewer, one software encoder per (route, settings, rate
included), per-viewer packetization: N viewers of one setting cost one encode
and N packetizations. The relay signals offers, answers and candidates on the
topic control lane and never carries the media.

Imports aiortc and PyAV at module level; topic_service imports this module
lazily, only when ``video.enabled`` and both import.

Upgrade audit for the aiortc==1.14.0 pin (the aioquic one is in _wt_session.py):
* ``RTCRtpSender._send_keyframe`` is replaced per sender, so a viewer's PLI or
  FIR (consumed in ``_handle_rtcp_packet``) reaches the shared encoder.
* ``RTCRtpSender._next_encoded_frame`` packs an ``av.Packet`` returned by
  ``track.recv()`` with ``H264Encoder.pack`` instead of encoding a frame.
* ``aiortc.sdp.candidate_from_sdp`` parses a browser candidate line without
  its ``candidate:`` prefix.
* ``RTCPeerConnection.setRemoteDescription`` raises ``OperationError`` only when
  no offered codec is among the transceiver's preferences.
"""

from __future__ import annotations

import asyncio
from collections import deque
from collections.abc import Callable, Mapping
from contextlib import suppress
from dataclasses import dataclass, field
import fractions
from threading import Lock
import time
from typing import Any

from aiortc import (
    RTCConfiguration,
    RTCPeerConnection,
    RTCRtpSender,
    RTCSessionDescription,
)
from aiortc.exceptions import OperationError
from aiortc.mediastreams import MediaStreamError, MediaStreamTrack
from aiortc.sdp import candidate_from_sdp
import av
import numpy as np

from dimos.utils.logging_config import setup_logger
from dimos.web.relay_bridge.topic_media import decode_color
from dimos.web.relay_bridge.topic_profiles import video_bitrate
from dimos.web.relay_bridge.topic_protocol import (
    MAX_TOPIC_RTC_CANDIDATES,
    TopicRtcAnswer,
    TopicRtcCandidate,
    TopicRtcClose,
    TopicRtcOffer,
)
from dimos.web.relay_bridge.topic_transform import TopicTransforms

logger = setup_logger()

VIDEO_CLOCK_RATE = 90_000
VIDEO_TIME_BASE = fractions.Fraction(1, VIDEO_CLOCK_RATE)
# A newly active peer and PLI/FIR force the next frame to be an IDR, at most
# this often per encoder; the safety keyframe interval bounds a lost one.
KEYFRAME_FORCE_S = 0.5
SAFETY_KEYFRAME_S = 4.0
# Encoded frames queued per peer; an overflowing peer drops its queue and
# waits for the next IDR.
PEER_QUEUE = 2
# Consecutive encode failures that rebuild the encoder, then end its peers.
REBUILD_FAILURES = 3
FATAL_FAILURES = 6
PEER_CLOSE_S = 2.0
NEGOTIATE_S = 5.0
SOFTWARE_ENCODERS = ("libopenh264", "libx264")


def resolve_encoder(name: str) -> str:
    """The PyAV encoder for a ``video.encoder`` setting, or ValueError.

    ``software`` (and ``auto``, until hardware encoders exist) picks
    libopenh264 when the wheel has it, else libx264; an explicit name must be
    an H.264 encoder of this PyAV build.
    """
    available = set(av.codecs_available)
    if name in ("auto", "software"):
        for candidate in SOFTWARE_ENCODERS:
            if candidate in available:
                return candidate
        raise ValueError("no software H.264 encoder in this PyAV build")
    if name not in available:
        raise ValueError(f"unknown video encoder {name!r}")
    try:
        codec = av.Codec(name, "w")
    except av.FFmpegError as error:
        raise ValueError(f"{name!r} is not an encoder") from error
    if codec.id != av.Codec("h264", "r").id:
        raise ValueError(f"{name!r} does not encode H.264")
    return name


def fit_box(width: int, height: int, box: Mapping[int, int] | list[int]) -> tuple[int, int]:
    """The even output size that fits ``box`` without upscaling the source."""
    scale = min(1.0, box[0] / width, box[1] / height)
    return max(2, int(width * scale) // 2 * 2), max(2, int(height * scale) // 2 * 2)


def require_h264(pc: RTCPeerConnection) -> None:
    """Negotiate H.264 and its retransmission only: the sender packetizes
    encoded H.264 as is, which a peer that negotiated VP8 could never decode."""
    capabilities = RTCRtpSender.getCapabilities("video")
    codecs = [c for c in capabilities.codecs if c.mimeType.lower() in ("video/h264", "video/rtx")]
    for transceiver in pc.getTransceivers():
        if transceiver.kind == "video":
            transceiver.setCodecPreferences(codecs)


class PacketTrack(MediaStreamTrack):
    """A sendonly track whose ``recv()`` yields encoded packets, so aiortc
    packetizes instead of encoding. Bounded; waits for an IDR after a drop."""

    kind = "video"

    def __init__(self) -> None:
        super().__init__()
        self._queue: deque[av.Packet] = deque()
        self._wake = asyncio.Event()
        self.waiting_idr = True
        self.dropped = 0
        self.sent = 0

    def offer(self, packet: av.Packet) -> bool:
        """Queue one encoded frame; True when the peer now needs a keyframe."""
        if self.waiting_idr:
            if not packet.is_keyframe:
                return False
            self.waiting_idr = False
        if len(self._queue) >= PEER_QUEUE:
            self._queue.clear()
            self.dropped += 1
            self.waiting_idr = True
            return True
        self._queue.append(packet)
        self._wake.set()
        return False

    async def recv(self) -> av.Packet:
        while not self._queue:
            if self.readyState != "live":
                raise MediaStreamError
            self._wake.clear()
            await self._wake.wait()
        self.sent += 1
        return self._queue.popleft()

    def wait_for_keyframe(self) -> None:
        """Discard queued frames from an encoder configuration being replaced."""
        self._queue.clear()
        self.waiting_idr = True

    def stop(self) -> None:
        super().stop()
        self._wake.set()


class SharedEncoder:
    """One H.264 encoder per (route, settings), run in a worker thread under a
    transform permit; every peer of the settings gets every packet."""

    def __init__(self, codec_name: str, settings: Mapping[str, Any]) -> None:
        self.codec_name = codec_name
        self.box: list[int] = list(settings["maxResolution"])
        self.max_hz = float(settings["maxHz"])
        self.max_bitrate = int(settings.get("maxBitrate", video_bitrate(*self.box)))
        self._configuration_lock = Lock()
        self._pending_settings: dict[str, Any] | None = None
        self.revision = 0
        self.applied_revision = 0
        self.peers: dict[str, Peer] = {}
        self._codec: av.VideoCodecContext | None = None
        self._size: tuple[int, int] | None = None
        self._keyframe_wanted = False
        self._last_forced = float("-inf")
        self._started = time.monotonic()
        self._last_pts = -1
        self.failures = 0
        self.frames = 0
        self.keyframes = 0
        self.dropped = 0

    @property
    def deadline(self) -> float:
        """An encode slower than this is stale: dropped, and a keyframe follows."""
        return max(2 / self.max_hz, 0.5)

    def request_keyframe(self, *, now: bool = False) -> None:
        """Make the next frame an IDR: at most once per KEYFRAME_FORCE_S for a
        joining or asking (PLI, FIR) peer, at once after a dropped frame."""
        self._keyframe_wanted = True
        if now:
            self._last_forced = float("-inf")

    def reset(self) -> None:
        """Rebuild the codec on the next frame (after repeated failures)."""
        self._codec = None
        self._size = None

    def reconfigure(self, settings: Mapping[str, Any]) -> None:
        """Apply at the next encode boundary, never mutate an in-flight codec."""
        with self._configuration_lock:
            self._pending_settings = dict(settings)
            self.revision += 1

    def _apply_settings(self) -> None:
        with self._configuration_lock:
            settings, self._pending_settings = self._pending_settings, None
            self.applied_revision = self.revision
        if settings is not None:
            self.box = list(settings["maxResolution"])
            self.max_hz = float(settings["maxHz"])
            self.max_bitrate = int(settings["maxBitrate"])
            self.reset()

    def _open(self, width: int, height: int) -> av.VideoCodecContext:
        codec = av.CodecContext.create(self.codec_name, "w")
        assert isinstance(codec, av.VideoCodecContext)
        codec.width = width
        codec.height = height
        codec.pix_fmt = "yuv420p"
        codec.bit_rate = self.max_bitrate
        codec.time_base = VIDEO_TIME_BASE
        codec.framerate = fractions.Fraction(self.max_hz).limit_denominator(1000)
        codec.gop_size = max(1, round(SAFETY_KEYFRAME_S * self.max_hz))
        codec.max_b_frames = 0
        options = {"level": "31"}
        if self.codec_name == "libx264":
            # No scene-cut IDRs: a live camera stream keeps its fixed cadence
            # (the safety keyframe and PLI/FIR cover recovery) and its bitrate.
            options.update(tune="zerolatency", preset="veryfast", sc_threshold="0")
            codec.profile = "Baseline"
        elif self.codec_name == "libopenh264":
            options.update(profile="constrained_baseline", allow_skip_frames="0")
        codec.options = options
        codec.open()
        self._codec = codec
        self._size = (width, height)
        return codec

    def encode(self, payload: bytes) -> list[av.Packet]:
        """Decode an Image/CompressedImage sample and encode it; worker thread."""
        self._apply_settings()
        image, _ = decode_color(payload)
        try:
            width, height = fit_box(image.width, image.height, self.box)
            rgb = np.asarray(image, dtype=np.uint8)
        finally:
            image.close()
        frame = av.VideoFrame.from_ndarray(rgb, format="rgb24").reformat(
            width, height, "yuv420p", interpolation="BILINEAR"
        )
        codec = self._codec
        fresh = codec is None or self._size != (width, height)
        if fresh:
            codec = self._open(width, height)
        assert codec is not None
        now = time.monotonic()
        force = not fresh and self._keyframe_wanted and now - self._last_forced >= KEYFRAME_FORCE_S
        if fresh or force:
            self._keyframe_wanted = False
            self._last_forced = now
        frame.pict_type = av.video.frame.PictureType.I if force else av.video.frame.PictureType.NONE
        # Monotonic PTS from the encoder's own clock; aiortc requires it never to go back.
        pts = max(self._last_pts + 1, int((now - self._started) * VIDEO_CLOCK_RATE))
        self._last_pts = pts
        frame.pts = pts
        frame.time_base = VIDEO_TIME_BASE
        packets = list(codec.encode(frame))
        self.frames += 1
        self.keyframes += sum(1 for packet in packets if packet.is_keyframe)
        return packets


@dataclass
class Peer:
    token: str
    route: str
    gen: int | float
    variant: str
    pc: RTCPeerConnection
    track: PacketTrack
    encoder: SharedEncoder
    connected: bool = False
    closing: bool = False
    candidates: int = 0
    opened_at: float = field(default_factory=time.monotonic)


class VideoRoute:
    """The encoders of one video route, keyed by variant."""

    def __init__(self, video: TopicVideo, route: str) -> None:
        self._video = video
        self.route = route
        self.encoders: dict[str, SharedEncoder] = {}

    def reconfigure(self, variant: str, settings: Mapping[str, Any]) -> None:
        encoder = self.encoders.get(variant)
        if encoder is not None:
            encoder.reconfigure(settings)
            for peer in encoder.peers.values():
                peer.track.wait_for_keyframe()

    async def encode(self, variant: str, payload: bytes) -> None:
        """Encode once for every connected peer of ``variant``; permit held."""
        encoder = self.encoders.get(variant)
        if encoder is None or not any(peer.connected for peer in encoder.peers.values()):
            return
        started = time.monotonic()
        try:
            packets = await self._video.transforms.run(encoder.encode, payload)
        except Exception as error:
            encoder.failures += 1
            if encoder.failures >= FATAL_FAILURES:
                logger.error("video encoder failed", route=self.route, error=str(error))
                await self.close_variant(variant, "encoder_failed")
            elif encoder.failures >= REBUILD_FAILURES:
                logger.warning("video encoder rebuilt", route=self.route, error=str(error))
                encoder.reset()
            return
        if encoder.revision != encoder.applied_revision:
            # Encoded before the reconfigure reached the worker: the next encode
            # applies it, from an IDR. An encode that applied it already is kept.
            return
        encoder.failures = 0
        if time.monotonic() - started > encoder.deadline:
            # The frame is stale, and later frames reference it: the next one
            # is an IDR whatever the force window says, and until it arrives
            # the peers skip frames that would decode broken.
            encoder.dropped += 1
            encoder.request_keyframe(now=True)
            for peer in encoder.peers.values():
                peer.track.waiting_idr = True
            return
        for packet in packets:
            for peer in list(encoder.peers.values()):
                if peer.connected and peer.track.offer(packet):
                    encoder.request_keyframe()

    async def close_variant(self, variant: str, reason: str) -> None:
        encoder = self.encoders.pop(variant, None)
        if encoder is None:
            return
        for token in list(encoder.peers):
            await self._video.close_peer(token, reason)

    async def close(self, reason: str = "route_closed") -> None:
        self._video._routes.pop(self.route, None)
        for variant in list(self.encoders):
            await self.close_variant(variant, reason)


class TopicVideo:
    """The provider's direct video peers across every route, under the
    ``max_peers`` and ``max_encoders`` limits."""

    def __init__(
        self,
        codec_name: str,
        max_peers: int,
        max_encoders: int,
        send: Callable[[Any], Any],
        transforms: TopicTransforms,
        *,
        peer_factory: Callable[[], RTCPeerConnection] | None = None,
    ) -> None:
        self.codec_name = codec_name
        self.max_peers = max_peers
        self.max_encoders = max_encoders
        self.transforms = transforms
        self._send = send
        self._peer_factory = peer_factory or (
            lambda: RTCPeerConnection(RTCConfiguration(iceServers=[]))
        )
        self._peers: dict[str, Peer] = {}
        self._routes: dict[str, VideoRoute] = {}
        self._tasks: set[asyncio.Task[None]] = set()

    @property
    def peers(self) -> Mapping[str, Peer]:
        return self._peers

    def route(self, route: str) -> VideoRoute:
        video_route = VideoRoute(self, route)
        self._routes[route] = video_route
        return video_route

    def encoders(self) -> int:
        return sum(len(r.encoders) for r in self._routes.values())

    def _close_msg(self, msg: TopicRtcOffer | TopicRtcCandidate, reason: str) -> None:
        self._send(TopicRtcClose(id=msg.id, route=msg.route, gen=msg.gen, reason=reason))

    async def offer(
        self, route: VideoRoute | None, settings: Mapping[str, Any] | None, msg: TopicRtcOffer
    ) -> None:
        """Answer a viewer's offer for ``route``, or end it with the reason."""
        if msg.id in self._peers:
            await self.close_peer(msg.id, "replaced", notify=False)
        if route is None or settings is None:
            self._close_msg(msg, "rtc_unavailable")
            return
        variant = msg.variant or "v1"
        if len(self._peers) >= self.max_peers:
            self._close_msg(msg, "rtc_limit")
            return
        encoder = route.encoders.get(variant)
        if encoder is None:
            if self.encoders() >= self.max_encoders:
                self._close_msg(msg, "rtc_limit")
                return
            encoder = SharedEncoder(self.codec_name, settings)
            route.encoders[variant] = encoder
        pc = self._peer_factory()
        track = PacketTrack()
        transceiver = pc.addTransceiver(track, direction="sendonly")
        require_h264(pc)
        peer = Peer(msg.id, msg.route, msg.gen, variant, pc, track, encoder)
        self._peers[msg.id] = peer
        encoder.peers[msg.id] = peer

        @pc.on("connectionstatechange")  # type: ignore[untyped-decorator]
        def _state() -> None:
            if peer.closing:
                return
            if pc.connectionState == "connected":
                # Start this viewer from an IDR without touching the others.
                peer.connected = True
                track.waiting_idr = True
                encoder.request_keyframe()
            elif pc.connectionState in ("failed", "closed"):
                self._spawn(self.close_peer(peer.token, "ice_failed"))

        try:
            await pc.setRemoteDescription(RTCSessionDescription(sdp=msg.sdp, type="offer"))
            answer = await pc.createAnswer()
            # Host candidates only: gathering completes inside setLocalDescription.
            await asyncio.wait_for(pc.setLocalDescription(answer), NEGOTIATE_S)
        except OperationError:  # no compatible H.264: the viewer can use its fallback
            await self.close_peer(peer.token, "rtc_unavailable")
            return
        except Exception as error:  # a malformed offer
            logger.warning("video offer rejected", route=msg.route, error=str(error))
            await self.close_peer(peer.token, "rtc_failed")
            return
        if peer.closing:
            return
        # PLI and FIR from this viewer force the shared encoder's next frame.
        transceiver.sender._send_keyframe = encoder.request_keyframe  # type: ignore[method-assign]
        self._send(
            TopicRtcAnswer(id=msg.id, route=msg.route, gen=msg.gen, sdp=pc.localDescription.sdp)
        )

    async def candidate(self, msg: TopicRtcCandidate) -> None:
        peer = self._peers.get(msg.id)
        if peer is None or peer.closing:
            return
        peer.candidates += 1
        if peer.candidates > MAX_TOPIC_RTC_CANDIDATES:
            await self.close_peer(msg.id, "rtc_candidate_limit")
            return
        try:
            if msg.candidate == "":
                await peer.pc.addIceCandidate(None)
                return
            line = (
                msg.candidate.split(":", 1)[1]
                if msg.candidate.startswith("candidate:")
                else msg.candidate
            )
            candidate = candidate_from_sdp(line)
            candidate.sdpMid = msg.mid or None
            if candidate.sdpMid is None:
                candidate.sdpMLineIndex = 0
            await peer.pc.addIceCandidate(candidate)
        except Exception as error:
            # A malformed candidate, or one the ICE agent cannot take (an mDNS
            # name on a host without a multicast route is an OSError). It ends
            # this peer and never the provider's connection.
            logger.warning("video candidate rejected", route=msg.route, error=str(error))
            await self.close_peer(msg.id, "rtc_failed")

    def _spawn(self, coroutine: Any) -> None:
        task = asyncio.ensure_future(coroutine)
        self._tasks.add(task)
        task.add_done_callback(self._tasks.discard)

    async def close_peer(self, token: str, reason: str, *, notify: bool = True) -> None:
        """End one peer, bounded by PEER_CLOSE_S; releases its encoder with
        the last peer."""
        peer = self._peers.pop(token, None)
        if peer is None or peer.closing:
            return
        peer.closing = True
        peer.connected = False
        encoder = peer.encoder
        encoder.peers.pop(token, None)
        if not encoder.peers:
            for route in self._routes.values():
                for variant, candidate in list(route.encoders.items()):
                    if candidate is encoder:
                        del route.encoders[variant]
        if notify:
            self._send(TopicRtcClose(id=token, route=peer.route, gen=peer.gen, reason=reason))
        logger.info(
            "video peer closed",
            route=peer.route,
            reason=reason,
            lived_s=round(time.monotonic() - peer.opened_at, 1),
            sent=peer.track.sent,
            peer_drops=peer.track.dropped,
            encodes=encoder.frames,
            keyframes=encoder.keyframes,
            stale_drops=encoder.dropped,
        )
        peer.track.stop()
        with suppress(asyncio.TimeoutError, Exception):
            await asyncio.wait_for(peer.pc.close(), PEER_CLOSE_S)

    async def close(self) -> None:
        for route in list(self._routes.values()):
            await route.close("provider_closed")
        for token in list(self._peers):
            await self.close_peer(token, "provider_closed", notify=False)
        for task in list(self._tasks):
            task.cancel()
        await asyncio.gather(*self._tasks, return_exceptions=True)

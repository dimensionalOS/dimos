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

"""The bridge's WebRTC peer toward the relay's Cloudflare SFU: a sendonly
H.264 transceiver per track channel, offered once through the relay. aiortc
encodes only what it is fed, so nothing is encoded until a viewer subscribes.

Imports aiortc at module level: the bridge imports this module lazily, only
once a relay advertises WebRTC video and aiortc is installed.
"""

from __future__ import annotations

import asyncio
from collections.abc import Sequence
import time

from aiortc import (
    RTCBundlePolicy,
    RTCConfiguration,
    RTCIceServer,
    RTCPeerConnection,
    RTCSessionDescription,
)

from dimos.msgs.sensor_msgs.Image import Image
from dimos.protocol.pubsub.impl.webrtc.providers.sdp import propagate_bundle_candidates
from dimos.protocol.pubsub.impl.webrtc.providers.spec import wait_connected
from dimos.protocol.pubsub.impl.webrtc.providers.video_track import (
    CameraVideoTrack,
    prefer_video_codec,
)
from dimos.utils.logging_config import setup_logger
from dimos.web.relay_bridge.protocol import IceServer, RtcTrack

logger = setup_logger()

# Non-trickle: the SFU gets one complete offer. aiortc gathers inside
# setLocalDescription, so this only bounds a TURN allocation that hangs.
_GATHER_TIMEOUT_S = 10.0
# Hardware-decoded by every browser; the hosted teleop's default too.
VIDEO_CODEC = "h264"
# Cloudflare collects a track after 30 s without media (TRACK_GC_MS in
# web/relay/cloudflare.ts), across every session pulling it.
TRACK_GC_S = 30.0


class RtcPublisher:
    """The robot's SFU peer: one video track per track channel."""

    def __init__(self, channels: Sequence[str]) -> None:
        self._channels = tuple(channels)
        self._pc: RTCPeerConnection | None = None
        self._tracks: dict[str, CameraVideoTrack] = {}
        self._lost: asyncio.Event | None = None
        # Channel -> monotonic time of the last frame fed (the feed thread's).
        self._last_fed: dict[str, float] = {}

    async def start(self, ice_servers: Sequence[IceServer]) -> tuple[str, list[RtcTrack]]:
        """Build the PeerConnection; returns (offer SDP, channel -> mid map)."""
        loop = asyncio.get_running_loop()
        pc = RTCPeerConnection(
            RTCConfiguration(
                iceServers=[
                    RTCIceServer(urls=list(s.urls), username=s.username, credential=s.credential)
                    for s in ice_servers
                ],
                # Mandatory with the Cloudflare SFU (dimos/teleop/hosted/README.md).
                bundlePolicy=RTCBundlePolicy.MAX_BUNDLE,
            )
        )
        self._pc = pc
        lost = asyncio.Event()
        self._lost = lost

        @pc.on("connectionstatechange")  # type: ignore[untyped-decorator]
        def _on_state() -> None:
            # aiortc's terminal states.
            if pc.connectionState in ("failed", "closed"):
                lost.set()

        transceivers = []
        for ch in self._channels:
            track = CameraVideoTrack(loop)
            self._tracks[ch] = track
            transceivers.append((ch, pc.addTransceiver(track, direction="sendonly")))
        prefer_video_codec(pc, VIDEO_CODEC)
        await asyncio.wait_for(pc.setLocalDescription(await pc.createOffer()), _GATHER_TIMEOUT_S)
        # mids exist only once the local description is set.
        tracks = [RtcTrack(ch=ch, mid=str(transceiver.mid)) for ch, transceiver in transceivers]
        return pc.localDescription.sdp, tracks

    async def accept(self, answer_sdp: str) -> None:
        """Returns once ICE and DTLS are up (RuntimeError otherwise)."""
        assert self._pc is not None
        await self._pc.setRemoteDescription(
            RTCSessionDescription(sdp=propagate_bundle_candidates(answer_sdp), type="answer")
        )
        await wait_connected(self._pc)
        for track in self._tracks.values():
            track.arm()

    async def wait_lost(self) -> str:
        """Blocks while the connection lives; returns the terminal state."""
        assert self._pc is not None and self._lost is not None
        await self._lost.wait()
        return str(self._pc.connectionState)

    def feed(self, ch: str, image: Image) -> bool:
        """Thread-safe: called from the input callback thread. True when this
        frame ends a media gap of TRACK_GC_S or more: the SFU collected the
        track meanwhile, and the relay must declare and pull it again."""
        now = time.monotonic()
        last = self._last_fed.get(ch)
        self._last_fed[ch] = now
        self._tracks[ch].set_latest(image)
        return last is not None and now - last >= TRACK_GC_S

    async def close(self) -> None:
        if self._pc is not None:
            await self._pc.close()
            self._pc = None

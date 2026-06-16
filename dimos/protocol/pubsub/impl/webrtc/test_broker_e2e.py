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

"""End-to-end: simulated operator -> live broker -> Cloudflare -> CloudflareTransport.

The operator side is aiortc standing in for the browser, speaking the same
protocol as the teleop web client (join -> wait connected -> bridge-datachannel
-> negotiated cmd_unreliable channel). The robot side is the exact transport
object the teleop-hosted-go2-transport blueprint binds to cmd_vel.

Needs live-broker credentials, so it only runs when all three are set:

    TELEOP_API_KEY        dtk_live_... (dashboard -> New Key)
    TELEOP_OPERATOR_TOKEN Cognito ID token of the key's owner

TELEOP_ROBOT_ID is optional (the broker derives identity from the key).
"""

from __future__ import annotations

import asyncio
import json
import os
import time
import urllib.request

import pytest

from dimos.protocol.pubsub.impl.webrtc.providers.spec import WEBRTC_AVAILABLE

BROKER = os.environ.get("TELEOP_BROKER_URL", "https://teleop.dimensionalos.com")
CREDS_PRESENT = all(os.environ.get(k) for k in ("TELEOP_API_KEY", "TELEOP_OPERATOR_TOKEN"))

skip_unless_broker = pytest.mark.skipif(
    not (WEBRTC_AVAILABLE and CREDS_PRESENT),
    reason="needs aiortc + TELEOP_API_KEY/TELEOP_OPERATOR_TOKEN",
)


def _api(method: str, path: str, body: dict | None = None) -> dict:
    req = urllib.request.Request(
        f"{BROKER}/api/v1{path}",
        method=method,
        data=json.dumps(body).encode() if body else None,
        headers={
            "Authorization": f"Bearer {os.environ['TELEOP_OPERATOR_TOKEN']}",
            "Content-Type": "application/json",
        },
    )
    with urllib.request.urlopen(req, timeout=30) as r:
        return json.loads(r.read() or b"{}")


@skip_unless_broker
@pytest.mark.tool
@pytest.mark.timeout(120)
def test_operator_to_transport_e2e() -> None:
    from aiortc import (
        RTCConfiguration,
        RTCIceServer,
        RTCPeerConnection,
        RTCSessionDescription,
    )
    import numpy as np

    from dimos.core.transport import CloudflareTransport, CloudflareVideoTransport
    from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
    from dimos.msgs.sensor_msgs.Image import Image, ImageFormat

    received: list[TwistStamped] = []
    transport = CloudflareTransport("cmd_unreliable", TwistStamped)
    transport.subscribe(received.append)
    # Robot → operator telemetry + video on the same provider/session.
    back_transport = CloudflareTransport("state_reliable_back", TwistStamped)
    video_transport = CloudflareVideoTransport()
    time.sleep(3)  # session registration

    # The robot's own session id, straight from the shared provider — never
    # guess from the session list (stale sessions from aborted runs linger).
    assert transport._pubsub is not None
    session_id = transport._pubsub.provider.session_id
    assert session_id, "provider did not register a session"

    async def operator() -> int:
        pc = RTCPeerConnection(
            RTCConfiguration(iceServers=[RTCIceServer(urls=["stun:stun.cloudflare.com:3478"])])
        )
        pc.createDataChannel(
            "_sctp_init", negotiated=True, id=0
        )  # placeholder; pinned id so CF-assigned ids can never collide
        await pc.setLocalDescription(await pc.createOffer())
        while pc.iceGatheringState != "complete":
            await asyncio.sleep(0.05)

        video_frames: list[object] = []
        consume_tasks: list[asyncio.Task[None]] = []

        # Feed camera frames from the start: CF infers the pulled track's kind
        # from flowing RTP, so (as with a real camera stream) frames must be
        # arriving before the operator bridges, or the pull offer comes back
        # without a usable video m-line.
        frame = Image(data=np.full((120, 160, 3), 128, dtype=np.uint8), format=ImageFormat.BGR)
        feeding = True

        async def _feed() -> None:
            while feeding:
                video_transport.broadcast(None, frame)
                await asyncio.sleep(0.05)

        feed_task = asyncio.ensure_future(_feed())

        @pc.on("track")
        def _on_track(track: object) -> None:
            async def _consume() -> None:
                while len(video_frames) < 3:
                    video_frames.append(await track.recv())  # type: ignore[attr-defined]

            consume_tasks.append(asyncio.ensure_future(_consume()))

        join = _api(
            "POST",
            f"/sessions/{session_id}/join",
            {"role": "operator", "sdp_offer": pc.localDescription.sdp},
        )
        await pc.setRemoteDescription(RTCSessionDescription(sdp=join["sdp_answer"], type="answer"))
        for _ in range(100):
            if pc.connectionState == "connected":
                break
            await asyncio.sleep(0.1)
        assert pc.connectionState == "connected"

        bridge = _api("POST", f"/sessions/{session_id}/bridge-datachannel")
        ch = pc.createDataChannel(
            "cmd_unreliable",
            negotiated=True,
            id=bridge["cmd_channel_id"],
            ordered=False,
            maxRetransmits=0,
        )
        back_bytes: list[bytes] = []
        back_ch = pc.createDataChannel(
            "state_reliable_back",
            negotiated=True,
            id=bridge["state_back_channel_id"],
            ordered=True,
        )

        @back_ch.on("message")
        def _on_back(payload: object) -> None:
            back_bytes.append(payload if isinstance(payload, bytes) else str(payload).encode())

        for _ in range(200):
            if ch.readyState == "open":
                break
            await asyncio.sleep(0.1)
        assert ch.readyState == "open"
        await asyncio.sleep(3)  # robot heartbeat (1 Hz) delivers subscriber ids

        # Video: complete the broker's pull renegotiation (same as the web
        # client), then feed synthetic frames through the video transport.
        if bridge.get("video_offer"):
            await pc.setRemoteDescription(
                RTCSessionDescription(sdp=bridge["video_offer"], type="offer")
            )
            answer = await pc.createAnswer()
            await pc.setLocalDescription(answer)
            _api(
                "POST",
                f"/sessions/{session_id}/renegotiate-answer",
                {"sdp_answer": pc.localDescription.sdp},
            )
        assert bridge.get("video_offer"), f"no video_offer (status: {bridge.get('video_status')})"
        # Wire-level assertion: CF must forward the robot's video RTP to the
        # operator. Decoded frames are best-effort here — aiortc's receiver is
        # lazy about keyframe requests (PLI) when joining mid-stream, while the
        # real consumer (browser) requests keyframes immediately and renders.
        rtp_packets = 0
        for _ in range(150):  # up to 15s
            if len(video_frames) >= 3:
                break
            stats = await pc.getStats()
            rtp_packets = sum(
                getattr(v, "packetsReceived", 0) for k, v in stats.items() if "inbound-rtp" in k
            )
            if rtp_packets > 100 and len(video_frames) == 0:
                # forwarding proven; give decode a short grace then move on
                await asyncio.sleep(3)
                break
            await asyncio.sleep(0.1)
        assert len(consume_tasks) > 0, "ontrack never fired for the pulled video"
        assert rtp_packets > 50 or len(video_frames) >= 3, (
            f"no video RTP reached the operator (packets={rtp_packets}, "
            f"frames={len(video_frames)}, status={bridge.get('video_status')})"
        )
        if video_frames:
            assert video_frames[0].width == 160, video_frames[0]

        # Robot → operator: telemetry through the broker-bridged back channel.
        for i in range(10):
            back_transport.broadcast(None, TwistStamped(linear=[0.0, 0.0, 1.0 + i]))
            await asyncio.sleep(0.05)
        await asyncio.sleep(2)
        assert back_bytes, "no robot->operator telemetry arrived"
        back_msg = TwistStamped.lcm_decode(back_bytes[-1])
        assert back_msg.linear.z >= 1.0, back_msg.linear

        sent = 0
        for i in range(40):
            msg = TwistStamped(linear=[0.5, 0.0, 0.0], angular=[0.0, 0.0, i * 0.01])
            ch.send(msg.lcm_encode())
            sent += 1
            await asyncio.sleep(0.05)
        await asyncio.sleep(2)

        _api("POST", f"/sessions/{session_id}/leave", {"role": "operator"})
        feeding = False
        feed_task.cancel()
        for t in consume_tasks:
            t.cancel()
        await pc.close()
        return sent

    try:
        sent = asyncio.run(operator())
        # Unreliable channel: tolerate stragglers, but the path must work.
        assert len(received) >= sent * 0.8, f"sent={sent} received={len(received)}"
        sample = received[-1]
        assert isinstance(sample, TwistStamped)
        assert abs(sample.linear.x - 0.5) < 1e-9
    finally:
        transport.stop()
        back_transport.stop()
        video_transport.stop()
        # transport.stop() deliberately leaves the process-scoped provider
        # running; stop it here so the test doesn't leak its loop thread.
        if transport._pubsub is not None:
            transport._pubsub.provider.stop()

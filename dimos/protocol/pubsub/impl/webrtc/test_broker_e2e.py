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
    TELEOP_ROBOT_ID       namespaced id from key creation (owner_email:robot)
    TELEOP_OPERATOR_TOKEN Cognito ID token of the key's owner
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
CREDS_PRESENT = all(
    os.environ.get(k) for k in ("TELEOP_API_KEY", "TELEOP_ROBOT_ID", "TELEOP_OPERATOR_TOKEN")
)

skip_unless_broker = pytest.mark.skipif(
    not (WEBRTC_AVAILABLE and CREDS_PRESENT),
    reason="needs aiortc + TELEOP_API_KEY/TELEOP_ROBOT_ID/TELEOP_OPERATOR_TOKEN",
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

    from dimos.core.transport import CloudflareTransport
    from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped

    received: list[TwistStamped] = []
    transport = CloudflareTransport("cmd_unreliable", TwistStamped)
    transport.subscribe(received.append)
    time.sleep(3)  # session registration

    sessions = _api("GET", "/sessions")
    sess = next(s for s in sessions if s["robot_id"] == os.environ["TELEOP_ROBOT_ID"])
    session_id = sess["session_id"]

    async def operator() -> int:
        pc = RTCPeerConnection(
            RTCConfiguration(iceServers=[RTCIceServer(urls=["stun:stun.cloudflare.com:3478"])])
        )
        pc.createDataChannel("_sctp_init")  # placeholder, same as the web client
        await pc.setLocalDescription(await pc.createOffer())
        while pc.iceGatheringState != "complete":
            await asyncio.sleep(0.05)

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
        for _ in range(100):
            if ch.readyState == "open":
                break
            await asyncio.sleep(0.1)
        assert ch.readyState == "open"
        await asyncio.sleep(3)  # robot heartbeat (1 Hz) delivers subscriber ids

        sent = 0
        for i in range(40):
            msg = TwistStamped(linear=[0.5, 0.0, 0.0], angular=[0.0, 0.0, i * 0.01])
            ch.send(msg.lcm_encode())
            sent += 1
            await asyncio.sleep(0.05)
        await asyncio.sleep(2)

        _api("POST", f"/sessions/{session_id}/leave", {"role": "operator"})
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
        # transport.stop() deliberately leaves the process-scoped provider
        # running; stop it here so the test doesn't leak its loop thread.
        if transport._pubsub is not None:
            transport._pubsub.provider.stop()

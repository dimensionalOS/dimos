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

"""End-to-end: a blueprint stream carried by WebRTCTransport over live CF.

Deploys two modules through the real ModuleCoordinator with the stream
transport swapped to WebRTC — exercising transport pickling into the worker
process, per-process provider sharing, typed LCM encode/decode, and a real
Cloudflare Realtime round trip. Skipped without CF credentials.

Run with: ``pytest dimos/protocol/pubsub/impl/webrtc/tool_blueprint_e2e.py``
"""

from __future__ import annotations

import os
import time
from types import MappingProxyType

import pytest

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.core.transport import WebRTCTransport
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.protocol.pubsub.impl.webrtc.providers.cloudflare import CloudflareConfig
from dimos.protocol.pubsub.impl.webrtc.providers.spec import WEBRTC_AVAILABLE

CF_CREDS_PRESENT = bool(os.environ.get("CF_TELEOP_APP_ID")) and bool(
    os.environ.get("CF_TELEOP_APP_SECRET")
)

skip_unless_cf = pytest.mark.skipif(
    not (WEBRTC_AVAILABLE and CF_CREDS_PRESENT),
    reason="Requires aiortc + CF_TELEOP_APP_ID/CF_TELEOP_APP_SECRET",
)


class TwistSource(Module):
    cmd_webrtc: Out[TwistStamped]

    @rpc
    def send(self, x: float) -> None:
        self.cmd_webrtc.publish(TwistStamped(linear=[x, 0, 0], angular=[0, 0, 0]))


class TwistSink(Module):
    cmd_webrtc: In[TwistStamped]

    @rpc
    def start(self) -> None:
        super().start()
        self._received: list[float] = []
        self.cmd_webrtc.subscribe(lambda msg: self._received.append(msg.linear.x))

    @rpc
    def received(self) -> list[float]:
        return list(self._received)


@skip_unless_cf
@pytest.mark.timeout(120)
def test_blueprint_stream_over_cloudflare() -> None:
    from dimos.core.coordination.module_coordinator import ModuleCoordinator

    # The direct-CF provider is a loopback pair (own pub + sub session), so
    # both modules must share one worker → one provider. Cross-peer delivery
    # is the broker's job.
    blueprint = autoconnect(TwistSource.blueprint(), TwistSink.blueprint()).transports(
        {
            ("cmd_webrtc", TwistStamped): WebRTCTransport.spec(
                "cmd_webrtc",
                TwistStamped,
                config=CloudflareConfig(
                    app_id=os.environ["CF_TELEOP_APP_ID"],
                    app_secret=os.environ["CF_TELEOP_APP_SECRET"],
                ),
            )
        }
    )

    coordinator = ModuleCoordinator.build(
        blueprint, MappingProxyType({"g": {"viewer": "none", "n_workers": 1}}).copy()
    )
    try:
        coordinator.start_all_modules()
        source = coordinator.get_instance(TwistSource)
        sink = coordinator.get_instance(TwistSink)

        deadline = time.time() + 30.0
        received: list[float] = []
        sent = 0.0
        while time.time() < deadline:
            sent += 1.0
            source.send(sent)
            time.sleep(0.5)
            received = sink.received()
            if received:
                break

        assert received, "No TwistStamped arrived over the CF DataChannel"
        assert received[-1] in [float(i) for i in range(1, int(sent) + 1)]
    finally:
        coordinator.stop()

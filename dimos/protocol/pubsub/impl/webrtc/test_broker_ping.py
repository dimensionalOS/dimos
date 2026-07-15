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

"""Unit tests for BrokerProvider's inline clock-sync ping responder.

No network — a mocked ``state_reliable_back`` channel is injected and we
assert on what ``_maybe_answer_ping`` sends. The ping protocol itself
(ping → pong with echoed client_ts + fresh robot_ts) is what the web
client's RTT/offset estimator depends on.
"""

from __future__ import annotations

import json
import time
from typing import Any
from unittest.mock import MagicMock

import pytest

from dimos.protocol.pubsub.impl.webrtc.providers.broker import BrokerConfig, BrokerProvider

# broker.py imports aiortc lazily, but BrokerProvider.__init__ pulls in the
# video track (aiortc/av) — so constructing one needs the extras installed.


@pytest.fixture
def provider() -> BrokerProvider:
    """BrokerProvider with a mocked open state_reliable_back channel."""
    p = BrokerProvider(BrokerConfig(api_key="dtk_test_key"))
    back = MagicMock()
    back.readyState = "open"
    p._dcs["state_reliable_back"] = back
    return p


def _sent_payload(provider: BrokerProvider) -> dict[str, Any]:
    ch: Any = provider._dcs["state_reliable_back"]
    ch.send.assert_called_once()
    return json.loads(ch.send.call_args[0][0])  # type: ignore[no-any-return]


def test_ping_echoes_pong_with_robot_ts(provider: BrokerProvider) -> None:
    before = time.time()
    provider._maybe_answer_ping(json.dumps({"type": "ping", "client_ts": 100.5}).encode())
    after = time.time()

    sent = _sent_payload(provider)
    assert sent["type"] == "pong"
    assert sent["client_ts"] == 100.5
    assert before <= sent["robot_ts"] <= after


def test_non_json_binary_ignored(provider: BrokerProvider) -> None:
    """LCM binary on the channel (future telemetry) must not be parsed."""
    provider._maybe_answer_ping(b"\x00\x01\x02\x03lcm-ish")
    provider._dcs["state_reliable_back"].send.assert_not_called()  # type: ignore[attr-defined]


def test_malformed_json_dropped(provider: BrokerProvider) -> None:
    provider._maybe_answer_ping(b"{not json")
    provider._dcs["state_reliable_back"].send.assert_not_called()  # type: ignore[attr-defined]


def test_other_json_types_ignored(provider: BrokerProvider) -> None:
    provider._maybe_answer_ping(b'{"type":"video_stats","fps":30}')
    provider._dcs["state_reliable_back"].send.assert_not_called()  # type: ignore[attr-defined]


def test_pong_dropped_when_back_channel_closed(provider: BrokerProvider) -> None:
    provider._dcs["state_reliable_back"].readyState = "closed"  # type: ignore[misc]
    provider._maybe_answer_ping(b'{"type":"ping","client_ts":1.0}')
    provider._dcs["state_reliable_back"].send.assert_not_called()  # type: ignore[attr-defined]


def test_pong_dropped_when_back_channel_absent(provider: BrokerProvider) -> None:
    del provider._dcs["state_reliable_back"]
    # No channel at all — must not raise.
    provider._maybe_answer_ping(b'{"type":"ping","client_ts":1.0}')

#!/usr/bin/env python3
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

# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Unit tests for the Phase 1.5 clock-sync ping/pong protocol.

Validates ``HostedTeleopModule._on_state_message`` shape without standing up
a WebRTC stack — injects a mocked state channel and inspects what it sends.
Complements the dev_broker E2E test by pinning the JSON protocol itself.
"""

from __future__ import annotations

import json
import time
from unittest.mock import MagicMock

import pytest

from dimos.teleop.quest_hosted.hosted_teleop_module import HostedTeleopModule


@pytest.fixture
def module():
    """Bare ``HostedTeleopModule`` with a mocked open state channel injected."""
    m = HostedTeleopModule.__new__(HostedTeleopModule)
    m._state_channel = MagicMock()
    m._state_channel.readyState = "open"
    return m


def _sent_payload(module) -> dict:
    """Decode the JSON the mock channel was asked to send."""
    module._state_channel.send.assert_called_once()
    return json.loads(module._state_channel.send.call_args[0][0])


def test_ping_echoes_pong_with_robot_ts(module):
    """A well-formed ping → pong with same client_ts + a fresh robot_ts."""
    before = time.time()
    module._on_state_message(json.dumps({"type": "ping", "client_ts": 100.5}).encode("utf-8"))
    after = time.time()

    sent = _sent_payload(module)
    assert sent["type"] == "pong"
    assert sent["client_ts"] == 100.5
    assert before <= sent["robot_ts"] <= after


def test_ping_accepts_str_data(module):
    """aiortc may deliver str OR bytes on a DataChannel; both must work."""
    module._on_state_message('{"type":"ping","client_ts":42.0}')
    assert _sent_payload(module)["client_ts"] == 42.0


def test_malformed_json_dropped(module):
    """Garbage payload — pong not sent, no exception."""
    module._on_state_message(b"not json at all")
    module._state_channel.send.assert_not_called()


def test_unknown_type_dropped(module):
    """Future control-plane messages this version doesn't recognise are no-ops."""
    module._on_state_message(b'{"type":"mode_switch","mode":"arm"}')
    module._state_channel.send.assert_not_called()


def test_ping_missing_client_ts_dropped(module):
    """Don't echo nonsense pings — keeps the offset estimator clean."""
    module._on_state_message(b'{"type":"ping"}')
    module._state_channel.send.assert_not_called()


def test_pong_swallowed_when_channel_closed(module):
    """If the channel closed between recv and the send attempt, swallow it."""
    module._state_channel.readyState = "closed"
    module._on_state_message(b'{"type":"ping","client_ts":1.0}')
    module._state_channel.send.assert_not_called()


def test_non_utf8_bytes_dropped(module):
    """Random bytes that aren't valid UTF-8 — drop, don't crash."""
    module._on_state_message(b"\xff\xfe\xfd")
    module._state_channel.send.assert_not_called()

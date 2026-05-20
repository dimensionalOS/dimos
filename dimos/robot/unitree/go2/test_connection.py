# Copyright 2025-2026 Dimensional Inc.
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

"""Targeted test for go2.connection.make_connection's aes_128_key forwarding.

Pins the wiring added in PR #2117 so renaming the kwarg in either
UnitreeWebRTCConnection or make_connection without updating the other
fails loudly. The leaf (UnitreeWebRTCConnection.__init__) is exercised
in dimos/robot/unitree/test_connection.py — this file only covers the
go2-local routing.
"""

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from dimos.robot.unitree.go2 import connection as go2_conn


@pytest.fixture
def stub_webrtc(monkeypatch: pytest.MonkeyPatch) -> MagicMock:
    """Replace UnitreeWebRTCConnection in go2.connection with a MagicMock so
    make_connection's webrtc branch is exercised without dialing out."""
    stub = MagicMock(name="UnitreeWebRTCConnection")
    monkeypatch.setattr(go2_conn, "UnitreeWebRTCConnection", stub)
    return stub


def test_make_connection_webrtc_forwards_aes_128_key(stub_webrtc: MagicMock) -> None:
    """Webrtc branch must forward aes_128_key as a kwarg to UnitreeWebRTCConnection.

    Guards the Go2 half of the PR #2117 fix: without this forwarding, Go2
    robots on firmware >=1.1.15 fail the WebRTC handshake even when a key
    is provided in config.
    """
    cfg = SimpleNamespace(unitree_connection_type="webrtc")
    go2_conn.make_connection("192.168.123.161", cfg, aes_128_key="cafe" * 8)
    stub_webrtc.assert_called_once_with("192.168.123.161", aes_128_key="cafe" * 8)

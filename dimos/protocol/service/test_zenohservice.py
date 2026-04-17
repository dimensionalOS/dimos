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

"""Tests for ZenohService — singleton session management."""

from __future__ import annotations

import pytest

from dimos.protocol.service.zenohservice import ZenohConfig, ZenohService, _sessions


@pytest.fixture(autouse=True)
def _clear_sessions():
    """Clear the global session cache before each test."""
    yield
    # Close and remove all sessions after each test
    for session in _sessions.values():
        session.close()
    _sessions.clear()


class TestZenohConfig:
    def test_default_mode_is_peer(self) -> None:
        config = ZenohConfig()
        assert config.mode == "peer"

    def test_session_key_is_stable(self) -> None:
        config = ZenohConfig()
        assert config.session_key == config.session_key

    def test_different_modes_produce_different_keys(self) -> None:
        peer = ZenohConfig(mode="peer")
        client = ZenohConfig(mode="client")
        assert peer.session_key != client.session_key


class TestZenohService:
    def test_start_creates_session(self) -> None:
        svc = ZenohService()
        svc.start()
        assert svc.session is not None

    def test_two_services_share_session(self) -> None:
        svc1 = ZenohService()
        svc2 = ZenohService()
        svc1.start()
        svc2.start()
        assert svc1.session is svc2.session

    def test_stop_does_not_close_shared_session(self) -> None:
        svc1 = ZenohService()
        svc2 = ZenohService()
        svc1.start()
        svc2.start()
        svc1.stop()
        # svc2's session should still be valid
        assert svc2.session is not None

    def test_session_before_start_raises(self) -> None:
        svc = ZenohService()
        with pytest.raises(RuntimeError, match="not initialized"):
            _ = svc.session

    def test_start_is_idempotent(self) -> None:
        svc = ZenohService()
        svc.start()
        session1 = svc.session
        svc.start()
        session2 = svc.session
        assert session1 is session2

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

from __future__ import annotations

import os
from typing import Any

import pytest
import zenoh

from dimos.protocol.service import zenohservice
from dimos.protocol.service.zenohservice import ZenohConfig, ZenohService, ZenohSessionPool


@pytest.fixture()
def session_pool():
    """Provide a fresh, isolated session pool and close it after the test."""
    pool = ZenohSessionPool()
    yield pool
    pool.close_all()


class _RecordingLogger:
    """Stands in for the module logger to capture structured warnings."""

    def __init__(self) -> None:
        self.warnings: list[tuple[str, dict[str, Any]]] = []

    def warning(self, event: str, **fields: Any) -> None:
        self.warnings.append((event, fields))

    def info(self, event: str, **fields: Any) -> None:
        pass

    def debug(self, event: str, **fields: Any) -> None:
        pass


@pytest.fixture()
def recorded_logs(monkeypatch):
    recorder = _RecordingLogger()
    monkeypatch.setattr(zenohservice, "logger", recorder)
    return recorder


def _acquire(monkeypatch, config: ZenohConfig) -> None:
    monkeypatch.setattr(zenohservice.zenoh, "open", lambda zconfig: object())
    ZenohSessionPool().acquire(config)


def test_a_client_dialing_several_endpoints_warns(
    zenoh_defaults, recorded_logs, monkeypatch
) -> None:
    """Only the first endpoint that connects carries traffic."""
    endpoints = ["tcp/192.0.2.10:7447", "tcp/192.0.2.11:7447"]
    _acquire(monkeypatch, ZenohConfig(mode="client", connect=endpoints))

    event, fields = recorded_logs.warnings[0]
    assert "single link" in event
    assert fields["connect"] == endpoints


def test_a_peer_dialing_several_endpoints_does_not_warn(
    zenoh_defaults, recorded_logs, monkeypatch
) -> None:
    """A peer links to all of them, so there is nothing to warn about."""
    endpoints = ["tcp/192.0.2.10:7447", "tcp/192.0.2.11:7447"]
    _acquire(monkeypatch, ZenohConfig(mode="peer", connect=endpoints))
    assert recorded_logs.warnings == []


def test_a_client_with_one_endpoint_does_not_warn(
    zenoh_defaults, recorded_logs, monkeypatch
) -> None:
    _acquire(monkeypatch, ZenohConfig(mode="client", connect=["tcp/192.0.2.10:7447"]))
    assert recorded_logs.warnings == []


class _UnclosableSession:
    def close(self) -> None:
        raise zenoh.ZError("close timed out")


def test_close_all_empties_the_pool_even_when_a_session_will_not_close(
    zenoh_defaults, monkeypatch, recorded_logs
) -> None:
    """A session holding links to unreachable peers must not pin the pool."""
    opens = []
    monkeypatch.setattr(
        zenohservice.zenoh, "open", lambda zconfig: opens.append(zconfig) or _UnclosableSession()
    )
    pool = ZenohSessionPool()
    pool.acquire(ZenohConfig())

    pool.close_all()

    assert [event for event, _ in recorded_logs.warnings] == ["Zenoh session close failed"]
    pool.acquire(ZenohConfig())
    assert len(opens) == 2


def test_different_modes_produce_different_keys() -> None:
    peer = ZenohConfig(mode="peer")
    client = ZenohConfig(mode="client")
    assert peer.session_key != client.session_key


def test_start_creates_session(session_pool) -> None:
    svc = ZenohService(session_pool=session_pool)
    svc.start()
    assert svc.session is not None


def test_two_services_share_session(session_pool) -> None:
    svc1 = ZenohService(session_pool=session_pool)
    svc2 = ZenohService(session_pool=session_pool)
    svc1.start()
    svc2.start()
    assert svc1.session is svc2.session


def test_acquire_after_fork_raises(session_pool, mocker) -> None:
    mocker.patch("dimos.protocol.service.zenohservice.zenoh.open", return_value=mocker.MagicMock())
    config = ZenohConfig()
    session_pool.acquire(config)

    mocker.patch("os.getpid", return_value=os.getpid() + 1)
    with pytest.raises(RuntimeError, match="does not survive fork"):
        session_pool.acquire(config)


def test_stop_does_not_close_shared_session(session_pool) -> None:
    svc1 = ZenohService(session_pool=session_pool)
    svc2 = ZenohService(session_pool=session_pool)
    svc1.start()
    svc2.start()
    svc1.stop()
    # svc2's session should still be valid
    assert svc2.session is not None


def test_session_before_start_raises(session_pool) -> None:
    svc = ZenohService(session_pool=session_pool)
    with pytest.raises(RuntimeError, match="not initialized"):
        svc.session  # noqa: B018


def test_start_is_idempotent(session_pool) -> None:
    svc = ZenohService(session_pool=session_pool)
    svc.start()
    session1 = svc.session
    svc.start()
    session2 = svc.session
    assert session1 is session2

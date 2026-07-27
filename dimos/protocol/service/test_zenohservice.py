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

import pytest

from dimos.protocol.service.zenohservice import ZenohConfig, ZenohService, ZenohSessionPool


@pytest.fixture()
def session_pool():
    """Provide a fresh, isolated session pool and close it after the test."""
    pool = ZenohSessionPool()
    yield pool
    pool.close_all()


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

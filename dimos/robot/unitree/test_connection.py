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

"""Unit tests for UnitreeWebRTCConnection.

Pure-Python — no hardware, no network. Covers connect() error propagation,
aes_128_key forwarding, and the UNITREE_AES_128_KEY env var via GlobalConfig.
"""

from typing import Any
from unittest.mock import AsyncMock, MagicMock

import pytest

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.global_config import GlobalConfig
from dimos.robot.unitree import connection as conn_mod
from dimos.robot.unitree.connection import UnitreeWebRTCConnection


def _stub_driver(connect_exc: Exception | None = None) -> MagicMock:
    """A LegionConnection instance double covering everything connect() touches."""
    driver = MagicMock(name="LegionConnection-instance")
    driver.connect = AsyncMock(side_effect=connect_exc)
    driver.datachannel.disableTrafficSaving = AsyncMock()
    driver.datachannel.set_decoder = MagicMock()
    driver.datachannel.pub_sub.publish_request_new = AsyncMock()
    return driver


def test_connect_failure_propagates_to_caller(monkeypatch: pytest.MonkeyPatch) -> None:
    """A driver connect failure must raise from the constructor, not hang."""
    driver = _stub_driver(connect_exc=RuntimeError("aes_128_key required (data2=3)"))
    monkeypatch.setattr(conn_mod, "LegionConnection", MagicMock(return_value=driver))

    with pytest.raises(RuntimeError, match="aes_128_key required"):
        UnitreeWebRTCConnection(ip="10.0.0.99")


@pytest.fixture
def built_connection(monkeypatch: pytest.MonkeyPatch) -> Any:
    """A live UnitreeWebRTCConnection over a stubbed driver, torn down (loop
    stopped, thread joined) unconditionally so a failed assert can't leak it."""
    driver = _stub_driver()
    monkeypatch.setattr(conn_mod, "LegionConnection", MagicMock(return_value=driver))

    conn = UnitreeWebRTCConnection(ip="10.0.0.99")
    try:
        yield conn, driver
    finally:
        conn.loop.call_soon_threadsafe(conn.loop.stop)
        conn.thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)


def test_connect_success_completes_setup(built_connection: Any) -> None:
    """Happy path: constructor returns after the setup sequence ran."""
    _conn, driver = built_connection

    driver.connect.assert_awaited_once()
    driver.datachannel.pub_sub.publish_request_new.assert_awaited_once()


@pytest.fixture
def stub_legion(monkeypatch: pytest.MonkeyPatch) -> MagicMock:
    """Replace LegionConnection with a mock and no-op connect() so __init__
    stays inside the aes_128_key resolution without dialing out."""
    monkeypatch.setattr(UnitreeWebRTCConnection, "connect", lambda self: None)
    legion = MagicMock(name="LegionConnection")
    monkeypatch.setattr(conn_mod, "LegionConnection", legion)
    return legion


def _aes_kwarg(legion: MagicMock) -> Any:
    """The aes_128_key passed to LegionConnection, or None if absent."""
    return legion.call_args.kwargs.get("aes_128_key")


def test_no_key_forwards_falsy(stub_legion: MagicMock) -> None:
    """No key → a falsy value reaches the driver, which treats it as no key."""
    UnitreeWebRTCConnection(ip="192.168.123.161")
    assert not _aes_kwarg(stub_legion)


def test_aes_key_forwarded_when_provided(stub_legion: MagicMock) -> None:
    """A provided key is forwarded verbatim to the driver."""
    UnitreeWebRTCConnection(ip="192.168.123.161", aes_128_key="aa" * 16)
    assert _aes_kwarg(stub_legion) == "aa" * 16


def test_empty_string_key_forwarded_as_falsy(stub_legion: MagicMock) -> None:
    """Empty-string key stays falsy → the driver treats it as no key."""
    UnitreeWebRTCConnection(ip="192.168.123.161", aes_128_key="")
    assert not _aes_kwarg(stub_legion)


def test_global_config_reads_unitree_aes_128_key_env(monkeypatch: pytest.MonkeyPatch) -> None:
    """The key enters via GlobalConfig, read from the UNITREE_AES_128_KEY env var."""
    monkeypatch.setenv("UNITREE_AES_128_KEY", "ee" * 16)
    assert GlobalConfig().unitree_aes_128_key == "ee" * 16

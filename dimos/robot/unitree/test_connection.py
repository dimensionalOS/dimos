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

Pure-Python — no hardware, no network. Covers connect() error propagation and
the aes_128_key kwarg / UNITREE_AES_128_KEY env-var precedence.
"""

from typing import Any
from unittest.mock import AsyncMock, MagicMock

import pytest

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


def test_connect_success_completes_setup(monkeypatch: pytest.MonkeyPatch) -> None:
    """Happy path: constructor returns after the setup sequence ran."""
    driver = _stub_driver()
    monkeypatch.setattr(conn_mod, "LegionConnection", MagicMock(return_value=driver))

    conn = UnitreeWebRTCConnection(ip="10.0.0.99")

    driver.connect.assert_awaited_once()
    driver.datachannel.pub_sub.publish_request_new.assert_awaited_once()

    conn.loop.call_soon_threadsafe(conn.loop.stop)
    conn.thread.join(timeout=5)


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


def test_aes_key_omitted_when_neither_kwarg_nor_env(
    stub_legion: MagicMock, monkeypatch: pytest.MonkeyPatch
) -> None:
    """No kwarg, no env var → key not forwarded (byte-identical to pre-key call)."""
    monkeypatch.delenv("UNITREE_AES_128_KEY", raising=False)
    UnitreeWebRTCConnection(ip="192.168.123.161")
    assert "aes_128_key" not in stub_legion.call_args.kwargs


def test_aes_key_from_explicit_kwarg(
    stub_legion: MagicMock, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Explicit kwarg is forwarded verbatim."""
    monkeypatch.delenv("UNITREE_AES_128_KEY", raising=False)
    UnitreeWebRTCConnection(ip="192.168.123.161", aes_128_key="aa" * 16)
    assert _aes_kwarg(stub_legion) == "aa" * 16


def test_aes_key_from_env_when_kwarg_none(
    stub_legion: MagicMock, monkeypatch: pytest.MonkeyPatch
) -> None:
    """No kwarg → falls back to UNITREE_AES_128_KEY."""
    monkeypatch.setenv("UNITREE_AES_128_KEY", "bb" * 16)
    UnitreeWebRTCConnection(ip="192.168.123.161")
    assert _aes_kwarg(stub_legion) == "bb" * 16


def test_explicit_kwarg_beats_env(stub_legion: MagicMock, monkeypatch: pytest.MonkeyPatch) -> None:
    """Explicit kwarg wins over the env var."""
    monkeypatch.setenv("UNITREE_AES_128_KEY", "from-env")
    UnitreeWebRTCConnection(ip="192.168.123.161", aes_128_key="from-kwarg")
    assert _aes_kwarg(stub_legion) == "from-kwarg"


def test_empty_string_kwarg_falls_back_to_env_when_unset(
    stub_legion: MagicMock, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Empty-string kwarg + unset env → nothing forwarded (falsy guard)."""
    monkeypatch.delenv("UNITREE_AES_128_KEY", raising=False)
    UnitreeWebRTCConnection(ip="192.168.123.161", aes_128_key="")
    assert "aes_128_key" not in stub_legion.call_args.kwargs


def test_empty_string_kwarg_uses_env_when_set(
    stub_legion: MagicMock, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Empty-string kwarg + set env → env value used (config "" must not block it)."""
    monkeypatch.setenv("UNITREE_AES_128_KEY", "cc" * 16)
    UnitreeWebRTCConnection(ip="192.168.123.161", aes_128_key="")
    assert _aes_kwarg(stub_legion) == "cc" * 16

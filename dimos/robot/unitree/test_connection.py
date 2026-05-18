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

"""Unit tests for UnitreeWebRTCConnection's aes_128_key kwarg + env-var fallback.

Pure-Python tests — no hardware, no network. Mocks the LegionConnection driver
and the connect() side-effect so __init__ stays inside the kwarg-forwarding logic.
"""

from typing import Any
from unittest.mock import MagicMock

import pytest

from dimos.robot.unitree import connection as conn_mod
from dimos.robot.unitree.connection import UnitreeWebRTCConnection


@pytest.fixture
def stub_legion(monkeypatch: pytest.MonkeyPatch) -> MagicMock:
    """Replace LegionConnection in the module with a MagicMock and suppress
    UnitreeWebRTCConnection.connect so __init__ doesn't try to dial out."""
    monkeypatch.setattr(UnitreeWebRTCConnection, "connect", lambda self: None)
    legion = MagicMock(name="LegionConnection")
    monkeypatch.setattr(conn_mod, "LegionConnection", legion)
    return legion


def _aes_kwarg(legion: MagicMock) -> Any:
    """Pull aes_128_key out of the LegionConnection call args, or None if absent."""
    _args, kwargs = legion.call_args
    return kwargs.get("aes_128_key")


def test_aes_key_omitted_when_neither_kwarg_nor_env(
    stub_legion: MagicMock, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Default behaviour: no kwarg, no env var → aes_128_key not forwarded.

    Guarantees the call is byte-identical to the pre-PR behaviour for users
    on G1 firmware <1.5.1 and all Go2 robots.
    """
    monkeypatch.delenv("UNITREE_AES_128_KEY", raising=False)
    UnitreeWebRTCConnection(ip="192.168.123.161")
    assert _aes_kwarg(stub_legion) is None
    assert "aes_128_key" not in stub_legion.call_args.kwargs


def test_aes_key_from_explicit_kwarg(
    stub_legion: MagicMock, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Caller passes the key directly → forwarded verbatim."""
    monkeypatch.delenv("UNITREE_AES_128_KEY", raising=False)
    UnitreeWebRTCConnection(ip="192.168.123.161", aes_128_key="aa" * 16)
    assert _aes_kwarg(stub_legion) == "aa" * 16


def test_aes_key_from_env_when_kwarg_none(
    stub_legion: MagicMock, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Env-var fallback: kwarg unset → UNITREE_AES_128_KEY is used."""
    monkeypatch.setenv("UNITREE_AES_128_KEY", "bb" * 16)
    UnitreeWebRTCConnection(ip="192.168.123.161")
    assert _aes_kwarg(stub_legion) == "bb" * 16


def test_explicit_kwarg_beats_env(stub_legion: MagicMock, monkeypatch: pytest.MonkeyPatch) -> None:
    """Precedence: explicit kwarg wins over UNITREE_AES_128_KEY env var."""
    monkeypatch.setenv("UNITREE_AES_128_KEY", "from-env")
    UnitreeWebRTCConnection(ip="192.168.123.161", aes_128_key="from-kwarg")
    assert _aes_kwarg(stub_legion) == "from-kwarg"


def test_empty_string_kwarg_falls_back_to_env_when_unset(
    stub_legion: MagicMock, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Empty-string kwarg + unset env → final key is None, nothing forwarded.

    Truthiness guard means `aes_128_key=""` falls through to the env-var
    lookup just like `aes_128_key=None` would. With the env unset, the final
    value is None and the kwarg is omitted from the LegionConnection call —
    same byte-identical behaviour as the unset case.
    """
    monkeypatch.delenv("UNITREE_AES_128_KEY", raising=False)
    UnitreeWebRTCConnection(ip="192.168.123.161", aes_128_key="")
    assert "aes_128_key" not in stub_legion.call_args.kwargs


def test_empty_string_kwarg_uses_env_when_set(
    stub_legion: MagicMock, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Empty-string kwarg + set env → env value wins.

    Guards against the bug greptile flagged on PR #2117: a YAML/JSON config
    serialising `aes_128_key: ""` instead of `null` MUST still allow the env
    var to provide the real key. Otherwise the connection silently fails on
    G1 firmware >=1.5.1 (and Go2 firmware >=1.1.15) with
    'RSA key format is not supported'.
    """
    monkeypatch.setenv("UNITREE_AES_128_KEY", "cc" * 16)
    UnitreeWebRTCConnection(ip="192.168.123.161", aes_128_key="")
    assert _aes_kwarg(stub_legion) == "cc" * 16

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

"""Unit tests for the robot-contract registry."""

from __future__ import annotations

import pytest

from dimos.manipulation.policy.contracts.piper import PiperRobotContract
from dimos.manipulation.policy.contracts.registry import (
    available_contracts,
    get_contract,
    register_contract,
)


def test_piper_is_registered_by_default():
    assert "piper" in available_contracts()


def test_get_piper_returns_piper_instance():
    inst = get_contract("piper")
    assert isinstance(inst, PiperRobotContract)


def test_unknown_contract_raises_with_available_list():
    with pytest.raises(KeyError) as excinfo:
        get_contract("does-not-exist")
    msg = str(excinfo.value)
    assert "does-not-exist" in msg
    assert "piper" in msg


def test_register_contract_round_trip():
    sentinel = object()

    def factory():  # type: ignore[no-untyped-def]
        return sentinel

    register_contract("__test_contract__", factory)
    try:
        assert "__test_contract__" in available_contracts()
        # Direct identity equality on the factory output.
        assert get_contract("__test_contract__") is sentinel
    finally:
        # Clean up so test ordering doesn't pollute other tests.
        from dimos.manipulation.policy.contracts import registry as _r

        _r._registry.pop("__test_contract__", None)

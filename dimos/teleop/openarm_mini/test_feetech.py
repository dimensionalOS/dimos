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

import builtins
import sys
from types import ModuleType

import pytest

from dimos.teleop.openarm_mini.config import OpenArmMiniDependencyError
from dimos.teleop.openarm_mini.feetech import FeetechLeaderReader, _create_sdk_handlers


class _FakePortHandler:
    def __init__(self, port: str) -> None:
        self.port = port
        self.closed = False
        self.baudrate: int | None = None

    def openPort(self) -> bool:
        return True

    def setBaudRate(self, baudrate: int) -> bool:
        self.baudrate = baudrate
        return True

    def closePort(self) -> None:
        self.closed = True


class _FakePacketHandler:
    def __init__(self, port_handler: _FakePortHandler) -> None:
        self.port_handler = port_handler

    def ReadPos(self, motor_id: int) -> tuple[int, int, int]:
        return (1000 + motor_id, 0, 0)


def test_feetech_reader_uses_direct_optional_sdk_import(monkeypatch: pytest.MonkeyPatch) -> None:
    fake_sdk = ModuleType("scservo_sdk")
    fake_sdk.PortHandler = _FakePortHandler  # type: ignore[attr-defined]
    fake_sdk.sms_sts = _FakePacketHandler  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "scservo_sdk", fake_sdk)
    reader = FeetechLeaderReader("/dev/fake", 123456)

    reader.connect()
    try:
        raw_positions = reader.read_raw_positions({"joint_1": 1, "joint_2": 7})
    finally:
        reader.disconnect()

    assert raw_positions == {"joint_1": 1001, "joint_2": 1007}


def test_create_sdk_handlers_raises_openarm_mini_dependency_error(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delitem(sys.modules, "scservo_sdk", raising=False)

    real_import = builtins.__import__

    def fake_import(name: str, *args: object, **kwargs: object) -> object:
        if name == "scservo_sdk":
            raise ImportError("missing scservo_sdk")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", fake_import)

    with pytest.raises(OpenArmMiniDependencyError):
        _create_sdk_handlers("/dev/missing")

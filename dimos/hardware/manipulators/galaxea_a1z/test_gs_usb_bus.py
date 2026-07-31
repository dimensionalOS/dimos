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

import sys
import time
from types import ModuleType
from typing import Any

import pytest

from dimos.hardware.manipulators.galaxea_a1z import gs_usb_bus


class _UsbError(Exception):
    pass


class _ReenumeratingDevice:
    def __init__(self) -> None:
        self.configuration_calls = 0
        self.configuration = object()

    def get_active_configuration(self) -> Any:
        self.configuration_calls += 1
        return self.configuration


def test_usb_discovery_retries_device_that_is_found_but_not_ready(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    device = _ReenumeratingDevice()
    core = ModuleType("usb.core")
    core.USBError = _UsbError  # type: ignore[attr-defined]
    core.find = lambda **_kwargs: device  # type: ignore[attr-defined]
    usb = ModuleType("usb")
    usb.__path__ = []  # type: ignore[attr-defined]
    usb.core = core  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "usb", usb)
    monkeypatch.setitem(sys.modules, "usb.core", core)
    monkeypatch.setattr(time, "sleep", lambda _seconds: None)
    initialization_calls = 0

    def _initialize(_device: Any, _configuration: Any) -> str:
        nonlocal initialization_calls
        initialization_calls += 1
        if initialization_calls == 1:
            raise _UsbError(2, "Entity not found")
        return "ready"

    result = gs_usb_bus._initialize_ready_usb_device(
        0xA8FA,
        0x8598,
        1.0,
        _initialize,
    )

    assert result == "ready"
    assert device.configuration_calls == 2
    assert initialization_calls == 2

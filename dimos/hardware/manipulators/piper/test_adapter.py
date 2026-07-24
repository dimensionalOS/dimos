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

import sys
from types import ModuleType
from typing import Any

piper_sdk_module = ModuleType("piper_sdk")
piper_sdk_module.__dict__["C_PiperInterface_V2"] = lambda **_: None
sys.modules.setdefault("piper_sdk", piper_sdk_module)

from dimos.hardware.manipulators.piper import adapter as piper_adapter
from dimos.hardware.manipulators.piper.adapter import PiperAdapter


def test_connect_continues_when_gripper_startup_fails(
    mocker: Any,
) -> None:
    sdk = mocker.Mock()
    sdk.GetArmStatus.return_value = object()
    sdk.GripperCtrl.side_effect = RuntimeError("gripper unavailable")
    mocker.patch.object(piper_adapter, "C_PiperInterface_V2", lambda **_: sdk)
    mocker.patch("dimos.hardware.manipulators.piper.adapter.time.sleep")

    adapter = PiperAdapter()

    assert adapter.connect()
    assert adapter.is_connected()
    assert sdk.GripperCtrl.called

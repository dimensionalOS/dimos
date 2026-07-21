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
from types import SimpleNamespace
from typing import Any

import pytest

from dimos.hardware.manipulators.piper.adapter import PiperAdapter


def test_connect_reports_missing_sdk(
    mocker: Any, capsys: pytest.CaptureFixture[str]
) -> None:
    mocker.patch.dict(sys.modules, {"piper_sdk": None})

    assert not PiperAdapter().connect()
    assert "Piper SDK not installed" in capsys.readouterr().out


def test_connect_continues_when_gripper_startup_fails(
    mocker: Any,
) -> None:
    sdk = mocker.Mock()
    sdk.GetArmStatus.return_value = object()
    sdk.GripperCtrl.side_effect = RuntimeError("gripper unavailable")
    mocker.patch.dict(
        sys.modules,
        {"piper_sdk": SimpleNamespace(C_PiperInterface_V2=lambda **_: sdk)},
    )
    mocker.patch("dimos.hardware.manipulators.piper.adapter.time.sleep")

    adapter = PiperAdapter()

    assert adapter.connect()
    assert adapter.is_connected()
    assert sdk.GripperCtrl.called

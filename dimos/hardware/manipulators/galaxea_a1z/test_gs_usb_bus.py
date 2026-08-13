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

"""A1Z-specific gs_usb front-end tests.

The generic bus behavior is covered in ``dimos/hardware/can/test_gs_usb_bus.py``.
"""

from __future__ import annotations

import can
import pytest

pytest.importorskip("can")

from dimos.hardware.manipulators.galaxea_a1z import gs_usb_bus


def test_context_manager_scopes_vendor_bus_override(
    mocker,
) -> None:
    replacement = mocker.patch.object(gs_usb_bus, "GsUsbMacBus", return_value=object())
    original_bus = can.interface.Bus

    with gs_usb_bus.gs_usb_can_bus():
        created = can.interface.Bus(
            channel="can0",
            bustype="socketcan",
            bitrate=500_000,
        )

    assert created is replacement.return_value
    replacement.assert_called_once_with(bitrate=500_000)
    assert can.interface.Bus is original_bus


def test_wrapper_defaults_to_galaxea_usb_ids() -> None:
    assert gs_usb_bus.GALAXEA_VENDOR_ID == 0xA8FA
    assert gs_usb_bus.GALAXEA_PRODUCT_ID == 0x8598

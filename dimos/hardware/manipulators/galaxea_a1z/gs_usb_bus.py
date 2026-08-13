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

"""Galaxea A1Z front-end for the shared gs_usb macOS CAN bus.

The generic bus implementation lives in ``dimos.hardware.can.gs_usb_bus``
(promoted from here when OpenYAM became its second user). This module keeps
the A1Z specifics: the HHS "CANFD Analyser" USB IDs and the context manager
that reroutes the vendor SDK's hardcoded SocketCAN construction.
"""

from __future__ import annotations

from collections.abc import Iterator
import contextlib
from typing import Any

import can

from dimos.hardware.can.gs_usb_bus import (  # noqa: F401  (re-exported for compat)
    _GS_USB_NONE_ECHO_ID,
    GsUsbMacBus as _GenericGsUsbMacBus,
    _initialize_ready_usb_device,
)

# HHS USB-CANFD adapter bundled with the Galaxea A1Z
GALAXEA_VENDOR_ID = 0xA8FA
GALAXEA_PRODUCT_ID = 0x8598


class GsUsbMacBus(_GenericGsUsbMacBus):
    """Shared gs_usb bus with the Galaxea adapter's USB IDs as defaults."""

    def __init__(
        self,
        channel: str = "gs_usb",
        *,
        vendor_id: int = GALAXEA_VENDOR_ID,
        product_id: int = GALAXEA_PRODUCT_ID,
        **kwargs: Any,
    ) -> None:
        super().__init__(channel, vendor_id=vendor_id, product_id=product_id, **kwargs)


@contextlib.contextmanager
def gs_usb_can_bus() -> Iterator[None]:
    """Route the A1Z SDK's SocketCAN construction through ``GsUsbMacBus``.

    The vendor factory hardcodes ``can.interface.Bus(..., bustype="socketcan")``.
    On macOS, replace that factory only for the duration of robot construction.
    """
    original_bus = can.interface.Bus

    def _bus_factory(*args: Any, **kwargs: Any) -> GsUsbMacBus:
        return GsUsbMacBus(bitrate=kwargs.get("bitrate", 1_000_000))

    can.interface.Bus = _bus_factory  # type: ignore[assignment]
    try:
        yield
    finally:
        can.interface.Bus = original_bus  # type: ignore[assignment]

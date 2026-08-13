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

from __future__ import annotations

from collections.abc import Callable, Iterator
import queue
import sys
import threading
import time
from types import ModuleType
from typing import Any

import can
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


class _FakeGsUsbFrame:
    last_packed: _FakeGsUsbFrame | None = None

    def __init__(self, can_id: int = 0, data: bytes = b"") -> None:
        self.can_id = can_id
        self.data = data
        self.can_dlc = len(data)
        self.echo_id = gs_usb_bus._GS_USB_NONE_ECHO_ID

    def pack(self, _hardware_timestamp: bool) -> bytes:
        self.__class__.last_packed = self
        return b"packed-frame"


class _FakeUsbEndpoint:
    def __init__(self) -> None:
        self.writes: list[tuple[int, bytes]] = []

    def write(self, endpoint: int, data: bytes) -> None:
        self.writes.append((endpoint, data))


class _FakeGsUsb:
    def __init__(self) -> None:
        self.device_flags = 0
        self.gs_usb = _FakeUsbEndpoint()
        self.frames: queue.Queue[tuple[int, bytes, int]] = queue.Queue()
        self.frames_read = threading.Event()
        self.stopped = False

    def read(self, frame: _FakeGsUsbFrame, timeout_ms: int) -> bool:
        try:
            can_id, data, echo_id = self.frames.get(timeout=timeout_ms / 1000)
        except queue.Empty:
            return False
        frame.can_id = can_id
        frame.data = data
        frame.can_dlc = len(data)
        frame.echo_id = echo_id
        if self.frames.empty():
            self.frames_read.set()
        return True

    def stop(self) -> None:
        self.stopped = True


_MacBus = tuple[gs_usb_bus.GsUsbMacBus, _FakeGsUsb]


def _install_fake_gs_usb_modules(monkeypatch: pytest.MonkeyPatch) -> None:
    package = ModuleType("gs_usb")
    package.__path__ = []
    gs_module = ModuleType("gs_usb.gs_usb")
    gs_module.GS_CAN_MODE_HW_TIMESTAMP = 1
    gs_module.GsUsb = _FakeGsUsb
    frame_module = ModuleType("gs_usb.gs_usb_frame")
    frame_module.GsUsbFrame = _FakeGsUsbFrame
    monkeypatch.setitem(sys.modules, "gs_usb", package)
    monkeypatch.setitem(sys.modules, "gs_usb.gs_usb", gs_module)
    monkeypatch.setitem(sys.modules, "gs_usb.gs_usb_frame", frame_module)


@pytest.fixture
def mac_bus_factory(
    monkeypatch: pytest.MonkeyPatch,
    mocker,
) -> Iterator[Callable[[], _MacBus]]:
    _install_fake_gs_usb_modules(monkeypatch)
    buses: list[gs_usb_bus.GsUsbMacBus] = []

    def create() -> _MacBus:
        fake_gs = _FakeGsUsb()
        mocker.patch.object(
            gs_usb_bus,
            "_initialize_ready_usb_device",
            return_value=(fake_gs, 0x01),
        )
        bus = gs_usb_bus.GsUsbMacBus()
        buses.append(bus)
        return bus, fake_gs

    yield create
    for bus in buses:
        bus.shutdown()


@pytest.fixture
def mac_bus(mac_bus_factory: Callable[[], _MacBus]) -> _MacBus:
    return mac_bus_factory()


def test_usb_discovery_retries_device_that_is_found_but_not_ready(
    monkeypatch: pytest.MonkeyPatch,
    mocker,
) -> None:
    device = _ReenumeratingDevice()
    core = ModuleType("usb.core")
    core.USBError = _UsbError
    core.find = mocker.Mock(return_value=device)
    usb = ModuleType("usb")
    usb.__path__ = []
    usb.core = core
    monkeypatch.setitem(sys.modules, "usb", usb)
    monkeypatch.setitem(sys.modules, "usb.core", core)
    mocker.patch.object(time, "sleep")
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


def test_send_writes_packed_frame_to_discovered_endpoint(
    mac_bus: tuple[gs_usb_bus.GsUsbMacBus, _FakeGsUsb],
) -> None:
    bus, fake_gs = mac_bus

    bus.send(can.Message(arbitration_id=0x123, data=[1, 2, 3]))

    assert fake_gs.gs_usb.writes == [(0x01, b"packed-frame")]
    assert _FakeGsUsbFrame.last_packed is not None
    assert _FakeGsUsbFrame.last_packed.can_id == 0x123
    assert _FakeGsUsbFrame.last_packed.data == b"\x01\x02\x03"


def test_reader_filters_echo_and_returns_bus_frame(
    mac_bus: tuple[gs_usb_bus.GsUsbMacBus, _FakeGsUsb],
) -> None:
    bus, fake_gs = mac_bus
    fake_gs.frames.put((0x123, b"\x01", 0))
    fake_gs.frames.put((0x80000456, b"\x02\x03", gs_usb_bus._GS_USB_NONE_ECHO_ID))

    message = bus.recv(timeout=1.0)

    assert message is not None
    assert message.arbitration_id == 0x456
    assert message.is_extended_id
    assert bytes(message.data) == b"\x02\x03"


def test_queue_overflow_keeps_freshest_frame(
    mac_bus_factory: Callable[[], _MacBus],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(gs_usb_bus, "_RX_QUEUE_MAX_FRAMES", 1)
    bus, fake_gs = mac_bus_factory()
    fake_gs.frames.put((0x101, b"\x01", gs_usb_bus._GS_USB_NONE_ECHO_ID))
    fake_gs.frames.put((0x102, b"\x02", gs_usb_bus._GS_USB_NONE_ECHO_ID))
    assert fake_gs.frames_read.wait(timeout=1.0)

    message = bus.recv(timeout=1.0)

    assert message is not None
    assert message.arbitration_id == 0x102
    assert bus._rx_dropped == 1


def test_shutdown_stops_reader_and_usb_device(
    mac_bus: tuple[gs_usb_bus.GsUsbMacBus, _FakeGsUsb],
) -> None:
    bus, fake_gs = mac_bus

    bus.shutdown()

    assert fake_gs.stopped
    assert not bus._rx_thread.is_alive()

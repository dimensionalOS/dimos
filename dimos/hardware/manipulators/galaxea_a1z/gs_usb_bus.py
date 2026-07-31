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

"""python-can Bus for gs_usb CAN adapters via libusb - works on macOS.

SocketCAN is Linux-only; this bus drives gs_usb-protocol adapters (candlelight
class, including the HHS "CANFD Analyser" Galaxea ships) entirely from
userspace over libusb, so the A1Z runs natively on macOS. Validated on an
M4 Pro at a sustained 250 Hz control loop (p99 cycle 5.2 ms, 0/7500 cycles
over the SDK's 12.5 ms limit, ~100% feedback from all 7 motors).

Device quirks handled here:
- TX endpoint is discovered from the descriptors (this adapter uses 0x01;
  the gs_usb library assumes 0x02)
- macOS has no kernel driver to detach; the detach call is skipped
- TX echo frames are filtered out of recv()
- RX runs on a dedicated reader thread (see _rx_loop): the adapter is a
  Full-Speed USB device read one frame per libusb round-trip, and the SDK's
  250 Hz loop budgets only ~1 ms/cycle for draining. Synchronous reads from
  that budget cannot keep up with the ~3500 frames/s the bus carries (motor
  feedback plus TX echoes), so the device FIFO overflows and feedback
  freezes for hundreds of ms (observed on hardware: frozen joints in teach
  recordings, "CAN feedback stale" during replay). The reader thread drains
  USB continuously - libusb releases the GIL while blocked - and recv()
  becomes a queue pop that always meets the SDK's budget.

Requires the locked ``galaxea-a1z`` dependency group plus system libusb; run
``dimos/robot/manipulators/galaxea_a1z/scripts/setup_a1z.sh`` from the repo.
Lives in galaxea_a1z/ because it is the only user today; promote to a shared
location when a second CAN arm needs it.
"""

from __future__ import annotations

from collections.abc import Callable
import contextlib
import queue
import threading
import time
from typing import Any, TypeVar

import can

# HHS USB-CANFD adapter bundled with the Galaxea A1Z
GALAXEA_VENDOR_ID = 0xA8FA
GALAXEA_PRODUCT_ID = 0x8598

_GS_USB_NONE_ECHO_ID = 0xFFFFFFFF
_GS_CAN_MODE_LISTEN_ONLY = 1 << 0

# Reader-thread queue depth. At the arm's ~1750 feedback frames/s this holds
# multiple seconds of backlog; the consumer (SDK drain) normally keeps the
# queue near-empty, and on overflow the oldest frames are dropped so recv()
# keeps returning fresh state instead of replaying stale history.
_RX_QUEUE_MAX_FRAMES = 8192
_RX_READ_TIMEOUT_MS = 20
_UsbInitialization = TypeVar("_UsbInitialization")


def _initialize_ready_usb_device(
    vendor_id: int,
    product_id: int,
    discover_timeout: float,
    initialize: Callable[[Any, Any], _UsbInitialization],
) -> _UsbInitialization:
    """Wait for a USB device and complete a caller-supplied initialization.

    After ``GsUsb.stop()``, this adapter briefly re-enumerates on macOS. During
    that window PyUSB can find its descriptor while configuration or control
    transfers still raise ``USBError(ENOENT, "Entity not found")``. Retry the
    complete initialization so immediate DimOS restarts are reliable.
    """
    import usb.core  # type: ignore[import-untyped]

    deadline = time.perf_counter() + discover_timeout
    last_error: Exception | None = None
    while True:
        device = usb.core.find(idVendor=vendor_id, idProduct=product_id)
        if device is not None:
            try:
                configuration = device.get_active_configuration()
                return initialize(device, configuration)
            except usb.core.USBError as exc:
                last_error = exc

        if time.perf_counter() >= deadline:
            message = (
                f"gs_usb adapter {vendor_id:04x}:{product_id:04x} not ready on USB "
                f"(waited {discover_timeout:.0f}s)"
            )
            raise can.CanInitializationError(message) from last_error
        time.sleep(0.25)


class GsUsbMacBus(can.BusABC):
    """CAN bus over a gs_usb adapter through libusb (macOS-friendly)."""

    def __init__(
        self,
        channel: str = "gs_usb",
        *,
        vendor_id: int = GALAXEA_VENDOR_ID,
        product_id: int = GALAXEA_PRODUCT_ID,
        bitrate: int = 1_000_000,
        listen_only: bool = False,
        discover_timeout: float = 5.0,
        **_: Any,
    ) -> None:
        from gs_usb.gs_usb import (  # type: ignore[import-untyped]
            GS_CAN_MODE_HW_TIMESTAMP,
            GsUsb,
        )

        def _initialize(device: Any, cfg: Any) -> tuple[Any, int]:
            # No kernel driver claims the interface on macOS; gs_usb's detach
            # call would raise, so neutralize it.
            device.detach_kernel_driver = lambda intf: None

            # The gs_usb library hardcodes TX endpoint 0x02; discover the real
            # bulk OUT endpoint from the active configuration instead.
            intf = cfg[(0, 0)]
            out_eps = [ep for ep in intf if not (ep.bEndpointAddress & 0x80)]
            if not out_eps:
                raise can.CanInitializationError("gs_usb adapter has no OUT endpoint")
            gs = GsUsb(device)
            if not gs.set_bitrate(bitrate):
                raise can.CanInitializationError(f"failed to set bitrate {bitrate}")
            gs.start(_GS_CAN_MODE_LISTEN_ONLY if listen_only else 0)
            return gs, out_eps[0].bEndpointAddress

        self._gs, self._out_endpoint = _initialize_ready_usb_device(
            vendor_id,
            product_id,
            discover_timeout,
            _initialize,
        )
        self._hw_timestamp_flag = GS_CAN_MODE_HW_TIMESTAMP
        self._flush_rx()

        self._rx_queue: queue.Queue[can.Message] = queue.Queue(maxsize=_RX_QUEUE_MAX_FRAMES)
        self._rx_dropped = 0
        self._rx_stop = threading.Event()
        self._rx_thread = threading.Thread(target=self._rx_loop, name="gs_usb_rx", daemon=True)
        self._rx_thread.start()

        self.channel_info = f"gs_usb {vendor_id:04x}:{product_id:04x} @ {bitrate}"
        super().__init__(channel=channel)

    def _flush_rx(self, max_frames: int = 1024) -> int:
        """Discard frames queued in the device from a previous session.

        The device keeps its RX queue across open/close; stale frames (e.g.
        disable-command acks) parse as motor feedback with garbage velocity
        values and trip startup safety checks. Observed on hardware.
        """
        from gs_usb.gs_usb_frame import GsUsbFrame  # type: ignore[import-untyped]

        frame = GsUsbFrame()
        flushed = 0
        while flushed < max_frames and self._gs.read(frame, 5):
            flushed += 1
        if flushed:
            print(f"GsUsbMacBus: flushed {flushed} stale frames from device queue")
        return flushed

    @property  # type: ignore[misc]
    def state(self) -> can.BusState:
        return can.BusState.ACTIVE

    def send(self, msg: can.Message, timeout: float | None = None) -> None:
        from gs_usb.gs_usb_frame import GsUsbFrame  # type: ignore[import-untyped]

        frame = GsUsbFrame(can_id=msg.arbitration_id, data=bytes(msg.data))
        hw_ts = bool(self._gs.device_flags & self._hw_timestamp_flag)
        self._gs.gs_usb.write(self._out_endpoint, frame.pack(hw_ts))

    def _rx_loop(self) -> None:
        """Continuously drain the device into the RX queue.

        Runs until shutdown. libusb releases the GIL for the duration of each
        blocking read, so this thread keeps the device FIFO empty even while
        the SDK control thread and the rest of the process compete for the
        interpreter. TX echoes are discarded here so they never consume the
        consumer's drain budget.
        """
        from gs_usb.gs_usb_frame import GsUsbFrame  # type: ignore[import-untyped]

        frame = GsUsbFrame()
        while not self._rx_stop.is_set():
            try:
                got = self._gs.read(frame, _RX_READ_TIMEOUT_MS)
            except Exception:
                if self._rx_stop.is_set():
                    return
                # Transient libusb error (e.g. device re-enumerating); back
                # off briefly instead of spinning.
                time.sleep(0.01)
                continue
            if not got:
                continue
            if frame.echo_id != _GS_USB_NONE_ECHO_ID:
                continue  # our own TX echo, not bus traffic

            msg = can.Message(
                arbitration_id=frame.can_id & 0x1FFFFFFF,
                is_extended_id=bool(frame.can_id & 0x80000000),
                data=bytes(frame.data[: frame.can_dlc]),
                dlc=frame.can_dlc,
            )
            try:
                self._rx_queue.put_nowait(msg)
            except queue.Full:
                # Consumer stalled: drop the oldest frame so the queue holds
                # the freshest state. Single producer, so this cannot race
                # another put.
                with contextlib.suppress(queue.Empty):
                    self._rx_queue.get_nowait()
                    self._rx_dropped += 1
                with contextlib.suppress(queue.Full):
                    self._rx_queue.put_nowait(msg)

    def _recv_internal(self, timeout: float | None) -> tuple[can.Message | None, bool]:
        # The SDK's feedback drain calls recv(timeout=0.0) in a tight loop
        # with a ~1 ms budget; a true non-blocking pop keeps every call well
        # inside that budget.
        try:
            if timeout is not None and timeout <= 0:
                return self._rx_queue.get_nowait(), False
            return self._rx_queue.get(timeout=timeout), False
        except queue.Empty:
            return None, False

    def shutdown(self) -> None:
        self._rx_stop.set()
        rx_thread = getattr(self, "_rx_thread", None)
        if rx_thread is not None and rx_thread.is_alive():
            rx_thread.join(timeout=1.0)
        if self._rx_dropped:
            print(f"GsUsbMacBus: dropped {self._rx_dropped} RX frames on queue overflow")
        try:
            self._gs.stop()
        except Exception:
            pass
        super().shutdown()

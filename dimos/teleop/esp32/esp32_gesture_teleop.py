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

"""ESP32 gesture sensor as a handheld controller: BLE advertisements in, ``Joy`` out.

Only listens to advertisements from one configured board. It never connects to the
board, loads a robot connection or sends robot commands; what a gesture does is up to
whichever module subscribes to ``gesture_joy``.

Each gesture is published as a tap on ``gesture_joy``: one ``Joy`` with that gesture's
button set to 1, then, ``press_hold_s`` later, one with all buttons 0. ``buttons``
follows ``GESTURES`` (``[SIT, STAND]``). ``axes[0]`` is how far the hand was from the
sensor, scaled to 0..1 over 0..1 m (1 also means no echo). ``frame_id`` is
``"esp32_gesture"``.
"""

from __future__ import annotations

import asyncio
import threading
import time
from typing import Any

from bleak import BleakScanner
from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.teleop.esp32.packets import (
    GESTURES,
    MANUFACTURER_ID,
    GesturePacket,
    GestureTracker,
    decode_packet,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

FRAME_ID = "esp32_gesture"
FULL_SCALE_CM = 100.0


class Esp32GestureTeleopConfig(ModuleConfig):
    """Settings for the ESP32 gesture sensor.

    Attributes:
        address: BLE address of the board, e.g. "AA:BB:CC:DD:EE:FF". Required.
        press_hold_s: How long a gesture's button stays pressed before the release.
        scan_start_timeout_s: How long starting the Bluetooth scan may take.
        poll_interval_s: How often the online/stale state is checked and logged.
        retry_max_s: Longest wait between retries after the Bluetooth scanner fails.
    """

    address: str = ""
    press_hold_s: float = Field(default=0.1, ge=0, allow_inf_nan=False)
    scan_start_timeout_s: float = Field(default=15.0, gt=0, allow_inf_nan=False)
    poll_interval_s: float = Field(default=0.5, gt=0, allow_inf_nan=False)
    retry_max_s: float = Field(default=30.0, gt=0, allow_inf_nan=False)


def gesture_joy(packet: GesturePacket) -> tuple[Joy, Joy]:
    """The press and the release for one gesture."""
    cm = packet.distance_cm or FULL_SCALE_CM  # 0 means no echo: as far as it gets
    axes = [min(cm, FULL_SCALE_CM) / FULL_SCALE_CM]
    pressed = [int(name == packet.gesture) for name in GESTURES]
    return (
        Joy(frame_id=FRAME_ID, axes=axes, buttons=pressed),
        Joy(frame_id=FRAME_ID, axes=axes, buttons=[0] * len(GESTURES)),
    )


class Esp32GestureTeleop(Module):
    config: Esp32GestureTeleopConfig

    gesture_joy: Out[Joy]

    _thread: threading.Thread | None = None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop_event = threading.Event()
        self._tracker = GestureTracker()
        self._last_warning: str | None = None
        self._release_lock = threading.Lock()
        self._release_timer: threading.Timer | None = None
        self._pending_release: Joy | None = None

    @rpc
    def start(self) -> None:
        if not self.config.address:
            raise ValueError("Esp32GestureTeleop needs the board's BLE address (address=...)")
        super().start()
        self._stop_event.clear()
        self._tracker = GestureTracker()
        self._thread = threading.Thread(target=self._run, name="esp32-gestures", daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        # A tap being published holds the lock, so it finishes before this flush; later taps see
        # the stop event and publish nothing.
        self._flush_release()  # never leave a button pressed
        if self._thread is not None:
            # The scanner may still be starting (scan_start_timeout_s) or stopping (5 s).
            self._thread.join(timeout=self.config.scan_start_timeout_s + 5)
            if self._thread.is_alive():
                logger.warning("[esp32] Bluetooth scanner did not stop in time")
            self._thread = None
        self._flush_release()
        super().stop()

    def _run(self) -> None:
        """Scan until stopped; if the scanner fails, log it and retry with backoff."""
        wait_s = 1.0
        while not self._stop_event.is_set():
            try:
                asyncio.run(self._listen())
                return
            except Exception:
                logger.exception(f"[esp32] Bluetooth scanner failed; retrying in {wait_s:.0f} s")
            if self._stop_event.wait(wait_s):
                return
            wait_s = min(wait_s * 2, self.config.retry_max_s)

    async def _listen(self) -> None:
        scanner = BleakScanner(detection_callback=self._on_advertisement)
        await asyncio.wait_for(scanner.start(), timeout=self.config.scan_start_timeout_s)
        logger.info(f"[esp32] listening for {self._address}")
        try:
            last_state: str | None = None
            while not self._stop_event.is_set():
                state = self._tracker.state(time.monotonic())
                if state != last_state:
                    logger.info(f"[esp32] {self._address} {state}")
                    last_state = state
                await asyncio.sleep(self.config.poll_interval_s)
        finally:
            await asyncio.wait_for(scanner.stop(), timeout=5)

    @property
    def _address(self) -> str:
        return self.config.address.upper()

    def _on_advertisement(self, device: Any, advertisement: Any) -> None:
        if device.address.upper() != self._address:
            return
        payload = advertisement.manufacturer_data.get(MANUFACTURER_ID)
        if payload is None:
            return  # Scan responses carry no manufacturer data.
        self._on_payload(bytes(payload), time.monotonic())

    def _on_payload(self, payload: bytes, now: float) -> None:
        try:
            packet = decode_packet(payload)
        except ValueError as exc:
            self._warn(str(exc))
            return
        is_new, warning = self._tracker.observe(packet, now)
        self._warn(warning)
        if not is_new:
            return
        press, release = gesture_joy(packet)
        self._publish_tap(press, release)
        logger.info(f"[esp32] {packet.gesture} count={packet.count} cm={packet.distance_cm}")

    def _publish_tap(self, press: Joy, release: Joy) -> None:
        with self._release_lock:
            if self._stop_event.is_set():
                return  # stopping: publish nothing new
            # A still-pending release from the previous gesture goes out first, so taps never overlap.
            self._flush_release_locked()
            self.gesture_joy.publish(press)
            if self.config.press_hold_s <= 0:
                self.gesture_joy.publish(release)
                return
            self._pending_release = release
            timer = threading.Timer(
                self.config.press_hold_s, lambda: self._release_after_hold(timer)
            )
            timer.daemon = True
            self._release_timer = timer
            timer.start()

    def _release_after_hold(self, timer: threading.Timer) -> None:
        with self._release_lock:
            # A timer that lost the race to a newer tap must not release that tap early.
            if self._release_timer is timer:
                self._flush_release_locked()

    def _flush_release(self) -> None:
        with self._release_lock:
            self._flush_release_locked()

    def _flush_release_locked(self) -> None:
        if self._release_timer is not None:
            self._release_timer.cancel()
            self._release_timer = None
        release, self._pending_release = self._pending_release, None
        if release is not None:
            self.gesture_joy.publish(release)

    def _warn(self, warning: str | None) -> None:
        if warning is not None and warning != self._last_warning:
            logger.warning(f"[esp32] {self._address}: {warning}")
        self._last_warning = warning

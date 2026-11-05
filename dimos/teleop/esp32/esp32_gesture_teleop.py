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
whichever module subscribes to ``joystick``.

Each gesture is published as a tap on ``joystick``: one ``Joy`` with that gesture's
button set to 1, then one with all buttons 0. ``buttons`` follows ``GESTURES``
(``[SIT, STAND]``); ``axes[0]`` is the hand distance in metres.
"""

from __future__ import annotations

import asyncio
import threading
import time
from typing import Any

from bleak import BleakScanner

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


class Esp32GestureTeleopConfig(ModuleConfig):
    address: str = ""  # BLE address of the board, e.g. "D4:E9:F4:E2:79:02"
    scan_start_timeout_s: float = 15.0
    poll_interval_s: float = 0.5


def gesture_joy(packet: GesturePacket) -> list[Joy]:
    """The press-then-release pair for one gesture."""
    axes = [packet.distance_cm / 100.0]
    pressed = [int(name == packet.gesture) for name in GESTURES]
    return [Joy(axes=axes, buttons=pressed), Joy(axes=axes, buttons=[0] * len(GESTURES))]


class Esp32GestureTeleop(Module):
    config: Esp32GestureTeleopConfig

    joystick: Out[Joy]

    _thread: threading.Thread | None = None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop_event = threading.Event()
        self._tracker = GestureTracker()
        self._last_warning: str | None = None

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
        if self._thread is not None:
            self._thread.join(timeout=10)
            if self._thread.is_alive():
                logger.warning("[esp32] Bluetooth scanner did not stop within 10 seconds")
            self._thread = None
        super().stop()

    def _run(self) -> None:
        try:
            asyncio.run(self._listen())
        except Exception:
            logger.exception("[esp32] Bluetooth scanner failed")

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
        for msg in gesture_joy(packet):
            self.joystick.publish(msg)
        logger.info(f"[esp32] {packet.gesture} count={packet.count} cm={packet.distance_cm}")

    def _warn(self, warning: str | None) -> None:
        if warning is not None and warning != self._last_warning:
            logger.warning(f"[esp32] {self._address}: {warning}")
        self._last_warning = warning

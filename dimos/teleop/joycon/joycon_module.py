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

"""Nintendo Joy-Cons as a teleop input: jaw triggers, episode buttons and a pause button.

Each Joy-Con pairs over Bluetooth, and the Linux ``hid-nintendo`` driver shows
it as an input device. The module reads those devices and publishes on streams
the follower and the episode monitor already listen to:

- ZL / ZR held close the left / right jaw.
- Every button goes out as ``teleop_buttons``; the episode monitor's defaults
  make B start or save an episode and Y discard it.
- plus flips ``follow`` off and on; a leader connection holds the follower
  while it is off.
"""

from __future__ import annotations

from collections.abc import Mapping
import os
from pathlib import Path
import re
import struct
import threading
import time
from typing import Literal

from pydantic import Field

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.teleop.webxr.controller_types import Buttons
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

JoyConSide = Literal["left", "right"]
JOYCON_SIDES: tuple[JoyConSide, ...] = ("left", "right")
# Part of the input device name the driver gives each Joy-Con. It also
# registers an "(IMU)" device per Joy-Con, which only root can read.
_DEVICE_NAMES: dict[JoyConSide, str] = {"left": "Joy-Con (L)", "right": "Joy-Con (R)"}
# Key codes the driver reports (``evtest`` shows them live), and the bit of the
# WebXR Buttons message each lands on. The right Joy-Con's X and Y go to the
# left-hand bits, which carry the X and Y names the episode monitor uses.
_BUTTON_BITS: dict[JoyConSide, dict[int, str]] = {
    "left": {312: "left_trigger", 310: "left_grip", 314: "left_menu", 317: "left_thumbstick"},
    "right": {
        313: "right_trigger",
        311: "right_grip",
        305: "right_primary",
        304: "right_secondary",
        307: "left_primary",
        308: "left_secondary",
        315: "right_menu",
        318: "right_thumbstick",
    },
}
_JAW_KEYS: dict[JoyConSide, int] = {"left": 312, "right": 313}  # ZL, ZR
_PLUS_KEY = 315
# struct input_event from linux/input.h: time, type, code, value.
_EVENT_FORMAT = "llHHi"
_EVENT_SIZE = struct.calcsize(_EVENT_FORMAT)
_EV_KEY = 1
_RESCAN_PERIOD_S = 1.0
_DEVICES_FILE = Path("/proc/bus/input/devices")

# The keys held on each Joy-Con, or None for one that is not connected.
Pressed = Mapping[JoyConSide, frozenset[int] | None]


class JoyConTeleopModuleConfig(ModuleConfig):
    """Settings for the Joy-Con module.

    Attributes:
        read_rate_hz: How often the Joy-Cons are read and published, in Hz.
        jaw_speed_per_s: How fast a jaw opening moves, in full openings per
            second, so a jaw never slams; 3 closes it in about 0.33 s.
    """

    read_rate_hz: float = Field(default=50.0, gt=0, allow_inf_nan=False)
    jaw_speed_per_s: float = Field(default=3.0, gt=0, allow_inf_nan=False)


class JoyConTeleopModule(Module):
    """Read both Joy-Cons and publish jaw openings, buttons and follow on/off."""

    config: JoyConTeleopModuleConfig
    teleop_buttons: Out[Buttons]
    # Jaw openings, 1 = open, for the follower coordinator's gripper tasks.
    left_gripper_command: Out[Float32]
    right_gripper_command: Out[Float32]
    # Sent when plus flips it; following starts on.
    follow: Out[Bool]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._openings: dict[JoyConSide, float] = dict.fromkeys(JOYCON_SIDES, 1.0)
        self._following = True
        self._plus_was_down = False
        self._devices: _JoyConDevices | None = None
        self._stop_reading = threading.Event()
        self._reader: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._devices = _JoyConDevices()
        self._reader = threading.Thread(target=self._read_loop, daemon=True, name="joycon-read")
        self._reader.start()

    @rpc
    def stop(self) -> None:
        self._stop_reading.set()
        if self._reader is not None and self._reader.is_alive():
            self._reader.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        if self._devices is not None:
            self._devices.close()
            self._devices = None
        super().stop()

    def publish_reading(self, pressed: Pressed, dt: float) -> None:
        """Publish the buttons, both jaw openings and any change to follow.

        Args:
            pressed: The key codes held on each Joy-Con, or None for one that
                is not connected; its jaw stays where it is.
            dt: Seconds since the previous reading, for the jaw speed.
        """
        buttons = Buttons()
        for side in JOYCON_SIDES:
            for key in pressed[side] or ():
                if key in _BUTTON_BITS[side]:
                    buttons.set_attribute(_BUTTON_BITS[side][key], True)
        jaws_held = {side: _JAW_KEYS[side] in (pressed[side] or ()) for side in JOYCON_SIDES}
        buttons.pack_analog_triggers(float(jaws_held["left"]), float(jaws_held["right"]))
        self.teleop_buttons.publish(buttons)

        outputs = {"left": self.left_gripper_command, "right": self.right_gripper_command}
        step = self.config.jaw_speed_per_s * max(0.0, dt)
        for side in JOYCON_SIDES:
            if pressed[side] is None:
                continue
            target = 0.0 if jaws_held[side] else 1.0
            opening = self._openings[side]
            self._openings[side] = opening + max(-step, min(step, target - opening))
            outputs[side].publish(Float32(data=self._openings[side]))

        plus = _PLUS_KEY in (pressed["right"] or ())
        if plus and not self._plus_was_down:
            self._following = not self._following
            logger.info("Joy-Con plus: following %s", "on" if self._following else "off")
            self.follow.publish(Bool(data=self._following))
        self._plus_was_down = plus

    def _read_loop(self) -> None:
        period = 1.0 / self.config.read_rate_hz
        last = time.monotonic()
        while not self._stop_reading.wait(period):
            devices = self._devices
            if devices is None:
                return
            now = time.monotonic()
            try:
                self.publish_reading(devices.read(), now - last)
            except Exception:
                logger.exception("Joy-Con read failed")
            last = now


class _JoyConDevices:
    """Both Joy-Cons' input devices, opened when they appear and dropped when they go."""

    def __init__(self) -> None:
        self._fds: dict[JoyConSide, int] = {}
        self._held: dict[JoyConSide, set[int]] = {side: set() for side in JOYCON_SIDES}
        self._next_scan = 0.0

    def read(self) -> dict[JoyConSide, frozenset[int] | None]:
        """The keys held on each Joy-Con now, or None for one that is not connected."""
        if len(self._fds) < len(JOYCON_SIDES) and time.monotonic() >= self._next_scan:
            self._scan()
        return {
            side: frozenset(self._held[side]) if side in self._fds and self._drain(side) else None
            for side in JOYCON_SIDES
        }

    def close(self) -> None:
        for side in list(self._fds):
            self._drop(side)

    def _scan(self) -> None:
        self._next_scan = time.monotonic() + _RESCAN_PERIOD_S
        devices = _input_devices()
        for side in JOYCON_SIDES:
            path = next(
                (
                    path
                    for name, path in devices.items()
                    if _DEVICE_NAMES[side] in name and "(IMU)" not in name
                ),
                None,
            )
            if side in self._fds or path is None:
                continue
            try:
                self._fds[side] = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
            except OSError as error:
                logger.warning("Joy-Con %s at %s cannot be opened: %s", side, path, error)
                continue
            self._held[side] = set()
            logger.info("Joy-Con %s connected at %s", side, path)

    def _drain(self, side: JoyConSide) -> bool:
        """Apply every queued key event; False once the device is gone."""
        while True:
            try:
                data = os.read(self._fds[side], _EVENT_SIZE * 64)
            except BlockingIOError:
                return True
            except OSError:
                data = b""
            if not data:
                self._drop(side)
                logger.warning("Joy-Con %s disconnected; its jaw stays where it is", side)
                return False
            for offset in range(0, len(data) - len(data) % _EVENT_SIZE, _EVENT_SIZE):
                _, _, kind, code, value = struct.unpack_from(_EVENT_FORMAT, data, offset)
                if kind == _EV_KEY:
                    if value:
                        self._held[side].add(code)
                    else:
                        self._held[side].discard(code)

    def _drop(self, side: JoyConSide) -> None:
        fd = self._fds.pop(side, None)
        if fd is not None:
            os.close(fd)


def _input_devices(devices_file: Path = _DEVICES_FILE) -> dict[str, str]:
    """Each input device's name and its /dev/input/event* path, from the kernel's list."""
    paths: dict[str, str] = {}
    try:
        text = devices_file.read_text()
    except OSError:
        return paths
    for block in text.split("\n\n"):
        name = re.search(r'^N: Name="(.*)"$', block, re.M)
        handlers = re.search(r"^H: Handlers=(.*)$", block, re.M)
        event = re.search(r"\bevent\d+\b", handlers.group(1)) if handlers else None
        if name and event:
            paths[name.group(1)] = f"/dev/input/{event.group(0)}"
    return paths

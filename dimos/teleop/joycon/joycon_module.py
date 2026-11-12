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

"""Nintendo Joy-Cons as a teleop input: buttons, gripper openings and a base twist.

Each Joy-Con pairs over Bluetooth as its own device. Linux reads hid-nintendo
event devices; macOS reads HID reports with the optional joycon-teleop extra.
Nothing here is specific to a robot: the module publishes the same streams the
WebXR and keyboard teleop modules do, so the episode monitor and the gripper
tasks consume it as is.
"""

from __future__ import annotations

from collections.abc import Mapping
import os
from pathlib import Path
import re
import struct
import sys
import threading
import time
from typing import Literal, Protocol

from pydantic import Field

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.teleop.joycon.hid_backend import HidJoyConBackend
from dimos.teleop.joycon.mapping import (
    DEFAULT_AXIS_INDEX,
    DEFAULT_BUTTON_INDEX,
    GRIPPER_LABEL,
    JOYCON_SIDES,
    STICK_FULL_SCALE,
    GripperMode,
    JoyConReading,
    JoyConSide,
    JoyConState,
    buttons_message,
    pressed_labels,
    step_gripper,
    stick_to_base_velocity,
)
from dimos.teleop.joycon.raw import encode_readings
from dimos.teleop.webxr.controller_types import Buttons
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class JoyConBackend(Protocol):
    """Where the readings come from: the kernel driver on a machine, a fake in tests."""

    def open(self) -> None: ...
    def read(self) -> dict[JoyConSide, JoyConReading | None]: ...
    def close(self) -> None: ...


class JoyConTeleopModuleConfig(ModuleConfig):
    backend: Literal["auto", "evdev", "hid"] = "auto"
    hid_report_timeout_s: float = Field(default=0.2, gt=0.0, le=0.25)
    tick_period_s: float = Field(default=0.02, gt=0.0)
    # Substring of the input device name the kernel driver registers for each side.
    left_name: str = "Joy-Con (L)"
    right_name: str = "Joy-Con (R)"
    # Key and axis codes per side; run `python -m dimos.teleop.joycon.probe` and
    # override these when a driver version reports others.
    left_buttons: dict[str, int] = Field(default_factory=lambda: dict(DEFAULT_BUTTON_INDEX["left"]))
    right_buttons: dict[str, int] = Field(
        default_factory=lambda: dict(DEFAULT_BUTTON_INDEX["right"])
    )
    left_axes: dict[str, int] = Field(default_factory=lambda: dict(DEFAULT_AXIS_INDEX["left"]))
    right_axes: dict[str, int] = Field(default_factory=lambda: dict(DEFAULT_AXIS_INDEX["right"]))
    # hold: closed while ZL/ZR is held. toggle: a press flips open/closed.
    gripper_mode: GripperMode = "hold"
    # The published opening moves at most this much per second (1 = full travel).
    gripper_slew_per_s: float = Field(default=3.0, gt=0.0)
    deadzone: float = Field(default=0.15, ge=0.0, lt=1.0)
    # Left stick drives a base through cmd_vel only when this is on.
    publish_cmd_vel: bool = False
    max_linear_m_s: float = Field(default=0.3, gt=0.0)
    max_angular_rad_s: float = Field(default=0.6, gt=0.0)
    # Robot-specific profiles consume the full sample and own all output mapping.
    publish_legacy_commands: bool = True

    def button_index(self, side: JoyConSide) -> dict[str, int]:
        return self.left_buttons if side == "left" else self.right_buttons

    def axis_index(self, side: JoyConSide) -> dict[str, int]:
        return self.left_axes if side == "left" else self.right_axes

    def joystick_name(self, side: JoyConSide) -> str:
        return self.left_name if side == "left" else self.right_name


_EVENT_FORMAT = "llHHi"
_EVENT_SIZE = struct.calcsize(_EVENT_FORMAT)
_EV_KEY = 1
_EV_ABS = 3
_RESCAN_PERIOD_S = 1.0
_DEVICES_FILE = Path("/proc/bus/input/devices")


def input_device_paths(devices_file: Path = _DEVICES_FILE) -> dict[str, str]:
    """Input device name to its event node, from the kernel's device list."""
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


class EvdevJoyConBackend:
    """Both Joy-Cons through the kernel driver's event devices, no library in between."""

    def __init__(self, config: JoyConTeleopModuleConfig) -> None:
        self._config = config
        self._fds: dict[JoyConSide, int] = {}
        self._down: dict[JoyConSide, set[int]] = {side: set() for side in JOYCON_SIDES}
        self._axes: dict[JoyConSide, dict[int, int]] = {side: {} for side in JOYCON_SIDES}
        self._next_scan = 0.0

    def open(self) -> None:
        self._scan()

    def _scan(self) -> None:
        self._next_scan = time.monotonic() + _RESCAN_PERIOD_S
        paths = input_device_paths()
        for side in JOYCON_SIDES:
            if side in self._fds:
                continue
            wanted = self._config.joystick_name(side)
            # The driver also registers a root-only "(IMU)" device per Joy-Con.
            path = next(
                (p for name, p in paths.items() if wanted in name and "(IMU)" not in name), None
            )
            if path is None:
                continue
            try:
                self._fds[side] = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
            except OSError as exc:
                logger.warning("Joy-Con %s at %s cannot be opened: %s", side, path, exc)
                continue
            self._down[side] = set()
            self._axes[side] = {}
            logger.info("Joy-Con %s connected at %s", side, path)

    def _drop(self, side: JoyConSide) -> None:
        fd = self._fds.pop(side, None)
        if fd is not None:
            os.close(fd)

    def _drain(self, side: JoyConSide) -> bool:
        """Apply every queued event; False when the device is gone."""
        fd = self._fds[side]
        while True:
            try:
                data = os.read(fd, _EVENT_SIZE * 64)
            except BlockingIOError:
                return True
            except OSError:
                self._drop(side)
                return False
            if not data:
                self._drop(side)
                return False
            for offset in range(0, len(data) - len(data) % _EVENT_SIZE, _EVENT_SIZE):
                _, _, kind, code, value = struct.unpack_from(_EVENT_FORMAT, data, offset)
                if kind == _EV_KEY:
                    if value:
                        self._down[side].add(code)
                    else:
                        self._down[side].discard(code)
                elif kind == _EV_ABS:
                    self._axes[side][code] = value

    def read(self) -> dict[JoyConSide, JoyConReading | None]:
        if len(self._fds) < len(JOYCON_SIDES) and time.monotonic() >= self._next_scan:
            self._scan()
        readings: dict[JoyConSide, JoyConReading | None] = {}
        for side in JOYCON_SIDES:
            if side not in self._fds or not self._drain(side):
                readings[side] = None
                continue
            axes = self._config.axis_index(side)
            x = self._axes[side].get(axes["x"], 0) / STICK_FULL_SCALE
            y = self._axes[side].get(axes["y"], 0) / STICK_FULL_SCALE
            down = dict.fromkeys(self._down[side], True)
            readings[side] = JoyConReading(
                pressed=pressed_labels(down, self._config.button_index(side)),
                stick=(max(-1.0, min(1.0, x)), max(-1.0, min(1.0, y))),
            )
        return readings

    def close(self) -> None:
        for side in list(self._fds):
            self._drop(side)


class JoyConTeleopModule(Module):
    """Publish Joy-Con buttons, gripper openings and an optional base twist."""

    config: JoyConTeleopModuleConfig  # type: ignore[assignment]
    teleop_buttons: Out[Buttons]
    left_gripper_command: Out[Float32]
    right_gripper_command: Out[Float32]
    cmd_vel: Out[Twist]
    joycon_sample: Out[Joy]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._backend: JoyConBackend | None = None
        self._state = JoyConState()
        self._last_tick: float | None = None
        self._present: dict[JoyConSide, bool] = dict.fromkeys(JOYCON_SIDES, False)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def joycon_config(self) -> JoyConTeleopModuleConfig:
        return self.config

    def use_backend(self, backend: JoyConBackend) -> None:
        """Install a reading source other than the kernel driver, e.g. in tests."""
        self._backend = backend

    @rpc
    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            logger.warning("Joy-Con polling worker is already running")
            return
        super().start()
        self._stop_event.clear()
        if self._backend is None:
            use_hid = self.config.backend == "hid" or (
                self.config.backend == "auto" and sys.platform == "darwin"
            )
            self._backend = (
                HidJoyConBackend(self.config.hid_report_timeout_s)
                if use_hid
                else EvdevJoyConBackend(self.joycon_config)
            )
        self._backend.open()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(DEFAULT_THREAD_JOIN_TIMEOUT)
            if self._thread.is_alive():
                logger.warning("Joy-Con polling worker did not exit in time")
            else:
                self._thread = None
        if self._backend is not None:
            self._backend.close()
        super().stop()

    def tick(self, now: float | None = None) -> None:
        """Read both Joy-Cons once and publish what changed or is held."""
        if self._backend is None or self._stop_event.is_set():
            return
        now = time.monotonic() if now is None else now
        dt = self.joycon_config.tick_period_s if self._last_tick is None else now - self._last_tick
        self._last_tick = now
        readings = self._backend.read()
        self._note_presence(readings)
        self.joycon_sample.publish(encode_readings(readings["left"], readings["right"]))
        if not self.joycon_config.publish_legacy_commands:
            return
        self.teleop_buttons.publish(buttons_message(readings["left"], readings["right"]))
        config = self.joycon_config
        outputs: dict[JoyConSide, Out[Float32]] = {
            "left": self.left_gripper_command,
            "right": self.right_gripper_command,
        }
        for side in JOYCON_SIDES:
            output = outputs[side]
            reading = readings[side]
            if reading is None:
                continue
            opening = step_gripper(
                self._state.grippers[side],
                GRIPPER_LABEL[side] in reading.pressed,
                mode=config.gripper_mode,
                slew_per_s=config.gripper_slew_per_s,
                dt=dt,
            )
            output.publish(Float32(data=opening))
        if config.publish_cmd_vel:
            vx, vy, wz = stick_to_base_velocity(
                readings["left"],
                deadzone=config.deadzone,
                max_linear_m_s=config.max_linear_m_s,
                max_angular_rad_s=config.max_angular_rad_s,
            )
            self.cmd_vel.publish(
                Twist(linear=Vector3(x=vx, y=vy, z=0.0), angular=Vector3(x=0.0, y=0.0, z=wz))
            )

    def _note_presence(self, readings: Mapping[JoyConSide, JoyConReading | None]) -> None:
        for side in JOYCON_SIDES:
            present = readings[side] is not None
            if present != self._present[side]:
                self._present[side] = present
                if not present:
                    logger.warning("Joy-Con %s dropped; holding its last gripper opening", side)

    def _run_loop(self) -> None:
        next_tick = time.monotonic()
        while not self._stop_event.is_set():
            try:
                self.tick()
            except Exception:
                logger.exception("Unexpected Joy-Con polling worker error")
            next_tick += self.joycon_config.tick_period_s
            self._stop_event.wait(max(0.0, next_tick - time.monotonic()))

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

Each Joy-Con pairs over Bluetooth and the kernel's hid-nintendo driver shows
it as its own gamepad; pygame reads both. Nothing here is specific to a
robot: the module publishes the same streams the WebXR and keyboard teleop
modules do, so the episode monitor and the gripper tasks consume it as is.
"""

from __future__ import annotations

from collections.abc import Mapping
import os
import threading
import time
from typing import Any, Protocol

from pydantic import Field

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.teleop.joycon.mapping import (
    DEFAULT_AXIS_INDEX,
    DEFAULT_BUTTON_INDEX,
    GRIPPER_LABEL,
    JOYCON_SIDES,
    GripperMode,
    JoyConReading,
    JoyConSide,
    JoyConState,
    buttons_message,
    pressed_labels,
    step_gripper,
    stick_to_base_velocity,
)
from dimos.teleop.webxr.controller_types import Buttons
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class JoyConBackend(Protocol):
    """Where the readings come from: pygame on a machine, a fake in tests."""

    def open(self) -> None: ...
    def read(self) -> dict[JoyConSide, JoyConReading | None]: ...
    def close(self) -> None: ...


class JoyConTeleopModuleConfig(ModuleConfig):
    tick_period_s: float = Field(default=0.02, gt=0.0)
    # Substring of the SDL joystick name that identifies each side.
    left_name: str = "Left Joy-Con"
    right_name: str = "Right Joy-Con"
    # SDL button and axis indices per side; run `python -m dimos.teleop.joycon.probe`
    # and override these when a driver orders them differently.
    left_buttons: dict[str, int] = Field(default_factory=lambda: dict(DEFAULT_BUTTON_INDEX["left"]))
    right_buttons: dict[str, int] = Field(
        default_factory=lambda: dict(DEFAULT_BUTTON_INDEX["right"])
    )
    left_axes: dict[str, int] = Field(default_factory=lambda: dict(DEFAULT_AXIS_INDEX))
    right_axes: dict[str, int] = Field(default_factory=lambda: dict(DEFAULT_AXIS_INDEX))
    # hold: closed while ZL/ZR is held. toggle: a press flips open/closed.
    gripper_mode: GripperMode = "hold"
    # The published opening moves at most this much per second (1 = full travel).
    gripper_slew_per_s: float = Field(default=3.0, gt=0.0)
    deadzone: float = Field(default=0.15, ge=0.0, lt=1.0)
    # Left stick drives a base through cmd_vel only when this is on.
    publish_cmd_vel: bool = False
    max_linear_m_s: float = Field(default=0.3, gt=0.0)
    max_angular_rad_s: float = Field(default=0.6, gt=0.0)

    def button_index(self, side: JoyConSide) -> dict[str, int]:
        return self.left_buttons if side == "left" else self.right_buttons

    def axis_index(self, side: JoyConSide) -> dict[str, int]:
        return self.left_axes if side == "left" else self.right_axes

    def joystick_name(self, side: JoyConSide) -> str:
        return self.left_name if side == "left" else self.right_name


class PygameJoyConBackend:
    """Both Joy-Cons through pygame's joystick API on the kernel driver."""

    def __init__(self, config: JoyConTeleopModuleConfig) -> None:
        self._config = config
        self._pygame: Any = None
        self._sticks: dict[JoyConSide, Any] = {}

    def open(self) -> None:
        # The kernel driver is the one reader; SDL's own HID backend would fight it.
        os.environ.setdefault("SDL_JOYSTICK_HIDAPI", "0")
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
        try:
            import pygame
        except ImportError as exc:
            raise RuntimeError(
                "Joy-Con teleop needs pygame; install the manipulation extra"
            ) from exc
        pg: Any = pygame
        self._pygame = pg
        pg.init()
        pg.joystick.init()
        self._rescan()

    def _rescan(self) -> None:
        pygame = self._pygame
        found: dict[JoyConSide, Any] = {}
        for index in range(pygame.joystick.get_count()):
            stick = pygame.joystick.Joystick(index)
            name = stick.get_name()
            for side in JOYCON_SIDES:
                if side not in found and self._config.joystick_name(side) in name:
                    stick.init()
                    found[side] = stick
        for side in JOYCON_SIDES:
            if (side in found) != (side in self._sticks):
                logger.info("Joy-Con %s %s", side, "connected" if side in found else "not present")
        self._sticks = found

    def read(self) -> dict[JoyConSide, JoyConReading | None]:
        pygame = self._pygame
        rescan = False
        for event in pygame.event.get():
            if event.type in (pygame.JOYDEVICEADDED, pygame.JOYDEVICEREMOVED):
                rescan = True
        if rescan:
            self._rescan()
        readings: dict[JoyConSide, JoyConReading | None] = {}
        for side in JOYCON_SIDES:
            stick = self._sticks.get(side)
            if stick is None:
                readings[side] = None
                continue
            try:
                down = {i: bool(stick.get_button(i)) for i in range(stick.get_numbuttons())}
                axes = self._config.axis_index(side)
                count = stick.get_numaxes()
                x = float(stick.get_axis(axes["x"])) if axes["x"] < count else 0.0
                y = float(stick.get_axis(axes["y"])) if axes["y"] < count else 0.0
            except pygame.error as exc:
                logger.warning("Joy-Con %s read failed: %s", side, exc)
                readings[side] = None
                continue
            readings[side] = JoyConReading(
                pressed=pressed_labels(down, self._config.button_index(side)), stick=(x, y)
            )
        return readings

    def close(self) -> None:
        if self._pygame is not None:
            self._pygame.joystick.quit()
            self._pygame.quit()
        self._sticks = {}


class JoyConTeleopModule(Module):
    """Publish Joy-Con buttons, gripper openings and an optional base twist."""

    config: JoyConTeleopModuleConfig  # type: ignore[assignment]
    teleop_buttons: Out[Buttons]
    left_gripper_command: Out[Float32]
    right_gripper_command: Out[Float32]
    cmd_vel: Out[Twist]

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
        """Install a reading source other than pygame, e.g. in tests."""
        self._backend = backend

    @rpc
    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            logger.warning("Joy-Con polling worker is already running")
            return
        super().start()
        self._stop_event.clear()
        if self._backend is None:
            self._backend = PygameJoyConBackend(self.joycon_config)
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

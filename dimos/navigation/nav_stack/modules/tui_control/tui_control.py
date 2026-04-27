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

"""TUIControlModule: terminal-based teleop controller.

Provides arrow-key control for the vehicle and mode switching.
"""

from __future__ import annotations

import threading
import time
from typing import Any

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Twist import Twist


class TUIControlConfig(ModuleConfig):
    """Configuration for the TUI controller."""

    max_speed: float = 2.0
    max_yaw_rate: float = 1.5
    speed_step: float = 0.1
    publish_rate: float = 20.0  # Hz


class TUIControlModule(Module):
    """Terminal-based teleop controller with arrow key input.

    Ports:
        cmd_vel (Out[Twist]): Velocity commands from keyboard.
        way_point (Out[PointStamped]): Waypoint commands (typed coordinates).
    """

    config: TUIControlConfig

    cmd_vel: Out[Twist]
    way_point: Out[PointStamped]

    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._fwd = 0.0
        self._left = 0.0
        self._yaw = 0.0
        self._speed_scale = 1.0
        self._running = False
        self._publish_thread: threading.Thread | None = None
        self._input_thread: threading.Thread | None = None

    def __getstate__(self) -> dict[str, Any]:
        state: dict[str, Any] = super().__getstate__()  # type: ignore[no-untyped-call]
        state.pop("_lock", None)
        state.pop("_publish_thread", None)
        state.pop("_input_thread", None)
        return state

    def __setstate__(self, state: dict[str, Any]) -> None:
        super().__setstate__(state)
        self._lock = threading.Lock()
        self._publish_thread = None
        self._input_thread = None

    @rpc
    def start(self) -> None:
        self._running = True
        self._publish_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._publish_thread.start()
        self._input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self._input_thread.start()

    @rpc
    def stop(self) -> None:
        self._running = False
        if self._publish_thread:
            self._publish_thread.join(timeout=2.0)
        super().stop()

    def _publish_loop(self) -> None:
        """Publish current velocity at fixed rate."""
        dt = 1.0 / self.config.publish_rate
        while self._running:
            with self._lock:
                fwd = self._fwd
                left = self._left
                yaw = self._yaw
                scale = self._speed_scale
            twist = Twist(
                linear=[
                    fwd * scale * self.config.max_speed,
                    left * scale * self.config.max_speed,
                    0.0,
                ],
                angular=[
                    0.0,
                    0.0,
                    yaw * scale * self.config.max_yaw_rate,
                ],
            )
            self.cmd_vel.publish(twist)
            time.sleep(dt)

    def _input_loop(self) -> None:
        """Read keyboard input for teleop control.

        Controls:
            w/up: forward, s/down: backward
            a/left: strafe left, d/right: strafe right
            q: rotate left, e: rotate right
            +/-: increase/decrease speed
            space: stop
            Ctrl+C: quit
        """
        try:
            import sys
            import termios
            import tty

            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)

            print("\n--- SmartNav TUI Controller ---")
            print("w/s: fwd/back | a/d: strafe | q/e: rotate")
            print("+/-: speed    | g: waypoint | space: stop")
            print("Ctrl+C: quit")
            print("-------------------------------\n")

            try:
                tty.setraw(fd)
                while self._running:
                    ch = sys.stdin.read(1)
                    if ch == "\x03":  # Ctrl+C
                        self._running = False
                        break
                    self._handle_key(ch)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        except Exception:
            # Not a terminal (e.g., running in a worker process, piped stdin, etc.)
            while self._running:
                time.sleep(1.0)

    def _handle_key(self, ch: str) -> None:
        """Process a single keypress."""
        with self._lock:
            if ch in ("w", "W"):
                self._fwd = 1.0
                self._left = 0.0
                self._yaw = 0.0
            elif ch in ("s", "S"):
                self._fwd = -1.0
                self._left = 0.0
                self._yaw = 0.0
            elif ch in ("a", "A"):
                self._fwd = 0.0
                self._left = 1.0
                self._yaw = 0.0
            elif ch in ("d", "D"):
                self._fwd = 0.0
                self._left = -1.0
                self._yaw = 0.0
            elif ch in ("q", "Q"):
                self._fwd = 0.0
                self._left = 0.0
                self._yaw = 1.0
            elif ch in ("e", "E"):
                self._fwd = 0.0
                self._left = 0.0
                self._yaw = -1.0
            elif ch == " ":
                self._fwd = 0.0
                self._left = 0.0
                self._yaw = 0.0
            elif ch == "+" or ch == "=":
                self._speed_scale = min(self._speed_scale + 0.1, 1.0)
            elif ch == "-":
                self._speed_scale = max(self._speed_scale - 0.1, 0.1)
        if ch == "\x1b":
            import sys

            seq1 = sys.stdin.read(1)
            if seq1 == "[":
                seq2 = sys.stdin.read(1)
                with self._lock:
                    if seq2 == "A":  # Up
                        self._fwd = 1.0
                        self._left = 0.0
                        self._yaw = 0.0
                    elif seq2 == "B":  # Down
                        self._fwd = -1.0
                        self._left = 0.0
                        self._yaw = 0.0
                    elif seq2 == "C":  # Right
                        self._fwd = 0.0
                        self._left = -1.0
                        self._yaw = 0.0
                    elif seq2 == "D":  # Left
                        self._fwd = 0.0
                        self._left = 1.0
                        self._yaw = 0.0

    def send_waypoint(self, x: float, y: float, z: float = 0.0) -> None:
        """Programmatically send a waypoint."""
        wp = PointStamped(x=x, y=y, z=z, frame_id="map")
        self.way_point.publish(wp)

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

"""Keyboard-based EEF twist teleop module for arm teleoperation.

Wraps a pygame UI as a DimOS Module so it can be composed with coordinator
blueprints via autoconnect.

Keyboard controls:
    W/S: +X/-X (forward/backward)
    A/D: +Y/-Y (left/right)
    Q/E: +Z/-Z (up/down)
    R/F: +Roll/-Roll
    T/G: +Pitch/-Pitch
    Y/H: +Yaw/-Yaw
    [: Open gripper
    ]: Close gripper
    ESC: Quit
"""

from __future__ import annotations

from multiprocessing import Pipe
from multiprocessing.connection import Connection
import subprocess
import sys
import threading
from typing import Any

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.teleop.keyboard import keyboard_window
from dimos.teleop.keyboard.keyboard_window import (
    GripperCommand,
    KeyboardWindow,
    Shutdown,
    TwistCommand,
    TwistVector,
    WindowError,
    WindowMessage,
    WindowReady,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DEFAULT_LINEAR_SPEED = 0.05  # m/s
DEFAULT_ANGULAR_SPEED = 0.5  # rad/s
WINDOW_START_TIMEOUT = 10.0


class KeyboardTeleopConfig(ModuleConfig):
    linear_speed: float = DEFAULT_LINEAR_SPEED
    angular_speed: float = DEFAULT_ANGULAR_SPEED


class KeyboardTeleopModule(Module):
    """Publish keyboard EEF twist and normalized gripper commands."""

    config: KeyboardTeleopConfig

    ee_twist_command: Out[TwistStamped]
    gripper_command: Out[Float32]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop_event = threading.Event()
        self._lifecycle_lock = threading.RLock()
        self._thread: threading.Thread | None = None
        self._window: subprocess.Popen[bytes] | None = None
        self._connection: Connection | None = None
        self._gripper_opening: float | None = None

    @rpc
    def start(self) -> None:
        with self._lifecycle_lock:
            if self._thread is not None:
                return
            if keyboard_window.pygame is None:
                raise ImportError(
                    "pygame is required for keyboard teleop. Install it with: pip install pygame"
                )
            super().start()
            self._stop_event.clear()
            self._gripper_opening = None
            try:
                if sys.platform == "darwin":
                    self._start_window_process()
                    target = self._publish_window_commands
                else:
                    window = KeyboardWindow(
                        self.config.linear_speed,
                        self.config.angular_speed,
                        self._publish_message,
                        self._stop_event.is_set,
                    )
                    target = window.run
                self._thread = threading.Thread(target=target, daemon=True)
                self._thread.start()
            except Exception:
                self.stop()
                raise

    def _start_window_process(self) -> None:
        parent, child = Pipe()
        self._connection = parent
        try:
            # DimOS workers are daemonic, so multiprocessing.Process cannot start here.
            # Only the dedicated IPC descriptor crosses this executable boundary.
            self._window = subprocess.Popen(
                [
                    sys.executable,
                    "-m",
                    "dimos.teleop.keyboard.keyboard_window_worker",
                    "--connection-fd",
                    str(child.fileno()),
                    "--linear-speed",
                    str(self.config.linear_speed),
                    "--angular-speed",
                    str(self.config.angular_speed),
                ],
                pass_fds=(child.fileno(),),
            )
        finally:
            child.close()
        if not parent.poll(WINDOW_START_TIMEOUT):
            raise TimeoutError("Keyboard teleop window did not start")
        try:
            message = parent.recv()
        except EOFError as exc:
            raise RuntimeError("Keyboard teleop window exited before startup") from exc
        if isinstance(message, WindowError):
            raise RuntimeError(f"Keyboard teleop window failed: {message.detail}")
        if not isinstance(message, WindowReady):
            raise RuntimeError(f"Unexpected keyboard teleop startup message: {message!r}")

    @rpc
    def stop(self) -> None:
        with self._lifecycle_lock:
            self._stop_event.set()
            if self._connection is not None:
                try:
                    self._connection.send(Shutdown())
                except OSError:
                    pass
            if self._window is not None:
                try:
                    self._window.wait(DEFAULT_THREAD_JOIN_TIMEOUT)
                except subprocess.TimeoutExpired:
                    self._window.terminate()
                    try:
                        self._window.wait(DEFAULT_THREAD_JOIN_TIMEOUT)
                    except subprocess.TimeoutExpired:
                        self._window.kill()
                        self._window.wait()
                self._window = None
            if self._thread is not None:
                self._thread.join(DEFAULT_THREAD_JOIN_TIMEOUT)
                self._thread = None
            if self._connection is not None:
                self._connection.close()
                self._connection = None
            super().stop()

    def _publish_window_commands(self) -> None:
        connection = self._connection
        assert connection is not None
        try:
            while True:
                message = connection.recv()
                if self._stop_event.is_set():
                    continue
                self._publish_message(message)
        except (EOFError, ConnectionResetError):
            pass
        finally:
            self._publish_twist()

    def _publish_message(self, message: WindowMessage) -> None:
        match message:
            case TwistCommand(linear=linear, angular=angular):
                self._publish_twist(linear=linear, angular=angular)
            case GripperCommand(opening=opening):
                self._publish_gripper_command(opening=opening)
            case WindowError(detail=detail):
                logger.error("Keyboard teleop window failed", detail=detail)

    def _publish_twist(
        self,
        *,
        linear: TwistVector = (0.0, 0.0, 0.0),
        angular: TwistVector = (0.0, 0.0, 0.0),
    ) -> None:
        self.ee_twist_command.publish(TwistStamped(linear=list(linear), angular=list(angular)))

    def _publish_gripper_command(self, *, opening: float) -> None:
        """Publish a changed normalized gripper opening."""
        if self._gripper_opening == opening:
            return
        self._gripper_opening = opening
        self.gripper_command.publish(Float32(data=opening))

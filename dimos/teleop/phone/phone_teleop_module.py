#!/usr/bin/env python3
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

"""
Phone Teleoperation Module.

Receives raw sensor data (TwistStamped) and button state (Bool) from the
phone web app via the Deno LCM bridge.  Computes orientation deltas from
a initial orientation captured on engage, converts to TwistStamped velocity
commands via configurable gains, and publishes.

"""

from dataclasses import dataclass
from pathlib import Path
import shutil
import signal
import subprocess
import threading
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.core import In, Module, Out, rpc
from dimos.core.module import ModuleConfig
from dimos.msgs.geometry_msgs import Twist, TwistStamped, Vector3
from dimos.msgs.std_msgs.Bool import Bool
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class PhoneTeleopConfig(ModuleConfig):
    control_loop_hz: float = 50.0
    linear_gain: float = 1.0 / 30.0  # Gain: maps degrees of tilt to m/s. 30 deg -> 1.0 m/s
    angular_gain: float = 1.0 / 30.0  # Gain: maps gyro deg/s to rad/s. 30 deg/s -> 1.0 rad/s


class PhoneTeleopModule(Module[PhoneTeleopConfig]):
    """
    Receives raw sensor data from the phone web app:
      - TwistStamped: linear=(roll, pitch, yaw) deg, angular=(gyro) deg/s
      - Bool: teleop button state (True = held)

    Outputs:
        - twist_output: TwistStamped (velocity command for robot)
    """

    default_config = PhoneTeleopConfig

    # Inputs from Deno bridge
    phone_sensors: In[TwistStamped]
    phone_button: In[Bool]
    # Output: velocity command to robot
    twist_output: Out[TwistStamped]

    # -------------------------------------------------------------------------
    # Initialization
    # -------------------------------------------------------------------------

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)

        self._is_engaged: bool = False
        self._teleop_button: bool = False
        self._current_sensors: TwistStamped | None = None
        self._initial_sensors: TwistStamped | None = None
        self._lock = threading.RLock()

        # Control loop
        self._control_loop_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

        # Deno bridge server
        self._server_process: subprocess.Popen[bytes] | None = None
        self._server_script = Path(__file__).parent / "web" / "teleop_server.ts"

    # -------------------------------------------------------------------------
    # Lifecycle
    # -------------------------------------------------------------------------

    @rpc
    def start(self) -> None:
        super().start()
        for stream, handler in (
            (self.phone_sensors, self._on_sensors),
            (self.phone_button, self._on_button),
        ):
            self._disposables.add(Disposable(stream.subscribe(handler)))  # type: ignore[attr-defined]
        self._start_server()
        self._start_control_loop()

    @rpc
    def stop(self) -> None:
        self._stop_control_loop()
        self._stop_server()
        super().stop()

    # -------------------------------------------------------------------------
    # Internal engage / disengage (assumes lock is held)
    # -------------------------------------------------------------------------

    def _engage(self) -> bool:
        """Engage: capture current sensors as initial"""
        if self._current_sensors is None:
            logger.error("Engage failed: no sensor data yet")
            return False
        self._initial_sensors = self._current_sensors
        self._is_engaged = True
        logger.info("Phone teleop engaged")
        return True

    def _disengage(self) -> None:
        """Disengage: stop publishing"""
        self._is_engaged = False
        self._initial_sensors = None
        logger.info("Phone teleop disengaged")

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------

    def _on_sensors(self, msg: TwistStamped) -> None:
        """Callback for raw sensor TwistStamped from the phone"""
        with self._lock:
            self._current_sensors = msg

    def _on_button(self, msg: Bool) -> None:
        """Callback for teleop button state."""
        with self._lock:
            self._teleop_button = bool(msg.data)

    # -------------------------------------------------------------------------
    # Deno Bridge Server
    # -------------------------------------------------------------------------

    def _start_server(self) -> None:
        """Launch the Deno WebSocket-to-LCM bridge server as a subprocess."""
        if self._server_process is not None and self._server_process.poll() is None:
            logger.warning("Deno bridge already running", pid=self._server_process.pid)
            return

        if shutil.which("deno") is None:
            logger.error(
                "Deno is not installed. Install it with: curl -fsSL https://deno.land/install.sh | sh"
            )
            return

        script = str(self._server_script)
        cmd = [
            "deno",
            "run",
            "--allow-net",
            "--allow-read",
            "--allow-run",
            "--allow-write",
            "--unstable-net",
            script,
        ]
        try:
            self._server_process = subprocess.Popen(cmd)
            logger.info(f"Deno bridge server started (pid {self._server_process.pid})")
        except OSError as e:
            logger.error(f"Failed to start Deno bridge: {e}")

    def _stop_server(self) -> None:
        """Terminate the Deno bridge server subprocess."""
        if self._server_process is None or self._server_process.poll() is not None:
            self._server_process = None
            return

        logger.info("Stopping Deno bridge server", pid=self._server_process.pid)
        self._server_process.send_signal(signal.SIGTERM)
        try:
            self._server_process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            logger.warning(
                "Deno bridge did not exit, sending SIGKILL", pid=self._server_process.pid
            )
            self._server_process.kill()
            try:
                self._server_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                logger.error("Deno bridge did not exit after SIGKILL")
        logger.info("Deno bridge server stopped")
        self._server_process = None

    # -------------------------------------------------------------------------
    # Control Loop
    # -------------------------------------------------------------------------

    def _start_control_loop(self) -> None:
        if self._control_loop_thread is not None and self._control_loop_thread.is_alive():
            return

        self._stop_event.clear()
        self._control_loop_thread = threading.Thread(
            target=self._control_loop,
            daemon=True,
            name="PhoneTeleopControlLoop",
        )
        self._control_loop_thread.start()
        logger.info(f"Control loop started at {self.config.control_loop_hz} Hz")

    def _stop_control_loop(self) -> None:
        self._stop_event.set()
        if self._control_loop_thread is not None:
            self._control_loop_thread.join(timeout=1.0)
            self._control_loop_thread = None
        logger.info("Control loop stopped")

    def _control_loop(self) -> None:
        period = 1.0 / self.config.control_loop_hz

        while not self._stop_event.is_set():
            loop_start = time.perf_counter()
            with self._lock:
                self._handle_engage()

                if self._is_engaged:
                    output_twist = self._get_output_twist()
                    if output_twist is not None:
                        self._publish_msg(output_twist)

            elapsed = time.perf_counter() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                self._stop_event.wait(sleep_time)

    # -------------------------------------------------------------------------
    # Control Loop Internal Methods
    # -------------------------------------------------------------------------

    def _handle_engage(self) -> None:
        """
        Override to customize engagement logic.
        Default: button hold = engaged, release = disengaged.
        """
        if self._teleop_button:
            if not self._is_engaged:
                self._engage()
        else:
            if self._is_engaged:
                self._disengage()

    def _get_output_twist(self) -> TwistStamped | None:
        """Compute twist from orientation delta.
        Override to customize twist computation (e.g., apply scaling, filtering).
        Default: Computes delta angles from initial orientation, applies gains.
        """
        current = self._current_sensors
        initial = self._initial_sensors
        if current is None or initial is None:
            return None

        delta: Twist = Twist(current) - Twist(initial)

        # Handle yaw wraparound (linear.z = yaw, 0-360 degrees)
        d_yaw = delta.linear.z
        if d_yaw > 180:
            d_yaw -= 360
        elif d_yaw < -180:
            d_yaw += 360

        cfg = self.config
        return TwistStamped(
            ts=current.ts,
            frame_id="phone",
            linear=Vector3(
                x=-delta.linear.y * cfg.linear_gain,  # pitch forward -> drive forward
                y=-delta.linear.x * cfg.linear_gain,  # roll right -> strafe right
                z=d_yaw * cfg.linear_gain,  # yaw delta
            ),
            angular=Vector3(
                x=current.angular.x * cfg.angular_gain,
                y=current.angular.y * cfg.angular_gain,
                z=current.angular.z * cfg.angular_gain,
            ),
        )

    def _publish_msg(self, output_msg: TwistStamped) -> None:
        """
        Override to customize output (e.g., apply limits, remap axes).
        """
        self.twist_output.publish(output_msg)


phone_teleop_module = PhoneTeleopModule.blueprint

__all__ = [
    "PhoneTeleopConfig",
    "PhoneTeleopModule",
    "phone_teleop_module",
]

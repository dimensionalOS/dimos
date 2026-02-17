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
a home orientation captured on engage, converts to TwistStamped velocity
commands via configurable gains, and publishes.

Raw sensor TwistStamped layout from browser:
    linear  = (roll, pitch, yaw)  in degrees   (DeviceOrientation)
    angular = (gyro_x, gyro_y, gyro_z) in deg/s (DeviceMotion)
"""

from dataclasses import dataclass
from pathlib import Path
import subprocess
import threading
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.core import In, Module, Out, rpc
from dimos.core.module import ModuleConfig
from dimos.msgs.geometry_msgs import TwistStamped, Vector3
from dimos.msgs.std_msgs.Bool import Bool
from dimos.teleop.base import TeleopProtocol
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class PhoneTeleopConfig(ModuleConfig):
    """Configuration for Phone Teleoperation Module."""

    control_loop_hz: float = 50.0

    # Gain: maps degrees of tilt to m/s.  30 deg tilt -> 1.0 m/s
    linear_gain: float = 1.0 / 30.0
    # Gain: maps degrees of yaw delta to rad/s.  30 deg -> 1.0 rad/s
    angular_gain: float = 1.0 / 30.0


class PhoneTeleopModule(Module[PhoneTeleopConfig], TeleopProtocol):
    """Phone Teleoperation Module.

    Receives raw sensor data from the phone web app:
      - TwistStamped: linear=(roll, pitch, yaw) deg, angular=(gyro) deg/s
      - Bool: teleop button state (True = held)

    On engage (button pressed), captures the current orientation as home.
    Each control-loop tick computes the orientation delta from home,
    converts it to a TwistStamped via configurable gains, and publishes.

    Implements TeleopProtocol.

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

        # Raw sensor messages (like Quest stores PoseStamped)
        self._current_sensors: TwistStamped | None = None
        self._home_sensors: TwistStamped | None = None

        self._lock = threading.RLock()

        # Control loop
        self._control_loop_thread: threading.Thread | None = None
        self._control_loop_running = False

        # Deno bridge server
        self._server_process: subprocess.Popen | None = None
        self._server_script = Path(__file__).parent / "web" / "teleop_server.ts"

        logger.info("PhoneTeleopModule initialized")

    # -------------------------------------------------------------------------
    # Public RPC Methods
    # -------------------------------------------------------------------------

    @rpc
    def start(self) -> None:
        """Start the phone teleoperation module."""
        super().start()

        input_streams = {
            "phone_sensors": (self.phone_sensors, self._on_sensors),
            "phone_button": (self.phone_button, self._on_button),
        }
        connected = []
        for name, (stream, handler) in input_streams.items():
            if not (stream and stream.transport):  # type: ignore[attr-defined]
                logger.warning(f"Stream '{name}' has no transport — skipping")
                continue
            self._disposables.add(Disposable(stream.subscribe(handler)))  # type: ignore[attr-defined]
            connected.append(name)

        if connected:
            logger.info(f"Subscribed to: {', '.join(connected)}")

        self._start_server()
        self._start_control_loop()
        logger.info("Phone Teleoperation Module started")

    @rpc
    def stop(self) -> None:
        """Stop the phone teleoperation module."""
        logger.info("Stopping Phone Teleoperation Module...")
        self._stop_control_loop()
        self._stop_server()
        super().stop()

    @rpc
    def engage(self, hand: Any = None) -> bool:
        """Engage teleoperation."""
        with self._lock:
            return self._engage()

    @rpc
    def disengage(self, hand: Any = None) -> None:
        """Disengage teleoperation."""
        with self._lock:
            self._disengage()

    # -------------------------------------------------------------------------
    # Internal engage / disengage (assumes lock is held)
    # -------------------------------------------------------------------------

    def _engage(self) -> bool:
        """Engage: capture current sensors as home. Assumes lock held."""
        if self._current_sensors is None:
            logger.error("Engage failed: no sensor data yet")
            return False
        self._home_sensors = self._current_sensors
        self._is_engaged = True
        logger.info("Phone teleop engaged")
        return True

    def _disengage(self) -> None:
        """Disengage: stop publishing. Assumes lock held."""
        self._is_engaged = False
        self._home_sensors = None
        logger.info("Phone teleop disengaged")

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------

    def _on_sensors(self, msg: TwistStamped) -> None:
        """Callback for raw sensor TwistStamped from the phone web app.

        linear = (roll, pitch, yaw) in degrees
        angular = (gyro_x, gyro_y, gyro_z) in deg/s
        """
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
        if self._server_process is not None:
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
        except FileNotFoundError:
            logger.error("'deno' not found in PATH — install Deno or start the server manually")

    def _stop_server(self) -> None:
        """Terminate the Deno bridge server subprocess."""
        if self._server_process is None:
            return

        self._server_process.terminate()
        try:
            self._server_process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self._server_process.kill()
            self._server_process.wait(timeout=1)
        logger.info("Deno bridge server stopped")
        self._server_process = None

    # -------------------------------------------------------------------------
    # Control Loop
    # -------------------------------------------------------------------------

    def _start_control_loop(self) -> None:
        """Start the control loop thread."""
        if self._control_loop_running:
            return

        self._control_loop_running = True
        self._control_loop_thread = threading.Thread(
            target=self._control_loop,
            daemon=True,
            name="PhoneTeleopControlLoop",
        )
        self._control_loop_thread.start()
        logger.info(f"Control loop started at {self.config.control_loop_hz} Hz")

    def _stop_control_loop(self) -> None:
        """Stop the control loop thread."""
        self._control_loop_running = False
        if self._control_loop_thread is not None:
            self._control_loop_thread.join(timeout=1.0)
            self._control_loop_thread = None
        logger.info("Control loop stopped")

    def _control_loop(self) -> None:
        """Main control loop: handle engagement, compute deltas, publish twist.

        Holds self._lock for the entire iteration so overridable methods
        don't need to acquire it themselves.
        """
        period = 1.0 / self.config.control_loop_hz

        while self._control_loop_running:
            loop_start = time.perf_counter()
            try:
                with self._lock:
                    self._handle_engage()

                    if self._should_publish():
                        output_twist = self._get_output_twist()
                        if output_twist is not None:
                            self._publish_msg(output_twist)
            except Exception:
                logger.exception("Error in phone teleop control loop")

            elapsed = time.perf_counter() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    # -------------------------------------------------------------------------
    # Control Loop Internal Methods
    # -------------------------------------------------------------------------

    def _handle_engage(self) -> None:
        """Check button state and engage/disengage accordingly.

        Override to customize engagement logic.
        Default: button hold = engaged, release = disengaged.
        """
        if self._teleop_button:
            if not self._is_engaged:
                self._engage()
        else:
            if self._is_engaged:
                self._disengage()

    def _should_publish(self) -> bool:
        """Check if we should publish a twist command.

        Override to add custom conditions.
        Default: Returns True if engaged.
        """
        return self._is_engaged

    def _get_output_twist(self) -> TwistStamped | None:
        """Compute twist from orientation delta.
        Override to customize twist computation (e.g., apply scaling, filtering).
        Default: Computes delta angles from home orientation, applies gains.
        """
        current = self._current_sensors
        home = self._home_sensors
        if current is None or home is None:
            return None

        # Twist subtraction: delta.linear = orientation delta, delta.angular = gyro delta
        delta = current - home

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
                z=d_yaw
                * cfg.linear_gain,  # yaw delta -> linear.z (remapped to angular by extensions)
            ),
            angular=Vector3(
                x=current.angular.x * cfg.angular_gain,
                y=current.angular.y * cfg.angular_gain,
                z=current.angular.z * cfg.angular_gain,
            ),
        )

    def _publish_msg(self, output_msg: TwistStamped) -> None:
        """Publish twist command.

        Override to customize output (e.g., apply limits, remap axes).
        """
        self.twist_output.publish(output_msg)


phone_teleop_module = PhoneTeleopModule.blueprint

__all__ = [
    "PhoneTeleopConfig",
    "PhoneTeleopModule",
    "phone_teleop_module",
]

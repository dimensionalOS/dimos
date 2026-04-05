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
phone web app via an embedded FastAPI WebSocket server.  Computes orientation
deltas from an initial orientation captured on engage, converts to TwistStamped
velocity commands via configurable gains, and publishes.
"""

from pathlib import Path
import threading
import time
from typing import Any

from dimos_lcm.geometry_msgs import TwistStamped as LCMTwistStamped
from dimos_lcm.std_msgs import Bool as LCMBool
from fastapi import WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.std_msgs.Bool import Bool
from dimos.utils.logging_config import setup_logger
from dimos.utils.path_utils import get_project_root
from dimos.web.robot_web_interface import RobotWebInterface

logger = setup_logger()

STATIC_DIR = Path(__file__).parent / "web" / "static"


class PhoneTeleopConfig(ModuleConfig):
    control_loop_hz: float = 50.0
    linear_gain: float = 1.0 / 30.0  # Gain: maps degrees of tilt to m/s. 30 deg -> 1.0 m/s
    angular_gain: float = 1.0 / 30.0  # Gain: maps gyro deg/s to rad/s. 30 deg/s -> 1.0 rad/s
    server_port: int = 8444


class PhoneTeleopModule(Module):
    """
    Receives raw sensor data from the phone web app via an embedded WebSocket server:
      - TwistStamped: linear=(roll, pitch, yaw) deg, angular=(gyro) deg/s
      - Bool: teleop button state (True = held)

    Outputs:
        - twist_output: TwistStamped (velocity command for robot)
    """

    config: PhoneTeleopConfig

    # Output: velocity command to robot
    twist_output: Out[TwistStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        self._is_engaged: bool = False
        self._teleop_button: bool = False
        self._current_sensors: TwistStamped | None = None
        self._initial_sensors: TwistStamped | None = None
        self._lock = threading.RLock()

        # Control loop
        self._control_loop_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

        # Embedded web server — RobotWebInterface provides FastAPI app + run()/shutdown()
        self._web_server = RobotWebInterface(port=self.config.server_port)
        self._web_server_thread: threading.Thread | None = None

        # Fingerprint-based message dispatch table
        self._decoders: dict[bytes, Any] = {
            LCMTwistStamped._get_packed_fingerprint(): self._on_sensors_bytes,
            LCMBool._get_packed_fingerprint(): self._on_button_bytes,
        }

        self._setup_routes()

    def _setup_routes(self) -> None:
        """Register teleop routes on the embedded web server."""

        @self._web_server.app.get("/teleop", response_class=HTMLResponse)
        async def teleop_index() -> HTMLResponse:
            index_path = STATIC_DIR / "index.html"
            return HTMLResponse(content=index_path.read_text())

        if STATIC_DIR.is_dir():
            self._web_server.app.mount(
                "/static", StaticFiles(directory=str(STATIC_DIR)), name="teleop_static"
            )

        @self._web_server.app.websocket("/ws")
        async def websocket_endpoint(ws: WebSocket) -> None:
            await ws.accept()
            logger.info("Phone client connected")
            try:
                while True:
                    data = await ws.receive_bytes()
                    fingerprint = data[:8]
                    decoder = self._decoders.get(fingerprint)
                    if decoder:
                        decoder(data)
                    else:
                        logger.warning(f"Unknown message fingerprint: {fingerprint.hex()}")
            except WebSocketDisconnect:
                logger.info("Phone client disconnected")
            except Exception:
                logger.exception("WebSocket error")

    @rpc
    def start(self) -> None:
        super().start()
        self._start_server()
        self._start_control_loop()

    @rpc
    def stop(self) -> None:
        self._stop_control_loop()
        self._stop_server()
        super().stop()

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

    def _on_sensors_bytes(self, data: bytes) -> None:
        """Decode raw LCM bytes into TwistStamped and update sensor state."""
        msg = TwistStamped.lcm_decode(data)
        with self._lock:
            self._current_sensors = msg

    def _on_button_bytes(self, data: bytes) -> None:
        """Decode raw LCM bytes into Bool and update button state."""
        msg = Bool.lcm_decode(data)
        with self._lock:
            self._teleop_button = bool(msg.data)

    def _start_server(self) -> None:
        """Start the embedded FastAPI server with HTTPS in a daemon thread."""
        if self._web_server_thread is not None and self._web_server_thread.is_alive():
            logger.warning("Web server already running")
            return

        self._web_server_thread = threading.Thread(
            target=self._web_server.run,
            kwargs={"ssl": True, "ssl_certs_dir": get_project_root() / "assets" / "teleop_certs"},
            daemon=True,
            name="PhoneTeleopWebServer",
        )
        self._web_server_thread.start()
        logger.info(f"Phone teleop web server started on https://0.0.0.0:{self.config.server_port}")

    def _stop_server(self) -> None:
        """Shutdown the embedded web server."""
        self._web_server.shutdown()
        if self._web_server_thread is not None:
            self._web_server_thread.join(timeout=3)
            self._web_server_thread = None
        logger.info("Phone teleop web server stopped")

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

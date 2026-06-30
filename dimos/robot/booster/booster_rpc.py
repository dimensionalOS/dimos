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

"""Generic Booster booster-rpc connection (gRPC velocity control + WebSocket camera).

The transport layer for Booster robots, analogous to `unitree_webrtc.py` for
Unitree: it owns the vendor SDK and exposes a non-blocking velocity sink, a camera
stream, and stand/sit mode changes. It is robot-agnostic — both the K1 and the T1
connection Modules build on it. Robot-specific wiring (stream ports, camera
intrinsics, blueprints) lives in each robot's `connection.py`.
"""

import asyncio
from threading import Event, Lock, Thread
import time
from typing import Any

from booster_rpc import (  # type: ignore[import-not-found]
    BoosterConnection,
    RobotMode,
    RpcApiId,
)
import cv2
import numpy as np
from reactivex.observable import Observable
from reactivex.subject import Subject

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()

SEND_HZ = 30.0  # gRPC send rate to the robot, kept under booster-rpc's ~58/sec move ceiling
CMD_VEL_TIMEOUT_S = 0.5  # dead-man: send one zero if no new command arrives within this window
MODE_TRANSITION_SETTLE_S = 3.0  # settle time after each DAMPING/PREPARE/WALKING change


class BoosterRPCConnection:
    """Low-level wrapper around booster-rpc; the Module never touches the SDK directly.

    booster-rpc's ``move`` is a synchronous gRPC call with a ~58/sec ceiling, so a
    high-rate publisher (the 100 Hz coordinator) would back it up. ``move()`` is
    therefore non-blocking: it records the latest command, and ``_sender_loop`` issues
    the gRPC call at ``send_hz``, always sending the latest value (stale ones dropped).
    """

    def __init__(self, ip: str) -> None:
        self._conn = BoosterConnection(ip=ip)
        self._lock = Lock()  # serialize gRPC calls to the connection
        self._loop = asyncio.new_event_loop()
        self._thread: Thread | None = None
        self._video_future: Any = None
        self._cmd_lock = Lock()  # guards _latest and _deadline
        self._latest: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._deadline = 0.0  # command is stale past this monotonic time
        self._sender_thread: Thread | None = None
        self._sender_stop = Event()
        self.cmd_vel_timeout = CMD_VEL_TIMEOUT_S
        self.send_hz = SEND_HZ

    def start(self) -> None:
        self._thread = Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        self._sender_stop.clear()
        self._sender_thread = Thread(target=self._sender_loop, daemon=True)
        self._sender_thread.start()

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def stop(self) -> None:
        self._sender_stop.set()
        if self._sender_thread and self._sender_thread.is_alive():
            self._sender_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self._send(0.0, 0.0, 0.0)  # final stop
        if self._video_future:
            self._video_future.cancel()
        if self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        with self._lock:
            self._conn.close()

    def camera_stream(self) -> Observable[Image]:
        """Camera JPEG frames, decoded into `Image` messages.

        ``stream_video`` is an async coroutine that loops forever invoking a callback
        per JPEG frame; we drive it on the background event loop and push decoded
        frames onto a Subject.
        """
        subject: Subject[Image] = Subject()

        def on_jpeg(jpeg: bytes) -> None:
            arr = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if arr is None:
                return
            subject.on_next(
                Image.from_numpy(arr, format=ImageFormat.BGR, frame_id="camera_optical")
            )

        self._video_future = asyncio.run_coroutine_threadsafe(
            self._conn.stream_video(on_jpeg), self._loop
        )
        return backpressure(subject)

    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        # DimOS Twist (SI, body frame: +x fwd, +y left, +z yaw CCW) -> booster (vx, vy, vyaw).
        now = time.monotonic()
        with self._cmd_lock:
            self._latest = (twist.linear.x, twist.linear.y, twist.angular.z)
            self._deadline = now + (duration if duration > 0 else self.cmd_vel_timeout)
        if duration > 0:
            # Discrete "move for N seconds then stop" (the walk skill): block, then expire.
            time.sleep(duration)
            with self._cmd_lock:
                self._latest = (0.0, 0.0, 0.0)
                self._deadline = time.monotonic()
        return True

    def _sender_loop(self) -> None:
        period = 1.0 / self.send_hz
        was_active = False
        while not self._sender_stop.is_set():
            with self._cmd_lock:
                vx, vy, vyaw = self._latest
                active = time.monotonic() <= self._deadline
            if active:
                self._send(vx, vy, vyaw)
            elif was_active:
                self._send(0.0, 0.0, 0.0)  # one dead-man stop on active->idle, then go quiet
            was_active = active
            self._sender_stop.wait(period)

    def _send(self, vx: float, vy: float, vyaw: float) -> None:
        try:
            with self._lock:
                self._conn.move(vx, vy, vyaw)
        except Exception as e:
            # The robot rejects moves outside a locomotion mode ("Failed to move: code = 100").
            logger.warning("Booster move failed: %s: %s", type(e).__name__, e)

    def standup(self) -> bool:
        """Arm the robot for walking (DAMPING -> PREPARE -> WALKING); no-op if already WALKING.

        Refuses modes outside {WALKING, DAMPING, PREPARE} rather than forcing an unsafe transition.
        """
        with self._lock:
            mode = self._conn.get_mode()
        if mode == RobotMode.WALKING:
            return True
        if mode not in (RobotMode.DAMPING, RobotMode.PREPARE):
            logger.warning("Booster standup: unexpected mode %s; not forcing WALKING", mode)
            return False
        if mode == RobotMode.DAMPING:
            with self._lock:
                self._conn.change_mode(RobotMode.PREPARE)
            logger.info("Booster mode -> PREPARE")
            time.sleep(MODE_TRANSITION_SETTLE_S)
        with self._lock:
            self._conn.change_mode(RobotMode.WALKING)
        logger.info("Booster mode -> WALKING")
        time.sleep(MODE_TRANSITION_SETTLE_S)
        with self._lock:
            return bool(self._conn.get_mode() == RobotMode.WALKING)

    def sit(self) -> bool:
        with self._lock:
            self._conn.call(RpcApiId.ROBOT_LIE_DOWN)
        logger.info("Booster lying down")
        return True

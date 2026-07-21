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
stream, and stand/sit mode changes. It is robot-agnostic, so both the K1 and the T1
connection Modules build on it. Robot-specific wiring (stream ports, camera
intrinsics, blueprints) lives in each robot's `connection.py`.

It owns no event loop and no threads: `run_sender()` and `run_camera()` are
coroutines the connection Module spawns on its own loop.
"""

import asyncio
from threading import Event, Lock
import time

from booster_rpc import (  # type: ignore[import-not-found]
    BoosterConnection,
    RobotMode,
    RpcApiId,
)
import cv2
import grpc  # type: ignore[import-untyped]
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
MODE_TRANSITION_TIMEOUT_S = 15.0  # give up if the robot never reports the requested mode
MODE_REQUEST_RETRY_S = 1.0  # the robot ignores mode requests while mid-motion, so re-issue
MODE_POLL_S = 0.1


class BoosterRPCConnection:
    """Low-level wrapper around booster-rpc. The Module never touches the SDK directly.

    booster-rpc's ``move`` is a synchronous gRPC call with a ~58/sec ceiling, so a
    high-rate publisher (the 100 Hz coordinator) would back it up. ``move()`` is
    therefore non-blocking: it records the latest command, and ``run_sender()`` issues
    the gRPC call at ``send_hz``, always sending the latest value (stale ones dropped).
    """

    def __init__(self, ip: str) -> None:
        self._conn = BoosterConnection(ip=ip)
        self._lock = Lock()  # serialize gRPC calls to the connection
        self._cmd_lock = Lock()  # guards _latest and _deadline
        self._latest: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._deadline = 0.0  # command is stale past this monotonic time
        self._frames: Subject[Image] = Subject()
        self._sender_stop = Event()
        self._sender_done = Event()
        self._sender_done.set()
        self._send_failed = False
        self._send_count = 0
        self.cmd_vel_timeout = CMD_VEL_TIMEOUT_S
        self.send_hz = SEND_HZ
        self.mode_transition_timeout = MODE_TRANSITION_TIMEOUT_S
        self.mode_request_retry = MODE_REQUEST_RETRY_S

    def stop(self) -> None:
        self._sender_stop.set()
        self._sender_done.wait(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self._send(0.0, 0.0, 0.0)
        with self._lock:
            self._conn.close()

    def camera_stream(self) -> Observable[Image]:
        """Camera frames decoded from the robot's JPEG stream (see `run_camera`)."""
        return backpressure(self._frames)

    async def run_camera(self) -> None:
        """Decode the robot's JPEG stream onto `camera_stream()` until cancelled."""

        def on_jpeg(jpeg: bytes) -> None:
            arr = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if arr is None:
                return
            self._frames.on_next(
                Image.from_numpy(arr, format=ImageFormat.BGR, frame_id="camera_optical")
            )

        await self._conn.stream_video(on_jpeg)

    def move(self, twist: Twist) -> bool:
        # DimOS Twist (SI, body frame: +x fwd, +y left, +z yaw CCW) -> booster (vx, vy, vyaw).
        with self._cmd_lock:
            self._latest = (twist.linear.x, twist.linear.y, twist.angular.z)
            self._deadline = time.monotonic() + self.cmd_vel_timeout
        return True

    @property
    def send_failed(self) -> bool:
        """True if the robot rejected the most recent command sent to it."""
        return self._send_failed

    async def run_sender(self) -> None:
        """Issue the latest command at `send_hz` until stopped, with a dead-man stop."""
        period = 1.0 / self.send_hz
        was_active = False
        self._sender_stop.clear()
        self._sender_done.clear()
        try:
            while not self._sender_stop.is_set():
                with self._cmd_lock:
                    vx, vy, vyaw = self._latest
                    active = time.monotonic() <= self._deadline
                if active:
                    await asyncio.to_thread(self._send, vx, vy, vyaw)
                elif was_active:
                    # one dead-man stop on active->idle, then go quiet
                    await asyncio.to_thread(self._send, 0.0, 0.0, 0.0)
                was_active = active
                await asyncio.sleep(period)
        finally:
            self._sender_done.set()

    def _send(self, vx: float, vy: float, vyaw: float) -> None:
        try:
            with self._lock:
                self._conn.move(vx, vy, vyaw)
            self._send_failed = False
        except Exception as e:
            # The robot rejects moves outside a locomotion mode ("Failed to move: code = 100").
            self._send_failed = True
            logger.warning("Booster move failed: %s: %s", type(e).__name__, e)
        finally:
            self._send_count += 1

    def confirm_stop(self, timeout: float = 1.0) -> bool:
        """Block until the queued zero command has been transmitted. False if rejected or unsent."""
        target = self._send_count + 2
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._send_count >= target:
                return not self._send_failed
            time.sleep(1.0 / self.send_hz)
        return False

    def standup(self) -> bool:
        """Arm the robot for walking; no-op if already WALKING.

        Refuses modes outside {WALKING, DAMPING, PREPARE} rather than forcing an unsafe transition.
        """
        mode = self._get_mode()
        if mode is None:
            return False
        if mode == RobotMode.WALKING:
            return True
        if mode not in (RobotMode.DAMPING, RobotMode.PREPARE):
            logger.warning("Booster standup: unexpected mode %s, not forcing WALKING", mode)
            return False
        return self._arm(mode)

    def _arm(self, mode: RobotMode) -> bool:
        """Step the mode transitions to WALKING (DAMPING -> PREPARE -> WALKING)."""
        if mode == RobotMode.DAMPING and not self._change_mode(RobotMode.PREPARE):
            return False
        return self._change_mode(RobotMode.WALKING)

    def _change_mode(self, target: RobotMode) -> bool:
        """Request `target` until the robot reports it.

        The robot silently ignores mode requests while it is still physically completing
        the previous transition, so the request is re-issued while polling.
        """
        start = time.monotonic()
        deadline = start + self.mode_transition_timeout
        while time.monotonic() < deadline:
            try:
                with self._lock:
                    self._conn.change_mode(target)
            except grpc.RpcError as e:
                logger.warning("Booster change_mode(%s) failed: %s", target, e)
            retry_at = min(time.monotonic() + self.mode_request_retry, deadline)
            while time.monotonic() < retry_at:
                if self._get_mode() == target:
                    logger.info(
                        "Booster mode -> %s (confirmed in %.2fs)", target, time.monotonic() - start
                    )
                    return True
                time.sleep(MODE_POLL_S)
        logger.warning(
            "Booster mode %s not reached within %ss", target, self.mode_transition_timeout
        )
        return False

    def _get_mode(self) -> RobotMode | None:
        try:
            with self._lock:
                mode: RobotMode = self._conn.get_mode()
            return mode
        except grpc.RpcError as e:
            logger.warning("Booster get_mode failed: %s", e)
            return None

    def sit(self) -> bool:
        with self._lock:
            self._conn.call(RpcApiId.ROBOT_LIE_DOWN)
        logger.info("Booster lying down")
        return True

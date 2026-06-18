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

"""Booster K1 humanoid connection module (built on the booster-rpc SDK).

Scope: the K1 over booster-rpc exposes a camera (JPEG over WebSocket) and base
velocity control (+ stand/sit mode changes). It has no world-frame odometry or
lidar, so this connection implements only the `Camera` spec — no `odom`/`lidar`/
`pointcloud` ports, and therefore no mapping/navigation tier.
"""

import asyncio
from threading import Event, Lock, Thread
import time
from typing import Any

from booster_rpc import (  # type: ignore[import-untyped]
    BoosterConnection,
    RobotMode,
    RpcApiId,
)
import cv2
import numpy as np
from pydantic import Field
from reactivex.disposable import Disposable
from reactivex.observable import Observable
from reactivex.subject import Subject
import rerun.blueprint as rrb

from dimos.agents.annotation import skill
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.spec.perception import Camera
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()


# --- Step 5: configuration ---------------------------------------------------
class ConnectionConfig(ModuleConfig):
    ip: str = Field(default_factory=lambda m: m["g"].robot_ip)


# --- Step 3: connection backend (keep all booster-rpc protocol details out of the Module) ---
class BoosterRPCConnection:
    """Low-level wrapper around booster-rpc.

    Owns the gRPC connection, a background asyncio loop for the WebSocket video
    stream, and a fixed-rate command sender. The Module talks only to this class,
    never to booster-rpc directly.

    booster-rpc's ``move`` is a *synchronous* request/response gRPC call (~9-35 ms
    each → a ceiling of ~58 moves/sec on this firmware). Calling it directly from a
    high-rate publisher (e.g. the 100 Hz ControlCoordinator) overruns that ceiling
    and backs up. So ``move()`` here is non-blocking — it just records the latest
    command — and a single background thread (`_sender_loop`) issues the actual
    gRPC call at `send_hz`, always sending the *latest* value (stale commands are
    dropped, never queued). This is the request/response analogue of the Go2's
    fire-and-forget WebRTC `move`.
    """

    cmd_vel_timeout = 0.5  # dead-man: send zero if no new command within this window (s)
    send_hz = 30.0         # command rate to the robot — kept under the ~58/sec move ceiling

    def __init__(self, ip: str) -> None:
        self._conn = BoosterConnection(ip=ip)
        self._lock = Lock()  # serialize gRPC calls to the connection
        self._loop = asyncio.new_event_loop()
        self._thread: Thread | None = None
        self._video_future: Any = None
        # latest command state, guarded by _cmd_lock
        self._cmd_lock = Lock()
        self._latest: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._deadline = 0.0  # monotonic time after which the command is considered stale
        self._sender_thread: Thread | None = None
        self._sender_stop = Event()

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
        """JPEG frames from the K1 camera, decoded into `Image` messages.

        ``booster_rpc.stream_video`` is an async coroutine that loops forever
        invoking a callback per JPEG frame; we drive it on the background event
        loop and push decoded frames onto a Subject (the doc's async->Observable
        bridge for streams).
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
        # Non-blocking: record the latest command; `_sender_loop` does the actual
        # gRPC send at `send_hz`. Standard DimOS Twist (SI, robot body frame):
        # linear.x forward, linear.y left (m/s), angular.z yaw CCW (rad/s) — the
        # same (vx, vy, vyaw) convention booster-rpc uses.
        now = time.monotonic()
        with self._cmd_lock:
            self._latest = (twist.linear.x, twist.linear.y, twist.angular.z)
            self._deadline = now + (duration if duration > 0 else self.cmd_vel_timeout)
        if duration > 0:
            # Discrete "move for N seconds then stop" (e.g. the walk skill): block
            # the caller for the duration, then let the command go stale.
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
                self._send(0.0, 0.0, 0.0)  # one stop on active->idle (dead-man), then go quiet
            was_active = active
            self._sender_stop.wait(period)

    def _send(self, vx: float, vy: float, vyaw: float) -> None:
        try:
            with self._lock:
                self._conn.move(vx, vy, vyaw)
        except Exception as e:
            # The robot rejects moves when it isn't in a locomotion mode (e.g. it
            # left WALKING) — "Failed to move: code = 100". Log and keep going.
            logger.warning("K1 move failed: %s: %s", type(e).__name__, e)

    def standup(self) -> bool:
        """Arm the robot for walking: DAMPING -> PREPARE -> WALKING."""
        with self._lock:
            mode = self._conn.get_mode()
        if mode == RobotMode.WALKING:
            return True
        if mode == RobotMode.DAMPING:
            with self._lock:
                self._conn.change_mode(RobotMode.PREPARE)
            logger.info("K1 mode -> PREPARE")
            time.sleep(3)
        with self._lock:
            self._conn.change_mode(RobotMode.WALKING)
        logger.info("K1 mode -> WALKING")
        time.sleep(3)
        with self._lock:
            return self._conn.get_mode() == RobotMode.WALKING

    def sit(self) -> bool:
        with self._lock:
            self._conn.call(RpcApiId.ROBOT_LIE_DOWN)
        logger.info("K1 lying down")
        return True


# --- Step 4: backend factory (real-only for now; no K1 sim/replay yet) -------
def make_connection(ip: str, cfg: GlobalConfig) -> BoosterRPCConnection:
    # The Booster K1 has no simulation or replay backend yet, so this always
    # returns the real hardware connection. Add sim/replay branches here (keyed
    # off cfg) when they exist.
    return BoosterRPCConnection(ip)


def _camera_info_static() -> CameraInfo:
    # TODO: replace with measured K1 camera intrinsics (these are placeholders).
    fx, fy, cx, cy = (400.0, 400.0, 272.0, 153.0)
    width, height = (544, 306)
    return CameraInfo(
        frame_id="camera_optical",
        height=height,
        width=width,
        distortion_model="plumb_bob",
        D=[0.0, 0.0, 0.0, 0.0, 0.0],
        K=[fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
        R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        P=[fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        binning_x=0,
        binning_y=0,
    )


# --- Step 1: the connection Module -------------------------------------------
class K1Connection(Module, Camera):
    """Booster K1 humanoid — exposes camera + velocity control as DimOS streams/RPCs."""

    dedicated_worker = True

    config: ConnectionConfig

    # input: velocity command from MovementManager / teleop
    cmd_vel: In[Twist]
    # outputs: the Camera spec (color_image + camera_info)
    color_image: Out[Image]
    camera_info: Out[CameraInfo]

    camera_info_static: CameraInfo = _camera_info_static()
    _latest_frame: Image | None = None
    _camera_info_thread: Thread | None = None

    @classmethod
    def rerun_views(cls):  # type: ignore[no-untyped-def]
        """Rerun view blueprint for the K1 camera."""
        return [
            rrb.Spatial2DView(name="Camera", origin="world/robot/camera/rgb"),
        ]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop_event = Event()
        self.hw = make_connection(self.config.ip, self.config.g)

    @rpc
    def start(self) -> None:
        super().start()
        self.hw.start()

        def on_image(image: Image) -> None:  # publish AND cache for observe()
            self.color_image.publish(image)
            self._latest_frame = image

        self.register_disposable(self.hw.camera_stream().subscribe(on_image))
        self.register_disposable(Disposable(self.cmd_vel.subscribe(self.move)))

        # Camera intrinsics are static — republish on a timer so late subscribers get them.
        self._camera_info_thread = Thread(target=self._publish_camera_info, daemon=True)
        self._camera_info_thread.start()

        # Arm the robot so it accepts velocity commands.
        self.standup()
        logger.info("K1Connection started (ip=%s)", self.config.ip)

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._camera_info_thread and self._camera_info_thread.is_alive():
            self._camera_info_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self.hw.stop()
        super().stop()

    def _publish_camera_info(self) -> None:
        while not self._stop_event.is_set():
            self.camera_info.publish(self.camera_info_static)
            self._stop_event.wait(1.0)

    # --- control verbs (callable across the graph / by the CLI) ---
    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Send a base velocity command to the robot."""
        return self.hw.move(twist, duration)

    @rpc
    def standup(self) -> bool:
        """Arm the robot for walking (DAMPING -> PREPARE -> WALKING)."""
        return self.hw.standup()

    @rpc
    def sit(self) -> bool:
        """Make the robot lie down."""
        return self.hw.sit()

    # --- agent-callable skills ---
    @skill
    def walk(self, x: float, y: float = 0.0, yaw: float = 0.0, duration: float = 0.0) -> str:
        """Move the robot using direct velocity commands. Choose duration from the user's distance.

        Args:
            x: Forward velocity (m/s)
            y: Left/right velocity (m/s)
            yaw: Rotational velocity (rad/s)
            duration: How long to move (seconds); 0 = continuous until the next command
        """
        twist = Twist(linear=Vector3(x, y, 0.0), angular=Vector3(0.0, 0.0, yaw))
        if not self.move(twist, duration=duration):
            return "Failed to move."
        if duration > 0:
            return f"Moved at velocity=({x}, {y}, {yaw}) for {duration}s then stopped."
        return f"Moving at velocity=({x}, {y}, {yaw}) continuously — send a stop command to halt."

    @skill
    def stand(self) -> str:
        """Make the robot stand up from a sitting or damping position."""
        return "Robot is now standing." if self.standup() else "Failed to stand up."

    @skill
    def liedown(self) -> str:
        """Make the robot lie down."""
        return "Robot is now sitting." if self.sit() else "Failed to sit down."

    @skill
    def observe(self) -> Image | None:
        """Returns the latest camera frame. Use this for any visual world queries.

        Returns None if no frame has been captured yet.
        """
        return self._latest_frame

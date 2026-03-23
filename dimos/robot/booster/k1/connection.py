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

"""Booster K1 connection module using the booster-rpc SDK."""

import asyncio
import sys
from threading import Event, Lock, Thread
import time
from typing import Any

from booster_rpc import (  # type: ignore[import-untyped]
    BoosterConnection,
    GetRobotStatusResponse,
    RobotChangeModeRequest,
    RobotMode,
    RobotMoveRequest,
    RpcApiId,
)
import cv2
import numpy as np
from pydantic import Field
from reactivex.disposable import Disposable
import rerun.blueprint as rrb

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.spec.perception import Camera, Pointcloud
from dimos.utils.logging_config import setup_logger

if sys.version_info < (3, 13):
    pass
else:
    pass

logger = setup_logger()


class ConnectionConfig(ModuleConfig):
    ip: str = Field(default_factory=lambda m: m["g"].robot_ip)


def _camera_info_static() -> CameraInfo:
    # TODO: replace with actual K1 camera intrinsics
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


class K1Connection(Module[ConnectionConfig], Camera, Pointcloud):
    """Connection module for the Booster K1 humanoid robot."""

    default_config = ConnectionConfig

    cmd_vel: In[Twist]
    # TODO: publish pointcloud, odom, and lidar once K1 hardware exposes this data
    pointcloud: Out[PointCloud2]
    odom: Out[PoseStamped]
    lidar: Out[PointCloud2]
    color_image: Out[Image]
    camera_info: Out[CameraInfo]

    camera_info_static: CameraInfo = _camera_info_static()
    _camera_info_thread: Thread | None = None
    _video_thread: Thread | None = None
    _latest_video_frame: Image | None = None
    _conn: BoosterConnection | None = None

    @classmethod
    def rerun_views(cls):  # type: ignore[no-untyped-def]
        """Return Rerun view blueprints for K1 camera visualization."""
        return [
            rrb.Spatial2DView(
                name="Camera",
                origin="world/robot/camera/rgb",
            ),
        ]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._conn_lock = Lock()
        self._stop_event = Event()

    @rpc
    def start(self) -> None:
        super().start()

        self._conn = BoosterConnection(ip=self.config.ip)
        self._stop_event.clear()

        self._video_thread = Thread(target=self._run_video_stream, daemon=True)
        self._video_thread.start()

        self._disposables.add(Disposable(self.cmd_vel.subscribe(self.move)))

        self._camera_info_thread = Thread(target=self._publish_camera_info, daemon=True)
        self._camera_info_thread.start()

        logger.info("K1Connection started (ip=%s)", self.config.ip)

    @rpc
    def stop(self) -> None:
        self._stop_event.set()

        if self._video_thread and self._video_thread.is_alive():
            self._video_thread.join(timeout=3.0)

        if self._camera_info_thread and self._camera_info_thread.is_alive():
            self._camera_info_thread.join(timeout=1.0)

        if self._conn:
            with self._conn_lock:
                self._conn.close()
                self._conn = None

        super().stop()

    def _run_video_stream(self) -> None:
        """Run the async video stream in a background thread."""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self._stream_video())
        except Exception:
            if not self._stop_event.is_set():
                logger.exception("Video stream error")
        finally:
            loop.close()

    async def _stream_video(self) -> None:
        import websockets

        assert self._conn is not None
        uri = f"ws://{self._conn.ip}:{self._conn.ws_port}"

        JPEG_SOI = b"\xff\xd8"
        JPEG_EOI = b"\xff\xd9"

        while not self._stop_event.is_set():
            try:
                async with websockets.connect(uri, open_timeout=5) as ws:
                    while not self._stop_event.is_set():
                        data = await ws.recv()
                        if not isinstance(data, bytes):
                            continue
                        start = data.find(JPEG_SOI)
                        end = data.rfind(JPEG_EOI)
                        if start >= 0 and end >= 0:
                            frame = data[start : end + 2]
                            self._on_frame(frame)
            except TimeoutError as e:
                logger.warning("Video timeout: %s: %s, retrying in 3s...", type(e).__name__, e)
                await asyncio.sleep(3)
            except Exception as e:
                if self._stop_event.is_set():
                    break
                logger.warning("Video error: %s: %s, retrying in 3s...", type(e).__name__, e)
                await asyncio.sleep(3)

    def _on_frame(self, jpeg_bytes: bytes) -> None:
        if self._stop_event.is_set():
            return
        arr = cv2.imdecode(np.frombuffer(jpeg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
        if arr is None:
            return
        image = Image.from_numpy(arr, format=ImageFormat.BGR, frame_id="camera_optical")
        self.color_image.publish(image)
        self._latest_video_frame = image

    def _publish_camera_info(self) -> None:
        while not self._stop_event.is_set():
            self.camera_info.publish(self.camera_info_static)
            self._stop_event.wait(1.0)

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Send movement command to robot."""
        try:
            req = RobotMoveRequest(vx=twist.linear.x, vy=twist.linear.y, vyaw=twist.angular.z)
            with self._conn_lock:
                if not self._conn:
                    return False
                self._conn.call(RpcApiId.ROBOT_MOVE, bytes(req))
            if duration > 0:
                time.sleep(duration)
                stop = RobotMoveRequest(vx=0.0, vy=0.0, vyaw=0.0)
                with self._conn_lock:
                    if not self._conn:
                        return False
                    self._conn.call(RpcApiId.ROBOT_MOVE, bytes(stop))
            return True
        except Exception as e:
            logger.warning("Move command failed: %s: %s", type(e).__name__, e)
            return False

    @rpc
    def standup(self) -> bool:
        """Make the robot stand up (DAMPING -> PREPARE -> WALKING)."""
        try:
            with self._conn_lock:
                if not self._conn:
                    return False
                resp = self._conn.call(RpcApiId.GET_ROBOT_STATUS)
            status = GetRobotStatusResponse().parse(resp.payload)

            if status.mode == RobotMode.WALKING:
                return True

            if status.mode == RobotMode.DAMPING:
                with self._conn_lock:
                    if not self._conn:
                        return False
                    self._conn.call(
                        RpcApiId.ROBOT_CHANGE_MODE,
                        bytes(RobotChangeModeRequest(mode=RobotMode.PREPARE)),
                    )
                logger.info("K1 mode -> PREPARE")
                time.sleep(3)

            with self._conn_lock:
                if not self._conn:
                    return False
                self._conn.call(
                    RpcApiId.ROBOT_CHANGE_MODE,
                    bytes(RobotChangeModeRequest(mode=RobotMode.WALKING)),
                )
            logger.info("K1 mode -> WALKING")
            time.sleep(3)

            # Verify the mode transition actually succeeded
            with self._conn_lock:
                if not self._conn:
                    return False
                resp = self._conn.call(RpcApiId.GET_ROBOT_STATUS)
            status = GetRobotStatusResponse().parse(resp.payload)
            if status.mode != RobotMode.WALKING:
                logger.warning("K1 standup: expected WALKING mode but got %s", status.mode)
                return False
            return True
        except Exception:
            logger.exception("Failed to standup")
            return False

    @rpc
    def sit(self) -> bool:
        """Make the robot lie down."""
        try:
            with self._conn_lock:
                if not self._conn:
                    return False
                self._conn.call(RpcApiId.ROBOT_LIE_DOWN)
            logger.info("K1 lying down")
            return True
        except Exception:
            logger.exception("Failed to sit")
            return False

    @skill
    def walk(self, x: float, y: float = 0.0, yaw: float = 0.0, duration: float = 0.0) -> str:
        """Move the robot using direct velocity commands. Determine duration required based on user distance instructions.

        Example call:
            args = { "x": 0.5, "y": 0.0, "yaw": 0.0, "duration": 2.0 }
            walk(**args)

        Args:
            x: Forward velocity (m/s)
            y: Left/right velocity (m/s)
            yaw: Rotational velocity (rad/s)
            duration: How long to move (seconds)
        """
        twist = Twist(linear=Vector3(x, y, 0), angular=Vector3(0, 0, yaw))
        success = self.move(twist, duration=duration)
        if success:
            if duration > 0:
                return f"Moved with velocity=({x}, {y}, {yaw}) for {duration} seconds then stopped"
            else:
                return f"Started moving with velocity=({x}, {y}, {yaw}) continuously - send a stop command to halt"
        return "Failed to move."

    @skill
    def stand(self) -> str:
        """Make the robot stand up from a sitting or damping position.

        Example call:
            stand()
        """
        success = self.standup()
        if success:
            return "Robot is now standing."
        return "Failed to stand up."

    @skill
    def liedown(self) -> str:
        """Make the robot sit down (lie down).

        Example call:
            liedown()
        """
        success = self.sit()
        if success:
            return "Robot is now sitting."
        return "Failed to sit down."

    @skill
    def observe(self) -> Image | None:
        """Returns the latest video frame from the robot camera. Use this skill for any visual world queries.

        This skill provides the current camera view for perception tasks.
        Returns None if no frame has been captured yet.
        """
        return self._latest_video_frame


k1_connection = K1Connection.blueprint

__all__ = ["K1Connection", "k1_connection"]

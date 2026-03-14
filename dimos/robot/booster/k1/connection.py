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

"""Booster K1 connection module using the booster-rpc SDK."""

import asyncio
import logging
from threading import Thread
import time

from booster_rpc import (
    BoosterConnection,
    GetRobotStatusResponse,
    RobotChangeModeRequest,
    RobotMode,
    RobotMoveRequest,
    RpcApiId,
)
import cv2
import numpy as np
from reactivex.disposable import Disposable

from dimos import spec
from dimos.agents.annotation import skill
from dimos.core import In, Module, Out, rpc
from dimos.core.global_config import GlobalConfig, global_config
from dimos.msgs.geometry_msgs import PoseStamped, Twist
from dimos.msgs.sensor_msgs import CameraInfo, Image, PointCloud2
from dimos.msgs.sensor_msgs.Image import ImageFormat

logger = logging.getLogger(__name__)


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


class K1Connection(Module, spec.Camera):
    """Connection module for the Booster K1 humanoid robot."""

    cmd_vel: In[Twist]
    pointcloud: Out[PointCloud2]
    odom: Out[PoseStamped]
    lidar: Out[PointCloud2]
    color_image: Out[Image]
    camera_info: Out[CameraInfo]

    camera_info_static: CameraInfo = _camera_info_static()
    _global_config: GlobalConfig
    _camera_info_thread: Thread | None = None
    _video_thread: Thread | None = None
    _latest_video_frame: Image | None = None
    _conn: BoosterConnection | None = None
    _running: bool = False

    def __init__(
        self,
        ip: str | None = None,
        cfg: GlobalConfig = global_config,
        *args,
        **kwargs,
    ) -> None:
        self._global_config = cfg
        self._ip = ip if ip is not None else self._global_config.robot_ip
        Module.__init__(self, *args, **kwargs)

    @rpc
    def start(self) -> None:
        super().start()

        self._conn = BoosterConnection(ip=self._ip)
        self._running = True

        self._video_thread = Thread(target=self._run_video_stream, daemon=True)
        self._video_thread.start()

        self._disposables.add(Disposable(self.cmd_vel.subscribe(self.move)))

        self._camera_info_thread = Thread(target=self._publish_camera_info, daemon=True)
        self._camera_info_thread.start()

        logger.info("K1Connection started (ip=%s)", self._ip)

    @rpc
    def stop(self) -> None:
        self._running = False

        if self._video_thread and self._video_thread.is_alive():
            self._video_thread.join(timeout=3.0)

        if self._camera_info_thread and self._camera_info_thread.is_alive():
            self._camera_info_thread.join(timeout=1.0)

        if self._conn:
            self._conn.close()
            self._conn = None

        super().stop()

    def _run_video_stream(self) -> None:
        """Run the async video stream in a background thread."""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self._stream_video())
        except Exception as e:
            if self._running:
                logger.exception(f"Video stream exception: {type(e).__name__}: {e}")
        finally:
            loop.close()

    async def _stream_video(self) -> None:
        import websockets

        assert self._conn is not None
        uri = f"ws://{self._conn.ip}:{self._conn.ws_port}"

        JPEG_SOI = b"\xff\xd8"
        JPEG_EOI = b"\xff\xd9"

        while self._running:
            try:
                async with websockets.connect(uri, open_timeout=5) as ws:
                    while self._running:
                        data = await ws.recv()
                        if not isinstance(data, bytes):
                            continue
                        start = data.find(JPEG_SOI)
                        end = data.rfind(JPEG_EOI)
                        if start >= 0 and end >= 0:
                            frame = data[start : end + 2]
                            self._on_frame(frame)
            except KeyboardInterrupt:
                break
            except TimeoutError:
                logger.warning(f"Video timeout ({uri}), retrying in 3s...", uri)
                await asyncio.sleep(3)
            except Exception as e:
                if not self._running:
                    break
                logger.warning(f"Video error: {type(e).__name__}: {e}, retrying in 3s...", e)
                await asyncio.sleep(3)

    def _on_frame(self, jpeg_bytes: bytes) -> None:
        if not self._running:
            raise KeyboardInterrupt
        arr = cv2.imdecode(np.frombuffer(jpeg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
        if arr is None:
            return
        image = Image.from_numpy(arr, format=ImageFormat.BGR, frame_id="camera_optical")
        self.color_image.publish(image)
        self._latest_video_frame = image

    def _publish_camera_info(self) -> None:
        while self._running:
            self.camera_info.publish(self.camera_info_static)
            time.sleep(1.0)

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Send movement command to robot."""
        if not self._conn:
            return False
        try:
            req = RobotMoveRequest(vx=twist.linear.x, vy=twist.linear.y, vyaw=twist.angular.z)
            self._conn._call(RpcApiId.ROBOT_MOVE, bytes(req))
            return True
        except Exception as e:
            logger.debug("Move command failed: %s", e)
            return False

    @rpc
    def standup(self) -> bool:
        """Make the robot stand up (DAMPING -> PREPARE -> WALKING)."""
        if not self._conn:
            return False
        try:
            resp = self._conn._call(RpcApiId.GET_ROBOT_STATUS)
            status = GetRobotStatusResponse().parse(resp.payload)

            if status.mode == RobotMode.WALKING:
                return True

            if status.mode == RobotMode.DAMPING:
                self._conn._call(
                    RpcApiId.ROBOT_CHANGE_MODE,
                    bytes(RobotChangeModeRequest(mode=RobotMode.PREPARE)),
                )
                logger.info("K1 mode -> PREPARE")
                time.sleep(3)

            self._conn._call(
                RpcApiId.ROBOT_CHANGE_MODE,
                bytes(RobotChangeModeRequest(mode=RobotMode.WALKING)),
            )
            logger.info("K1 mode -> WALKING")
            time.sleep(3)
            return True
        except Exception:
            logger.exception("Failed to standup")
            return False

    @rpc
    def sit(self) -> bool:
        """Make the robot lie down."""
        if not self._conn:
            return False
        try:
            self._conn._call(RpcApiId.ROBOT_LIE_DOWN)
            logger.info("K1 lying down")
            return True
        except Exception:
            logger.exception("Failed to sit")
            return False

    @skill
    def observe(self) -> Image | None:
        """Returns the latest video frame from the robot camera. Use this skill for any visual world queries.

        This skill provides the current camera view for perception tasks.
        Returns None if no frame has been captured yet.
        """
        return self._latest_video_frame


k1_connection = K1Connection.blueprint

__all__ = ["K1Connection", "k1_connection"]

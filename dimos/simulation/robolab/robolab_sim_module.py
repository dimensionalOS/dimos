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

"""RoboLab's camera as a dimos depth camera.

RoboLab runs the scene and the DROID robot in Isaac Sim; its runner sends one RGB-D
camera frame each sim step (see dimos.simulation.robolab.protocol). This module
publishes it as ``color_image``, ``depth_image``, ``camera_info`` and the camera's
``tf`` in the world, like MujocoSimModule does for MuJoCo. The arm itself is driven by
the ``sim_robolab`` adapter.
"""

from __future__ import annotations

import threading
import time
from typing import Any

import numpy as np
import reactivex as rx
import zenoh

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.hardware.sensors.camera.spec import DepthCameraConfig, DepthCameraHardware
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.simulation.robolab.protocol import (
    DEFAULT_ENDPOINT,
    DEFAULT_PREFIX,
    CameraFrame,
    open_session,
    pack_release,
    unpack,
)
from dimos.spec import perception
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# OpenGL camera (looking down -z, y up) to optical (z forward, y down): a half turn about
# x, as a quaternion (x, y, z, w).
_OPENGL_TO_OPTICAL = np.array([1.0, 0.0, 0.0, 0.0])


def _quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product of two (x, y, z, w) quaternions: rotate by b, then by a."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ]
    )


class RoboLabSimModuleConfig(ModuleConfig, DepthCameraConfig):
    # Zenoh endpoint and key prefix of RoboLab's runner.
    address: str = DEFAULT_ENDPOINT
    prefix: str = DEFAULT_PREFIX
    # RoboLab's name for the camera the runner sends (its depth-enabled camera).
    camera_name: str = "over_shoulder_left_camera"
    frame_id_prefix: str | None = None
    width: int = 1280
    height: int = 720
    fps: int = 15
    align_depth_to_color: bool = True
    enable_depth: bool = True
    enable_pointcloud: bool = False
    pointcloud_fps: float = 5.0
    camera_info_fps: float = 1.0


class RoboLabSimModule(DepthCameraHardware, Module, perception.DepthCamera):
    """Publishes RoboLab's camera frames as dimos streams and reports episode boundaries."""

    config: RoboLabSimModuleConfig
    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]
    depth_camera_info: Out[CameraInfo]
    tf: Out[TFMessage]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._session: zenoh.Session | None = None
        self._release_pub: Any = None
        self._lock = threading.Lock()
        self._camera_info_base: CameraInfo | None = None
        self._latest_ts: float | None = None
        self._frames = 0
        self._episode: dict[str, Any] = {"episode": None, "running": False, "steps": 0}

    @property
    def _camera_link(self) -> str:
        return f"{self.config.camera_name}_link"

    @property
    def _color_frame(self) -> str:
        return f"{self.config.camera_name}_color_frame"

    @property
    def _color_optical_frame(self) -> str:
        return f"{self.config.camera_name}_color_optical_frame"

    @property
    def _depth_frame(self) -> str:
        return f"{self.config.camera_name}_depth_frame"

    @property
    def _depth_optical_frame(self) -> str:
        return f"{self.config.camera_name}_depth_optical_frame"

    @rpc
    def start(self) -> None:
        super().start()
        self._session = open_session(self.config.address)
        prefix = self.config.prefix
        self._session.declare_subscriber(f"{prefix}/camera", self._on_camera)
        self._session.declare_subscriber(f"{prefix}/status", self._on_status)
        self._release_pub = self._session.declare_publisher(f"{prefix}/cmd")
        self.register_disposable(
            rx.interval(1.0 / self.config.camera_info_fps).subscribe(
                on_next=lambda _: self._publish_camera_info(),
                on_error=lambda e: logger.error("CameraInfo publish error", error=str(e)),
            )
        )
        logger.info("RoboLabSimModule listening", endpoint=self.config.address, prefix=prefix)

    @rpc
    def stop(self) -> None:
        session, self._session = self._session, None
        self._release_pub = None
        if session is not None:
            session.close()
        with self._lock:
            self._camera_info_base = None
            self._latest_ts = None
        super().stop()

    @rpc
    def get_color_camera_info(self) -> CameraInfo | None:
        with self._lock:
            base, ts = self._camera_info_base, self._latest_ts
        return None if base is None else base.with_ts(ts if ts is not None else time.time())

    @rpc
    def get_depth_camera_info(self) -> CameraInfo | None:
        return self.get_color_camera_info()

    @rpc
    def get_depth_scale(self) -> float:
        return 1.0

    @rpc
    def episode_status(self) -> dict[str, Any]:
        """The current RoboLab episode: its index, whether it is running, and its sim steps."""
        with self._lock:
            return dict(self._episode)

    @rpc
    def finish_episode(self) -> bool:
        """Stop holding the sim for dimos: RoboLab runs the episode to its time limit."""
        if self._release_pub is None:
            return False
        self._release_pub.put(pack_release())
        return True

    def _on_status(self, sample: zenoh.Sample) -> None:
        message = unpack(sample)
        with self._lock:
            if message.get("event") == "episode_start":
                self._episode = {"episode": message.get("episode"), "running": True, "steps": 0}
            elif message.get("event") == "episode_end":
                self._episode = {
                    "episode": message.get("episode"),
                    "running": False,
                    "steps": int(message.get("steps", 0)),
                }
        logger.info(
            "RoboLab episode",
            robolab_event=message.get("event"),
            episode=message.get("episode"),
            steps=message.get("steps"),
        )

    def _on_camera(self, sample: zenoh.Sample) -> None:
        frame = CameraFrame.decode(unpack(sample))
        if frame.camera != self.config.camera_name:
            return
        ts = time.time()
        with self._lock:
            self._latest_ts = ts
            if self._camera_info_base is None:
                self._camera_info_base = self._camera_info_from(frame)
            self._episode["steps"] = frame.step
        self.color_image.publish(
            Image(data=frame.rgb, format=ImageFormat.RGB, frame_id=self._color_optical_frame, ts=ts)
        )
        if self.config.enable_depth:
            # Nothing in range reads inf in Isaac; depth cameras report that as 0.
            depth = np.where(np.isfinite(frame.depth), frame.depth, 0.0).astype(np.float32)
            self.depth_image.publish(
                Image(
                    data=depth, format=ImageFormat.DEPTH, frame_id=self._color_optical_frame, ts=ts
                )
            )
        self._publish_tf(frame, ts)
        self._frames += 1
        if self._frames == 1:
            self._publish_camera_info()
            logger.info("RoboLabSimModule first frame", rgb_shape=frame.rgb.shape, step=frame.step)

    def _camera_info_from(self, frame: CameraFrame) -> CameraInfo:
        height, width = frame.rgb.shape[:2]
        return CameraInfo.from_intrinsics(
            fx=float(frame.K[0, 0]),
            fy=float(frame.K[1, 1]),
            cx=float(frame.K[0, 2]),
            cy=float(frame.K[1, 2]),
            width=width,
            height=height,
            frame_id=self._color_optical_frame,
        )

    def _publish_camera_info(self) -> None:
        info = self.get_color_camera_info()
        if info is None:
            return
        self.camera_info.publish(info)
        self.depth_camera_info.publish(info)

    def _publish_tf(self, frame: CameraFrame, ts: float) -> None:
        position = Vector3(*(float(v) for v in frame.position))

        def transform(quat_xyzw: np.ndarray, child: str) -> Transform:
            x, y, z, w = quat_xyzw
            return Transform(
                translation=position,
                rotation=Quaternion(float(x), float(y), float(z), float(w)),
                frame_id="world",
                child_frame_id=child,
                ts=ts,
            )

        optical = _quat_multiply(frame.quat_xyzw, _OPENGL_TO_OPTICAL)
        self.tf.publish(
            TFMessage(
                transform(optical, self._color_optical_frame),
                transform(optical, self._depth_optical_frame),
                transform(frame.quat_xyzw, self._camera_link),
            )
        )

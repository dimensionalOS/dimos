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

"""Orbbec Gemini 330-series camera (pyorbbecsdk2): depth aligned to colour, RealSense frame names."""

from __future__ import annotations

import threading
from typing import Any

import numpy as np
from pyorbbecsdk import (
    AlignFilter,
    Config,
    OBFormat,
    OBFrameAggregateOutputMode,
    OBSensorType,
    OBStreamType,
    Pipeline,
)
import reactivex as rx
from scipy.spatial.transform import Rotation

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.hardware.sensors.camera.spec import OPTICAL_ROTATION, DepthCameraConfig
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.spec import perception
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()

_IDENTITY = Quaternion(0.0, 0.0, 0.0, 1.0)


class OrbbecCameraConfig(ModuleConfig, DepthCameraConfig):
    width: int = 848
    height: int = 480
    fps: int = 15
    # The camera's tf link, on the depth imager; imager frames drop its `_link`.
    frame_id: str | None = "camera_link"
    align_depth_to_color: bool = True
    enable_depth: bool = True
    enable_pointcloud: bool = False
    pointcloud_fps: float = 5.0
    depth_trunc_m: float = 3.0
    camera_info_fps: float = 1.0
    # Mounted upside down: images, intrinsics and the colour extrinsic are turned right side up.
    rotate_180: bool = False


class OrbbecCamera(Module, perception.DepthCamera):
    config: OrbbecCameraConfig
    color_image: Out[Image]
    depth_image: Out[Image]
    pointcloud: Out[PointCloud2]
    camera_info: Out[CameraInfo]
    depth_camera_info: Out[CameraInfo]
    tf: Out[TFMessage]

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._pipeline: Pipeline | None = None
        self._running = False
        self._thread: threading.Thread | None = None
        self._camera_info: CameraInfo | None = None
        self._mount: list[Transform] = []
        self._last_ts: float | None = None
        self._latest_rgbd: tuple[Image, Image] | None = None

    def _frame(self, suffix: str) -> str:
        stem = (self.config.frame_id or "camera_link").removesuffix("_link")
        name = f"{stem}_{suffix}"
        return f"{self.config.frame_id_prefix}/{name}" if self.config.frame_id_prefix else name

    @rpc
    def start(self) -> None:
        super().start()
        pipeline = Pipeline()
        color = pipeline.get_stream_profile_list(
            OBSensorType.COLOR_SENSOR
        ).get_video_stream_profile(
            self.config.width, self.config.height, OBFormat.RGB, self.config.fps
        )
        depth = pipeline.get_stream_profile_list(
            OBSensorType.DEPTH_SENSOR
        ).get_video_stream_profile(
            self.config.width, self.config.height, OBFormat.Y16, self.config.fps
        )
        config = Config()
        config.enable_stream(color)
        config.enable_stream(depth)
        config.set_frame_aggregate_output_mode(OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE)
        pipeline.enable_frame_sync()
        pipeline.start(config)
        self._pipeline = pipeline

        k = color.get_intrinsic()
        dist = color.get_distortion()
        cx, cy, p1, p2 = k.cx, k.cy, dist.p1, dist.p2
        if self.config.rotate_180:
            # Pixel (u, v) lands on (w-1-u, h-1-v); tangential distortion flips sign.
            cx, cy, p1, p2 = k.width - 1 - cx, k.height - 1 - cy, -p1, -p2
        self._camera_info = CameraInfo(
            height=k.height,
            width=k.width,
            distortion_model="plumb_bob",
            D=[dist.k1, dist.k2, p1, p2, dist.k3],
            K=[k.fx, 0.0, cx, 0.0, k.fy, cy, 0.0, 0.0, 1.0],
            P=[k.fx, 0.0, cx, 0.0, 0.0, k.fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
            frame_id=self._frame("color_optical_frame"),
        )
        self._mount = self._mount_edges(color.get_extrinsic_to(depth))

        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        self.register_disposable(
            rx.interval(1.0 / self.config.camera_info_fps).subscribe(
                on_next=lambda _: self._publish_camera_info()
            )
        )
        if self.config.enable_pointcloud:
            self.register_disposable(
                backpressure(rx.interval(1.0 / self.config.pointcloud_fps)).subscribe(
                    on_next=lambda _: self._publish_pointcloud()
                )
            )

    def _mount_edges(self, color_to_depth: Any) -> list[Transform]:
        """camera_link sits on the depth imager; the colour imager hangs off it by the factory extrinsic."""
        body_from_optical = Rotation.from_quat(
            [OPTICAL_ROTATION.x, OPTICAL_ROTATION.y, OPTICAL_ROTATION.z, OPTICAL_ROTATION.w]
        ).as_matrix()
        rotation = (
            body_from_optical @ np.array(color_to_depth.rot).reshape(3, 3) @ body_from_optical.T
        )
        translation = body_from_optical @ (np.array(color_to_depth.transform) / 1000.0)
        if self.config.rotate_180:
            # The colour imager as seen from the camera rolled right side up.
            flip = np.diag([1.0, -1.0, -1.0])
            rotation = flip @ rotation @ flip
            translation = flip @ translation
        link = self.frame_id
        edges = [
            (link, self._frame("depth_frame"), Vector3(), _IDENTITY),
            (
                link,
                self._frame("color_frame"),
                Vector3(*translation),
                Quaternion(*Rotation.from_matrix(rotation).as_quat()),
            ),
        ]
        for imager in ("depth", "color"):
            edges.append(
                (
                    self._frame(f"{imager}_frame"),
                    self._frame(f"{imager}_optical_frame"),
                    Vector3(),
                    OPTICAL_ROTATION,
                )
            )
        return [
            Transform(translation=t, rotation=r, frame_id=parent, child_frame_id=child)
            for parent, child, t, r in edges
        ]

    def _capture_loop(self) -> None:
        align = AlignFilter(align_to_stream=OBStreamType.COLOR_STREAM)
        optical = self._frame("color_optical_frame")
        while self._running and self._pipeline is not None:
            try:
                frames = self._pipeline.wait_for_frames(1000)
                aligned = align.process(frames) if frames is not None else None
            except Exception:
                logger.exception("Orbbec frame wait failed")
                continue
            if aligned is None:
                continue
            aligned = aligned.as_frame_set()
            color, depth = aligned.get_color_frame(), aligned.get_depth_frame()
            if color is None or depth is None:
                continue
            # Device time mapped onto the host clock, falling back to arrival time.
            ts = (color.get_global_timestamp_us() or color.get_system_timestamp_us()) / 1e6
            rgb = np.frombuffer(color.get_data(), dtype=np.uint8)
            rgb = rgb.reshape(color.get_height(), color.get_width(), 3)
            mm = np.frombuffer(depth.get_data(), dtype=np.uint16)
            mm = mm.reshape(depth.get_height(), depth.get_width())
            if self.config.rotate_180:
                rgb, mm = rgb[::-1, ::-1], mm[::-1, ::-1]
            rgb, mm = rgb.copy(), mm.copy()
            color_img = Image(data=rgb, format=ImageFormat.RGB, frame_id=optical, ts=ts)
            depth_img = Image(data=mm, format=ImageFormat.DEPTH16, frame_id=optical, ts=ts)
            self.color_image.publish(color_img)
            self.depth_image.publish(depth_img)
            self._latest_rgbd = (color_img, depth_img)
            self._last_ts = ts
            for edge in self._mount:
                edge.ts = ts
            self.tf.publish(TFMessage(*self._mount))

    def _publish_camera_info(self) -> None:
        if self._camera_info is None or self._last_ts is None:
            return
        self._camera_info.ts = self._last_ts
        self.camera_info.publish(self._camera_info)
        self.depth_camera_info.publish(self._camera_info)

    def _publish_pointcloud(self) -> None:
        rgbd = self._latest_rgbd
        if rgbd is None or self._camera_info is None:
            return
        try:
            self.pointcloud.publish(
                PointCloud2.from_rgbd(
                    color_image=rgbd[0],
                    depth_image=rgbd[1],
                    camera_info=self._camera_info,
                    depth_scale=0.001,
                    depth_trunc=self.config.depth_trunc_m,
                ).voxel_downsample(0.005)
            )
        except Exception:
            logger.exception("Orbbec pointcloud failed")

    @rpc
    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._thread = None
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:
                logger.exception("Orbbec pipeline stop failed")
            self._pipeline = None
        super().stop()

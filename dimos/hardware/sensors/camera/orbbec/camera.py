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

"""Orbbec Gemini 330-series color and depth camera."""

from __future__ import annotations

import threading
import time
from typing import Any

import numpy as np
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

try:
    from pyorbbecsdk import (
        AlignFilter,
        Config,
        Context,
        OBFormat,
        OBFrameAggregateOutputMode,
        OBSensorType,
        OBStreamType,
        Pipeline,
        VideoStreamProfile,
    )
except ImportError as exc:
    _sdk_error: ImportError | None = exc
else:
    _sdk_error = None

logger = setup_logger()

_IDENTITY = Quaternion(0.0, 0.0, 0.0, 1.0)


class OrbbecCameraConfig(ModuleConfig, DepthCameraConfig):
    width: int = 848
    height: int = 480
    fps: int = 15
    frame_id: str | None = "camera_link"
    serial_number: str | None = None
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
        self._context: Context | None = None
        self._pipeline: Pipeline | None = None
        self._running = False
        self._thread: threading.Thread | None = None
        self._camera_info: CameraInfo | None = None
        self._depth_camera_info: CameraInfo | None = None
        self._mount: list[Transform] = []
        self._last_ts: float | None = None
        self._latest_rgbd: tuple[Image, Image] | None = None

    def _frame(self, suffix: str) -> str:
        stem = (self.config.frame_id or "camera_link").removesuffix("_link")
        name = f"{stem}_{suffix}"
        return f"{self.config.frame_id_prefix}/{name}" if self.config.frame_id_prefix else name

    @classmethod
    def check_dependencies(cls, **kwargs: Any) -> None:
        if _sdk_error is not None:
            raise ImportError(
                "Install the Gemini SDK with 'uv pip install --no-deps pyorbbecsdk2==2.1.2'"
            ) from _sdk_error

    @rpc
    def start(self) -> None:
        self.check_dependencies()
        if self.config.enable_pointcloud and not (
            self.config.enable_depth and self.config.align_depth_to_color
        ):
            raise ValueError("Orbbec point clouds require depth aligned to color")
        super().start()
        if self.config.serial_number:
            self._context = Context()
            devices = self._context.query_devices()
            pipeline = Pipeline(devices.get_device_by_serial_number(self.config.serial_number))
        else:
            pipeline = Pipeline()
        color = pipeline.get_stream_profile_list(
            OBSensorType.COLOR_SENSOR
        ).get_video_stream_profile(
            self.config.width, self.config.height, OBFormat.RGB, self.config.fps
        )
        config = Config()
        config.enable_stream(color)
        self._camera_info = self._intrinsics(color, "color_optical_frame")
        depth_profiles = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
        if self.config.enable_depth:
            depth = depth_profiles.get_video_stream_profile(
                self.config.width, self.config.height, OBFormat.Y16, self.config.fps
            )
            config.enable_stream(depth)
            config.set_frame_aggregate_output_mode(OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE)
            pipeline.enable_frame_sync()
            self._depth_camera_info = (
                self._camera_info
                if self.config.align_depth_to_color
                else self._intrinsics(depth, "depth_optical_frame")
            )
        else:
            # Read the factory extrinsic without opening the depth stream.
            depth = depth_profiles.get_default_video_stream_profile()
        self._mount = self._mount_edges(color.get_extrinsic_to(depth))

        pipeline.start(config)
        self._pipeline = pipeline
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

    def _intrinsics(self, profile: VideoStreamProfile, frame: str) -> CameraInfo:
        k = profile.get_intrinsic()
        dist = profile.get_distortion()
        cx, cy, p1, p2 = k.cx, k.cy, dist.p1, dist.p2
        if self.config.rotate_180:
            cx, cy, p1, p2 = k.width - 1 - cx, k.height - 1 - cy, -p1, -p2
        return CameraInfo(
            height=k.height,
            width=k.width,
            distortion_model="plumb_bob",
            D=[dist.k1, dist.k2, p1, p2, dist.k3],
            K=[k.fx, 0.0, cx, 0.0, k.fy, cy, 0.0, 0.0, 1.0],
            P=[k.fx, 0.0, cx, 0.0, 0.0, k.fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
            frame_id=self._frame(frame),
        )

    def _mount_edges(self, color_to_depth: Any) -> list[Transform]:
        """Express the factory color-to-depth extrinsic in the camera link frame."""
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
        align = (
            AlignFilter(align_to_stream=OBStreamType.COLOR_STREAM)
            if self.config.enable_depth and self.config.align_depth_to_color
            else None
        )
        optical = self._frame("color_optical_frame")
        while self._running and self._pipeline is not None:
            try:
                frames = self._pipeline.wait_for_frames(1000)
                if frames is None:
                    continue
                if align is not None:
                    aligned = align.process(frames)
                    if aligned is None:
                        continue
                    frames = aligned.as_frame_set()
            except Exception:
                logger.exception("Orbbec frame wait failed")
                time.sleep(0.1)
                continue
            color = frames.get_color_frame()
            if color is None:
                continue
            # Device time mapped onto the host clock, falling back to arrival time.
            ts = (color.get_global_timestamp_us() or color.get_system_timestamp_us()) / 1e6
            rgb = np.frombuffer(color.get_data(), dtype=np.uint8)
            rgb = rgb.reshape(color.get_height(), color.get_width(), 3)
            if self.config.rotate_180:
                rgb = rgb[::-1, ::-1]
            color_img = Image(data=rgb.copy(), format=ImageFormat.RGB, frame_id=optical, ts=ts)
            self.color_image.publish(color_img)
            if (
                self._depth_camera_info is not None
                and (depth := frames.get_depth_frame()) is not None
            ):
                mm = np.frombuffer(depth.get_data(), dtype=np.uint16).reshape(
                    depth.get_height(), depth.get_width()
                )
                if self.config.rotate_180:
                    mm = mm[::-1, ::-1]
                depth_img = Image(
                    data=mm.copy(),
                    format=ImageFormat.DEPTH16,
                    frame_id=self._depth_camera_info.frame_id,
                    ts=ts,
                )
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
        if self._depth_camera_info is not None:
            self._depth_camera_info.ts = self._last_ts
            self.depth_camera_info.publish(self._depth_camera_info)

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
        self._context = None
        super().stop()

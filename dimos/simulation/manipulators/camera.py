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

"""MuJoCo simulation camera module.

Drop-in replacement for RealSenseCamera in manipulation blueprints.
Renders RGB + depth from a named camera in a shared MujocoEngine.
"""

from __future__ import annotations

import math
import threading
import time

from pydantic import Field
import reactivex as rx

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.hardware.sensors.camera.spec import (
    DepthCameraConfig,
    DepthCameraHardware,
)
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.simulation.engines.mujoco_engine import (
    CameraConfig,
    CameraFrame,
    MujocoEngine,
    get_or_create_engine,
)
from dimos.spec import perception
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()


def _default_identity_transform() -> Transform:
    return Transform(
        translation=Vector3(0.0, 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )


class MujocoCameraConfig(ModuleConfig, DepthCameraConfig):
    """Configuration for the MuJoCo simulation camera."""

    address: str = ""
    headless: bool = False
    camera_name: str = "wrist_camera"
    width: int = 640
    height: int = 480
    fps: int = 15
    base_frame_id: str = "link7"
    base_transform: Transform | None = Field(default_factory=_default_identity_transform)
    # MuJoCo renders color+depth from same virtual camera — alignment is a no-op
    align_depth_to_color: bool = True
    enable_depth: bool = True
    enable_pointcloud: bool = False
    pointcloud_fps: float = 5.0
    camera_info_fps: float = 1.0


class MujocoCamera(DepthCameraHardware, Module[MujocoCameraConfig], perception.DepthCamera):
    """Simulated depth camera that renders from a MujocoEngine.

    Implements the same output ports and TF chain as RealSenseCamera so it can
    be used as a drop-in replacement in manipulation blueprints.

    The engine is resolved automatically from the registry via ``address``
    (the same MJCF path used by the sim_mujoco adapter). Alternatively,
    call ``set_engine()`` before ``start()``.
    """

    color_image: Out[Image]
    depth_image: Out[Image]
    pointcloud: Out[PointCloud2]
    camera_info: Out[CameraInfo]
    depth_camera_info: Out[CameraInfo]

    default_config = MujocoCameraConfig

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self._engine: MujocoEngine | None = None
        self._running = False
        self._thread: threading.Thread | None = None
        self._color_camera_info: CameraInfo | None = None
        self._depth_camera_info: CameraInfo | None = None

    # -- Engine injection (called before start) --

    def set_engine(self, engine: MujocoEngine) -> None:
        """Inject the shared MujocoEngine reference."""
        self._engine = engine

    # -- DepthCameraHardware interface --

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
    def get_color_camera_info(self) -> CameraInfo | None:
        return self._color_camera_info

    @rpc
    def get_depth_camera_info(self) -> CameraInfo | None:
        return self._depth_camera_info

    @rpc
    def get_depth_scale(self) -> float:
        # MuJoCo depth is already in meters
        return 1.0

    # -- Intrinsics --

    def _build_camera_info(self) -> None:
        """Compute camera intrinsics from the MuJoCo model (pinhole, no distortion)."""
        if self._engine is None:
            return
        fovy_deg = self._engine.get_camera_fovy(self.config.camera_name)
        if fovy_deg is None:
            logger.error(f"Camera '{self.config.camera_name}' not found in MJCF")
            return

        h = self.config.height
        w = self.config.width
        fovy_rad = math.radians(fovy_deg)
        fy = h / (2.0 * math.tan(fovy_rad / 2.0))
        fx = fy  # square pixels
        cx = w / 2.0
        cy = h / 2.0

        K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        D = [0.0, 0.0, 0.0, 0.0, 0.0]

        info = CameraInfo(
            height=h,
            width=w,
            distortion_model="plumb_bob",
            D=D,
            K=K,
            P=P,
            frame_id=self._color_optical_frame,
        )
        # Color and depth share the same virtual camera
        self._color_camera_info = info
        self._depth_camera_info = CameraInfo(
            height=h,
            width=w,
            distortion_model="plumb_bob",
            D=D,
            K=K,
            P=P,
            frame_id=self._color_optical_frame,
        )

    # -- Lifecycle --

    @rpc
    def start(self) -> None:
        if self._engine is None and self.config.address:
            from pathlib import Path

            self._engine = get_or_create_engine(
                config_path=Path(self.config.address),
                headless=self.config.headless,
                cameras=[
                    CameraConfig(
                        name=self.config.camera_name,
                        width=self.config.width,
                        height=self.config.height,
                        fps=self.config.fps,
                    )
                ],
            )
        if self._engine is None:
            raise RuntimeError(
                "MujocoCamera: set address (MJCF path) in config or call set_engine()"
            )

        logger.info(
            f"MujocoCamera: engine resolved, connected={self._engine.connected}, "
            f"cameras={[c.name for c in self._engine._camera_configs]}"
        )

        self._build_camera_info()

        self._running = True
        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()

        # Periodic camera_info publishing
        interval_sec = 1.0 / self.config.camera_info_fps
        self._disposables.add(
            rx.interval(interval_sec).subscribe(
                on_next=lambda _: self._publish_camera_info(),
                on_error=lambda e: logger.error(f"CameraInfo publish error: {e}"),
            )
        )

        # Optional pointcloud generation
        if self.config.enable_pointcloud and self.config.enable_depth:
            pc_interval = 1.0 / self.config.pointcloud_fps
            self._disposables.add(
                backpressure(rx.interval(pc_interval)).subscribe(
                    on_next=lambda _: self._generate_pointcloud(),
                    on_error=lambda e: logger.error(f"Pointcloud error: {e}"),
                )
            )

    @rpc
    def stop(self) -> None:
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
            self._thread = None
        self._color_camera_info = None
        self._depth_camera_info = None
        super().stop()

    # -- Publishing --

    def _publish_loop(self) -> None:
        """Poll engine for rendered frames and publish at configured FPS."""
        interval = 1.0 / self.config.fps
        last_timestamp = 0.0
        published_count = 0

        logger.info(f"MujocoCamera publish loop started (camera={self.config.camera_name})")

        # Wait for engine to connect (adapter may not have started yet)
        while self._running and self._engine is not None and not self._engine.connected:
            time.sleep(0.1)

        if not self._running:
            return

        logger.info("MujocoCamera: engine connected, polling for frames")

        while self._running and self._engine is not None:
            try:
                frame = self._engine.read_camera(self.config.camera_name)
                if frame is None or frame.timestamp <= last_timestamp:
                    time.sleep(interval * 0.5)
                    continue

                last_timestamp = frame.timestamp
                ts = time.time()

                # Publish RGB
                color_img = Image(
                    data=frame.rgb,
                    format=ImageFormat.RGB,
                    frame_id=self._color_optical_frame,
                    ts=ts,
                )
                self.color_image.publish(color_img)

                # Publish depth (float32 meters)
                if self.config.enable_depth:
                    depth_img = Image(
                        data=frame.depth,
                        format=ImageFormat.DEPTH,
                        frame_id=self._color_optical_frame,
                        ts=ts,
                    )
                    self.depth_image.publish(depth_img)

                # Publish TF (world -> camera from MuJoCo pose)
                self._publish_tf(ts, frame)

                published_count += 1
                if published_count == 1:
                    logger.info(
                        f"MujocoCamera: first frame published "
                        f"(rgb={frame.rgb.shape}, depth={frame.depth.shape})"
                    )

                # Rate limit
                elapsed = time.time() - ts
                sleep_time = interval - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
            except Exception as e:
                logger.error(f"MujocoCamera publish error: {e}")
                time.sleep(1.0)

    def _publish_camera_info(self) -> None:
        ts = time.time()
        if self._color_camera_info:
            self._color_camera_info.ts = ts
            self.camera_info.publish(self._color_camera_info)
        if self._depth_camera_info:
            self._depth_camera_info.ts = ts
            self.depth_camera_info.publish(self._depth_camera_info)

    def _publish_tf(self, ts: float, frame: CameraFrame | None = None) -> None:
        if frame is None:
            return

        from scipy.spatial.transform import Rotation as R
        # MuJoCo cam frame -> optical frame: flip Y and Z (Rx 180°)
        _RX180 = R.from_euler("x", 180, degrees=True)
        mj_rot = R.from_matrix(frame.cam_mat.reshape(3, 3))
        optical_rot = mj_rot * _RX180
        q = optical_rot.as_quat()  # xyzw

        pos = Vector3(
            float(frame.cam_pos[0]),
            float(frame.cam_pos[1]),
            float(frame.cam_pos[2]),
        )
        rot = Quaternion(float(q[0]), float(q[1]), float(q[2]), float(q[3]))

        # Publish world -> all optical/link frames (all co-located in sim)
        self.tf.publish(
            Transform(
                translation=pos,
                rotation=rot,
                frame_id="world",
                child_frame_id=self._color_optical_frame,
                ts=ts,
            ),
            Transform(
                translation=pos,
                rotation=rot,
                frame_id="world",
                child_frame_id=self._depth_optical_frame,
                ts=ts,
            ),
            Transform(
                translation=pos,
                rotation=rot,
                frame_id="world",
                child_frame_id=self._camera_link,
                ts=ts,
            ),
        )

    def _generate_pointcloud(self) -> None:
        """Generate pointcloud from latest depth frame (optional, for visualization)."""
        if self._engine is None or self._color_camera_info is None:
            return
        frame = self._engine.read_camera(self.config.camera_name)
        if frame is None:
            return
        try:
            color_img = Image(
                data=frame.rgb,
                format=ImageFormat.RGB,
                frame_id=self._color_optical_frame,
                ts=frame.timestamp,
            )
            depth_img = Image(
                data=frame.depth,
                format=ImageFormat.DEPTH,
                frame_id=self._color_optical_frame,
                ts=frame.timestamp,
            )
            pcd = PointCloud2.from_rgbd(
                color_image=color_img,
                depth_image=depth_img,
                camera_info=self._color_camera_info,
                depth_scale=1.0,
            )
            pcd = pcd.voxel_downsample(0.005)
            self.pointcloud.publish(pcd)
        except Exception as e:
            logger.error(f"Pointcloud generation error: {e}")


__all__ = ["MujocoCamera", "MujocoCameraConfig"]

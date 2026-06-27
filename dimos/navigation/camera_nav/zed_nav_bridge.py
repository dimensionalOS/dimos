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

"""ZEDNavBridge — ZED camera outputs → nav-pipeline-compatible messages.

Replaces HardwareDepthModule for ZED cameras. Key differences:

1. TF: looks up world→camera_link directly (1-hop, always published by ZEDCamera)
   then composes with cached static extrinsics (camera_link→depth_optical_frame).
   HardwareDepthModule attempts a 3-hop chain that fails on startup timing.

2. Retry logic: updates VIO transform every frame, not once every 5s on failure.

3. Output ports named to match downstream consumers without remappings:
     lidar:    Out[PointCloud2]  →  VoxelGridMapper.lidar / RayTracingVoxelMap.lidar
     odometry: Out[Odometry]    →  RayTracingVoxelMap.odometry

Wire it between ZEDCamera and VoxelGridMapper (or RayTracingVoxelMap):

    camera_nav_zed_nav = autoconnect(
        ZEDCamera.blueprint(..., enable_tracking=True),
        ZEDNavBridge.blueprint(camera_name="camera", stride=2),
        VoxelGridMapper.blueprint(voxel_size=0.05),
        _RERUN_VIZ,
    )
"""
from __future__ import annotations

import threading
from typing import Any

import numpy as np
from reactivex import operators as ops
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.depth.utils import make_colored_cloud
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()


class Config(ModuleConfig):
    camera_name: str = "camera"       # must match ZEDCamera.config.camera_name
    world_frame: str = "world"
    min_depth: float = 0.3
    max_depth: float = 8.0
    stride: int = 2
    max_freq: float = 10.0
    tf_timeout: float = 1.0           # temporal tolerance for TF lookup (seconds)


class ZEDNavBridge(Module):
    """ZED camera → world-frame PointCloud2 + Odometry for the nav pipeline.

    Takes depth + colour from ZEDCamera, backprojects to 3D in depth-optical
    frame, transforms to world using VIO pose, and emits on ports that match
    VoxelGridMapper / RayTracingVoxelMap without remappings.
    """

    config: Config

    color_image: In[Image]
    depth_image: In[Image]
    depth_camera_info: In[CameraInfo]

    lidar: Out[PointCloud2]
    odometry: Out[Odometry]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._latest_depth: Image | None = None
        self._latest_info: CameraInfo | None = None
        # VIO pose: world → camera_link (updated every frame)
        self._vio_tf: Any | None = None
        # Static extrinsics: camera_link → depth_optical_frame (cached once)
        self._R_cam_opt: np.ndarray | None = None   # (3,3) float32
        self._t_cam_opt: np.ndarray | None = None   # (3,)  float32

    @property
    def _camera_link(self) -> str:
        return f"{self.config.camera_name}_link"

    @property
    def _depth_optical_frame(self) -> str:
        return f"{self.config.camera_name}_depth_optical_frame"

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.depth_image.subscribe(self._on_depth)))
        self.register_disposable(Disposable(self.depth_camera_info.subscribe(self._on_info)))
        self.register_disposable(
            Disposable(
                backpressure(
                    self.color_image.pure_observable().pipe(
                        ops.throttle_with_timeout(1.0 / self.config.max_freq)
                    )
                ).subscribe(self._on_color)
            )
        )

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_depth(self, img: Image) -> None:
        with self._lock:
            self._latest_depth = img

    def _on_info(self, info: CameraInfo) -> None:
        with self._lock:
            self._latest_info = info

    def _refresh_tf(self, ts: float) -> None:
        # 1-hop VIO pose: ZEDCamera publishes world→camera_link directly.
        # Retry every frame — no 5-second backoff.
        tf = self.tf.get(
            self.config.world_frame,
            self._camera_link,
            ts,
            self.config.tf_timeout,
        )
        if tf is not None:
            self._vio_tf = tf

        # Static extrinsics: only look up once (never change after camera init).
        if self._R_cam_opt is None:
            tf_ext = self.tf.get(
                self._camera_link,
                self._depth_optical_frame,
                ts,
                self.config.tf_timeout,
            )
            if tf_ext is not None:
                self._R_cam_opt = tf_ext.rotation.to_rotation_matrix().astype(np.float32)
                tr = tf_ext.translation
                self._t_cam_opt = np.array([tr.x, tr.y, tr.z], dtype=np.float32)
                logger.info(
                    "ZEDNavBridge: cached %s→%s extrinsics",
                    self._camera_link, self._depth_optical_frame,
                )

    def _on_color(self, color: Image) -> None:
        with self._lock:
            depth = self._latest_depth
            info = self._latest_info

        if depth is None:
            return

        self._refresh_tf(color.ts)

        try:
            xyz_opt, colors = self._backproject(color, depth, info)
        except Exception:
            logger.exception("ZEDNavBridge: backprojection failed")
            return

        if len(xyz_opt) == 0:
            return

        xyz_world = self._to_world(xyz_opt)
        if xyz_world is None:
            return

        self.lidar.publish(
            make_colored_cloud(xyz_world, colors, self.config.world_frame, color.ts)
        )

        if self._vio_tf is not None:
            self.odometry.publish(self._tf_to_odometry(self._vio_tf, color.ts))

    def _backproject(
        self, color: Image, depth: Image, info: CameraInfo | None
    ) -> tuple[np.ndarray, np.ndarray]:
        rgb = color.to_rgb().data
        if hasattr(rgb, "get"):
            rgb = rgb.get()
        H, W = rgb.shape[:2]

        depth_data = depth.data
        if hasattr(depth_data, "get"):
            depth_data = depth_data.get()
        if depth_data.ndim == 3:
            depth_data = depth_data[:, :, 0]
        depth_np = depth_data.astype(np.float32)

        # ZED returns metres; convert mm only if median is clearly in mm range.
        nonzero = depth_np[depth_np > 0]
        if nonzero.size and np.nanmedian(nonzero) > 100:
            depth_np /= 1000.0

        if info is not None:
            K = info.get_K_matrix()
            fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        if info is None or fx == 0:
            f = max(H, W) / 2.0
            fx = fy = f
            cx, cy = W / 2.0, H / 2.0

        s = self.config.stride
        us = np.arange(0, W, s, dtype=np.float32)
        vs = np.arange(0, H, s, dtype=np.float32)
        uu, vv = np.meshgrid(us, vs)
        depth_sub = depth_np[::s, ::s]

        valid = (
            np.isfinite(depth_sub)
            & (depth_sub > self.config.min_depth)
            & (depth_sub < self.config.max_depth)
        )
        uu, vv, dd = uu[valid], vv[valid], depth_sub[valid]

        if len(dd) == 0:
            return np.zeros((0, 3), np.float32), np.zeros((0, 3), np.float32)

        # Points in depth-optical frame: Z forward, X right, Y down.
        xyz_opt = np.column_stack(
            [(uu - cx) * dd / fx, (vv - cy) * dd / fy, dd]
        ).astype(np.float32)

        ui = uu.astype(np.int32).clip(0, W - 1)
        vi = vv.astype(np.int32).clip(0, H - 1)
        colors = rgb[vi, ui, :3].astype(np.float32) / 255.0

        return xyz_opt, colors

    def _to_world(self, xyz_opt: np.ndarray) -> np.ndarray | None:
        tf = self._vio_tf
        if tf is None:
            return None

        R_world_cam = tf.rotation.to_rotation_matrix().astype(np.float32)
        t_world_cam = np.array(
            [tf.translation.x, tf.translation.y, tf.translation.z], dtype=np.float32
        )

        # depth-optical → camera_link (static extrinsics, cached)
        if self._R_cam_opt is not None:
            xyz_cam = xyz_opt @ self._R_cam_opt.T + self._t_cam_opt
        else:
            xyz_cam = xyz_opt  # approximation until extrinsics are cached

        # camera_link → world (VIO pose, updated every frame)
        return xyz_cam @ R_world_cam.T + t_world_cam

    def _tf_to_odometry(self, tf: Any, ts: float) -> Odometry:
        p = tf.translation
        q = tf.rotation
        return Odometry(
            ts=ts,
            frame_id=self.config.world_frame,
            child_frame_id=self._camera_link,
            pose=Pose(
                position=Vector3(p.x, p.y, p.z),
                orientation=Quaternion(q.x, q.y, q.z, q.w),
            ),
        )

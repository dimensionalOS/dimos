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

ZEDCamera's _tracking_transform publishes two different TF frames depending on
whether base_transform is set:
  - base_transform=None  → world → camera_link  (1-hop, what we want)
  - base_transform=*     → world → base_link     (then base_link → camera_link separately)

This module handles BOTH cases: it tries world→camera_link first, then falls back
to world→base_link + base_link→camera_link so the nav pipeline works regardless of
how ZEDCamera is configured.

Static extrinsics (camera_link → depth_optical_frame) are looked up once after
startup and cached for the lifetime of the module.

Outputs:
    lidar:    Out[PointCloud2]  world-frame cloud — feeds VoxelGridMapper directly
    odometry: Out[Odometry]    VIO pose          — feeds RayTracingVoxelMap
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
    base_frame: str = "base_link"     # must match ZEDCamera.config.base_frame_id
    world_frame: str = "world"
    min_depth: float = 0.3
    max_depth: float = 8.0
    stride: int = 2
    max_freq: float = 10.0
    tf_timeout: float = 1.0


class ZEDNavBridge(Module):
    """ZED depth + colour → world-frame PointCloud2 + Odometry for nav pipeline.

    Use alongside ZEDCamera with base_transform=None so world→camera_link is a
    direct single-hop transform. Falls back to world→base_link→camera_link when
    base_transform is set.

    Output ports match VoxelGridMapper / RayTracingVoxelMap — no remappings needed.
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
        # VIO pose: world → camera_link (or world → base_link composed below)
        self._vio_tf: Any | None = None
        # Static extrinsics camera_link → depth_optical_frame (cached once)
        self._R_cam_opt: np.ndarray | None = None
        self._t_cam_opt: np.ndarray | None = None
        self._frames_without_tf: int = 0

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

    def _resolve_vio_tf(self, ts: float) -> Any | None:
        """Get world→camera_link transform. Tries direct 1-hop first, then 2-hop via base_link."""
        tol = self.config.tf_timeout

        # Primary: world → camera_link (works when ZEDCamera.base_transform=None)
        tf = self.tf.get(self.config.world_frame, self._camera_link, ts, tol)
        if tf is not None:
            return tf

        # Fallback: world → base_link → camera_link (works when base_transform is set)
        tf_wb = self.tf.get(self.config.world_frame, self.config.base_frame, ts, tol)
        tf_bc = self.tf.get(self.config.base_frame, self._camera_link, ts, tol)
        if tf_wb is not None and tf_bc is not None:
            composed = tf_wb + tf_bc
            composed.frame_id = self.config.world_frame
            composed.child_frame_id = self._camera_link
            return composed

        return None

    def _refresh_tf(self, ts: float) -> None:
        tf = self._resolve_vio_tf(ts)
        if tf is not None:
            self._vio_tf = tf
            self._frames_without_tf = 0
        else:
            self._frames_without_tf += 1
            # Log once after 10 consecutive misses (not every frame) to avoid spam.
            if self._frames_without_tf == 10:
                available = self.tf.get_frames()
                logger.warning(
                    "ZEDNavBridge: no VIO transform after 10 frames. "
                    "Available TF frames: %s. "
                    "Set base_transform=None in ZEDCamera blueprint if not on a robot.",
                    sorted(available),
                )

        # Static extrinsics: camera_link → depth_optical_frame (look up once).
        if self._R_cam_opt is None:
            tf_ext = self.tf.get(self._camera_link, self._depth_optical_frame, ts, 2.0)
            if tf_ext is not None:
                self._R_cam_opt = tf_ext.rotation.to_rotation_matrix().astype(np.float32)
                tr = tf_ext.translation
                self._t_cam_opt = np.array([tr.x, tr.y, tr.z], dtype=np.float32)
                logger.info("ZEDNavBridge: extrinsics cached (%s→%s)", self._camera_link, self._depth_optical_frame)

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

        nonzero = depth_np[depth_np > 0]
        if nonzero.size and np.nanmedian(nonzero) > 100:
            depth_np /= 1000.0

        fx = fy = cx = cy = 0.0
        if info is not None:
            K = info.get_K_matrix()
            fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        if fx == 0:
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

        # depth-optical → camera_link (static; use identity until extrinsics arrive)
        if self._R_cam_opt is not None:
            xyz_cam = xyz_opt @ self._R_cam_opt.T + self._t_cam_opt
        else:
            xyz_cam = xyz_opt

        R_wc = tf.rotation.to_rotation_matrix().astype(np.float32)
        t_wc = np.array(
            [tf.translation.x, tf.translation.y, tf.translation.z], dtype=np.float32
        )
        return xyz_cam @ R_wc.T + t_wc

    def _tf_to_odometry(self, tf: Any, ts: float) -> Odometry:
        p, q = tf.translation, tf.rotation
        return Odometry(
            ts=ts,
            frame_id=self.config.world_frame,
            child_frame_id=self._camera_link,
            pose=Pose(
                position=Vector3(p.x, p.y, p.z),
                orientation=Quaternion(q.x, q.y, q.z, q.w),
            ),
        )

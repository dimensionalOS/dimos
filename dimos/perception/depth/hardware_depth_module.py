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

"""Hardware depth → coloured PointCloud2 for cameras with built-in stereo (ZED, RealSense).

Drop-in replacement for MonocularDepthModule; publishes the same frame_cloud port.
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
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.depth.utils import make_colored_cloud, to_world
from dimos.spec.depth import DepthBackprojector
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()


class Config(ModuleConfig):
    min_depth: float = 0.3
    max_depth: float = 5.0          # ZED stereo reliable to ~5 m; beyond this gets noisy
    stride: int = 2                 # skip every other pixel — halves noise sources, keeps coverage
    edge_grad_threshold: float = 0.1  # mask depth pixels at sharp discontinuities (flying pixels)
    max_freq: float = 10.0
    camera_frame: str = "camera_optical"
    world_frame: str = "world"
    tf_timeout: float = 0.2


class HardwareDepthModule(Module, DepthBackprojector):
    """Backprojects hardware depth + RGB into a coloured PointCloud2 per frame."""

    config: Config

    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]
    frame_cloud: Out[PointCloud2]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._latest_depth: Image | None = None
        self._latest_info: CameraInfo | None = None
        self._lock = threading.Lock()
        self._last_tf = None
        self._tf_last_attempt = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.depth_image.subscribe(self._on_depth)))
        self.register_disposable(Disposable(self.camera_info.subscribe(self._on_camera_info)))
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

    def _on_camera_info(self, info: CameraInfo) -> None:
        with self._lock:
            self._latest_info = info

    def _on_color(self, color: Image) -> None:
        with self._lock:
            depth = self._latest_depth
            info = self._latest_info

        if depth is None:
            logger.debug("HardwareDepthModule: no depth yet, skipping frame")
            return

        try:
            points, colors = self._backproject(color, depth, info)
        except Exception:
            logger.exception("HardwareDepthModule: backprojection failed")
            return

        if len(points) == 0:
            return

        pts_world, frame_id, self._last_tf, self._tf_last_attempt = to_world(
            points, color.ts, self.tf,
            self.config.world_frame, self.config.camera_frame, self.config.tf_timeout,
            self._last_tf, self._tf_last_attempt, logger,
        )
        if pts_world is not None:
            self.frame_cloud.publish(make_colored_cloud(pts_world, colors, frame_id, color.ts))

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

        # ZED SDK returns metres; other cameras may use mm
        if np.nanmedian(depth_np[depth_np > 0]) > 100:
            depth_np /= 1000.0

        if info is not None:
            K = info.get_K_matrix()
            fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        if info is None or fx == 0 or fy == 0:
            f = max(H, W) / 2.0
            fx = fy = f
            cx, cy = W / 2.0, H / 2.0

        stride = self.config.stride
        us = np.arange(0, W, stride, dtype=np.float32)
        vs = np.arange(0, H, stride, dtype=np.float32)
        uu, vv = np.meshgrid(us, vs)

        depth_sub = depth_np[::stride, ::stride]

        # Mask depth edges: large gradients indicate depth discontinuities where
        # flying pixels occur (one pixel sees both foreground and background).
        gx = np.abs(np.gradient(depth_sub, axis=1))
        gy = np.abs(np.gradient(depth_sub, axis=0))
        edge_mask = (gx + gy) < (self.config.edge_grad_threshold * depth_sub + 1e-6)

        valid = (
            np.isfinite(depth_sub)
            & (depth_sub > self.config.min_depth)
            & (depth_sub < self.config.max_depth)
            & edge_mask
        )
        uu, vv, dd = uu[valid], vv[valid], depth_sub[valid]

        if len(dd) == 0:
            return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.float32)

        points = np.column_stack(
            [(uu - cx) * dd / fx, (vv - cy) * dd / fy, dd]
        ).astype(np.float32)

        ui = uu.astype(np.int32).clip(0, W - 1)
        vi = vv.astype(np.int32).clip(0, H - 1)
        colors = rgb[vi, ui, :3].astype(np.float32) / 255.0

        return points, colors

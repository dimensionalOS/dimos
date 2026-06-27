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

"""DepthAnythingV2 monocular depth → coloured PointCloud2 per camera frame."""

from __future__ import annotations

import threading
import time
from typing import Any, Callable

import numpy as np
import open3d as o3d
import open3d.core as o3c
from reactivex import operators as ops
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.spec.depth import DepthBackprojector
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()


def _load_depth_anything_v2(model_name: str, device: str) -> Callable[..., dict[str, Any]]:
    import torch
    import torch.nn.functional as F
    from PIL import Image as PILImage
    from transformers import AutoImageProcessor, AutoModelForDepthEstimation

    processor = AutoImageProcessor.from_pretrained(model_name)
    model = AutoModelForDepthEstimation.from_pretrained(model_name).to(device).eval()

    @torch.no_grad()  # type: ignore[misc]
    def _infer(rgb_t: Any, _K_t: Any) -> dict[str, Any]:
        H, W = rgb_t.shape[-2], rgb_t.shape[-1]
        rgb_np = (rgb_t[0].permute(1, 2, 0).cpu().numpy() * 255).astype("uint8")
        inputs = processor(images=PILImage.fromarray(rgb_np), return_tensors="pt")
        inputs = {k: v.to(device) for k, v in inputs.items()}
        depth = model(**inputs).predicted_depth  # type: ignore[attr-defined]
        if depth.ndim == 2:
            depth = depth.unsqueeze(0).unsqueeze(0)
        elif depth.ndim == 3:
            depth = depth.unsqueeze(1)
        return {"depth": F.interpolate(depth.float(), size=(H, W), mode="bilinear", align_corners=False)}

    return _infer


class Config(ModuleConfig):
    model_name: str = "depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf"
    min_depth: float = 0.3
    max_depth: float = 8.0
    stride: int = 4
    max_freq: float = 5.0
    camera_frame: str = "camera_optical"
    world_frame: str = "world"
    tf_timeout: float = 0.2
    device: str | None = None


class MonocularDepthModule(Module, DepthBackprojector):
    """Runs DepthAnythingV2 on color_image and publishes frame_cloud in world frame."""

    config: Config

    color_image: In[Image]
    camera_info: In[CameraInfo]

    frame_cloud: Out[PointCloud2]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._infer_fn: Callable[..., dict[str, Any]] | None = None
        self._device: str = "cpu"
        self._model_lock = threading.Lock()
        self._latest_camera_info: CameraInfo | None = None
        self._camera_info_lock = threading.Lock()
        self._last_tf = None
        self._tf_last_attempt = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        self._load_model()
        self.register_disposable(
            Disposable(self.camera_info.subscribe(self._on_camera_info))
        )
        self.register_disposable(
            Disposable(
                backpressure(
                    self.color_image.pure_observable().pipe(
                        ops.throttle_with_timeout(1.0 / self.config.max_freq)
                    )
                ).subscribe(self._on_image)
            )
        )

    @rpc
    def stop(self) -> None:
        super().stop()

    def _load_model(self) -> None:
        import torch

        device = self.config.device
        if device is None:
            if torch.cuda.is_available():
                device = "cuda"
            elif torch.backends.mps.is_available():
                device = "mps"
            else:
                device = "cpu"
        self._device = device

        logger.info("MonocularDepthModule: loading %s on %s", self.config.model_name, device)
        t0 = time.perf_counter()
        with self._model_lock:
            self._infer_fn = _load_depth_anything_v2(self.config.model_name, device)
        logger.info("MonocularDepthModule: model ready in %.1fs", time.perf_counter() - t0)

    def _on_camera_info(self, info: CameraInfo) -> None:
        with self._camera_info_lock:
            self._latest_camera_info = info

    def _on_image(self, image: Image) -> None:
        t0 = time.perf_counter()
        try:
            points, colors = self._run_inference(image)
        except Exception:
            logger.exception("MonocularDepthModule: inference failed")
            return
        logger.info("MonocularDepthModule: %.2fs/frame  %d pts  device=%s",
                    time.perf_counter() - t0, len(points), self._device)

        points_world, frame_id = self._to_world(points, image.ts)
        if points_world is not None and len(points_world) > 0:
            self.frame_cloud.publish(
                _make_colored_cloud(points_world, colors, frame_id, image.ts)
            )

    def _run_inference(self, image: Image) -> tuple[np.ndarray, np.ndarray]:
        import torch

        rgb = image.to_rgb().data
        if hasattr(rgb, "get"):
            rgb = rgb.get()
        H, W = rgb.shape[:2]

        rgb_t = (
            torch.from_numpy(rgb).permute(2, 0, 1).unsqueeze(0).float().to(self._device) / 255.0
        )

        with self._camera_info_lock:
            info = self._latest_camera_info

        K_t = None
        if info is not None:
            import torch as _torch
            K_t = _torch.from_numpy(info.get_K_matrix().astype(np.float32)).unsqueeze(0).to(self._device)

        with self._model_lock:
            preds = self._infer_fn(rgb_t, K_t)  # type: ignore[misc]

        depth_np: np.ndarray = preds["depth"].squeeze().cpu().numpy()
        return self._backproject(depth_np, rgb, info, H, W)

    def _backproject(
        self,
        depth: np.ndarray,
        rgb: np.ndarray,
        info: CameraInfo | None,
        H: int,
        W: int,
    ) -> tuple[np.ndarray, np.ndarray]:
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
        dd = depth[::stride, ::stride]

        valid = np.isfinite(dd) & (dd > self.config.min_depth) & (dd < self.config.max_depth)
        uu, vv, dd = uu[valid], vv[valid], dd[valid]

        if len(dd) == 0:
            return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.float32)

        points = np.column_stack(
            [(uu - cx) * dd / fx, (vv - cy) * dd / fy, dd]
        ).astype(np.float32)

        ui = uu.astype(np.int32).clip(0, W - 1)
        vi = vv.astype(np.int32).clip(0, H - 1)
        colors = rgb[vi, ui, :3].astype(np.float32) / 255.0

        return points, colors

    def _to_world(self, points_cam: np.ndarray, ts: float) -> tuple[np.ndarray | None, str]:
        if len(points_cam) == 0:
            return points_cam, self.config.world_frame

        now = time.monotonic()
        # When TF has never been found, throttle retries to 5s intervals so the
        # TF module doesn't spam a warning on every frame (e.g. trial/no-odometry mode).
        # Once TF is known, always refresh to track robot movement.
        if self._last_tf is not None or now - self._tf_last_attempt >= 5.0:
            self._tf_last_attempt = now
            tf = self.tf.get(
                self.config.world_frame, self.config.camera_frame, ts, self.config.tf_timeout
            )
            if tf is not None:
                self._last_tf = tf

        if self._last_tf is None:
            # No odometry: camera is the world origin. Use world frame_id so the
            # Rerun bridge doesn't attach a different TF than global_map uses.
            return points_cam, self.config.world_frame

        R = self._last_tf.rotation.to_rotation_matrix()
        t = np.array(
            [self._last_tf.translation.x, self._last_tf.translation.y, self._last_tf.translation.z],
            dtype=np.float32,
        )
        return (points_cam @ R.T).astype(np.float32) + t, self.config.world_frame


def _make_colored_cloud(
    points: np.ndarray,
    colors: np.ndarray,
    frame_id: str,
    ts: float,
) -> PointCloud2:
    pcd_t = o3d.t.geometry.PointCloud()
    pcd_t.point["positions"] = o3c.Tensor(points, dtype=o3c.float32)
    if len(colors) == len(points):
        pcd_t.point["colors"] = o3c.Tensor(colors, dtype=o3c.float32)
    return PointCloud2(pcd_t, frame_id=frame_id, ts=ts)

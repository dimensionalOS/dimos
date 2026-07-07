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

"""Stereo depth → per-frame cloud + persistent global map for CostMapper."""

from __future__ import annotations

import threading
from typing import Any

import numpy as np
from reactivex.disposable import Disposable
from scipy.ndimage import sobel

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# ── Constants ─────────────────────────────────────────────────────────────────

_VOFF  = np.int64(100_000)
_VMASK = np.int64(0x3FFFF)

# Optical frame (X=right, Y=down, Z=depth) → camera_link (X=fwd, Y=left, Z=up)
_R_OPT_TO_LINK = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float32)


# ── Helpers ───────────────────────────────────────────────────────────────────

def _pack(vkeys: np.ndarray) -> np.ndarray:
    v = (vkeys.astype(np.int64) + _VOFF) & _VMASK
    return (v[:, 0] << 36) | (v[:, 1] << 18) | v[:, 2]


def _gradient_mask(depth: np.ndarray, threshold: float) -> np.ndarray:
    """True where depth is valid and Sobel gradient magnitude is below threshold (m/px)."""
    valid   = np.isfinite(depth)
    depth_f = np.where(valid, depth, 0.0).astype(np.float64)
    grad    = np.hypot(sobel(depth_f, axis=1), sobel(depth_f, axis=0))
    return valid & (grad < threshold)


def _raycast_free_keys(
    xyz_rel: np.ndarray,
    vox: float,
    n_rays: int = 200,
    max_steps: int = 40,
) -> np.ndarray:
    """Keys of free-space voxels on rays from camera origin to observed surfaces (xyz_rel is camera-relative)."""
    if len(xyz_rel) == 0:
        return np.array([], dtype=np.int64)

    idx  = np.random.choice(len(xyz_rel), min(n_rays, len(xyz_rel)), replace=False)
    dirs = xyz_rel[idx].astype(np.float32)
    dist = np.linalg.norm(dirs, axis=1, keepdims=True)
    dist = np.where(dist < 1e-6, 1e-6, dist)
    dirs /= dist

    steps = np.linspace(0.1, 0.9, max_steps, dtype=np.float32)
    pts   = dirs[:, None, :] * (dist[:, None, :] * steps[None, :, None])
    pts   = pts.reshape(-1, 3)
    return np.unique(_pack(np.floor(pts / vox).astype(np.int32)))


# ── Module ────────────────────────────────────────────────────────────────────

class Config(ModuleConfig):
    min_depth: float          = 0.3
    max_depth: float          = 8.0
    gradient_threshold: float = 0.30    # m/px; above → depth edge artifact → reject
    vox_size: float           = 0.020   # per-frame dedup voxel (2 cm)
    global_vox_size: float    = 0.040   # accumulation voxel (4 cm, absorbs pose jitter)
    floor_margin: float       = 0.04    # metres above detected floor to keep points
    max_global_pts: int       = 200_000
    publish_every: int        = 3       # emit global_map every N frames
    world_frame: str          = "world"
    camera_frame: str         = "camera_depth_optical_frame"
    base_frame: str           = "base_link"
    tf_timeout: float         = 0.2


class StereoPointCloud(Module):
    """Gradient filter → backproject → floor removal → voxel dedup → ray-cast ghost clearing → global_map."""

    config: Config

    depth_image:       In[Image]
    depth_camera_info: In[CameraInfo]

    frame_cloud: Out[PointCloud2]
    global_map:  Out[PointCloud2]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock                          = threading.Lock()
        self._latest_info: CameraInfo | None = None
        self._last_tf                       = None
        self._floor_buf: list[float]        = []
        self._acc_pts: np.ndarray           = np.empty((0, 3), dtype=np.float32)
        self._frame                         = 0

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.depth_camera_info.subscribe(self._on_info)))
        self.register_disposable(Disposable(self.depth_image.subscribe(self._on_depth)))

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_info(self, info: CameraInfo) -> None:
        with self._lock:
            self._latest_info = info

    def _on_depth(self, img: Image) -> None:
        with self._lock:
            info = self._latest_info

        depth = img.data
        if hasattr(depth, "get"):
            depth = depth.get()
        if depth.ndim == 3:
            depth = depth[:, :, 0]
        depth = depth.astype(np.float32)
        if np.nanmedian(depth[depth > 0]) > 100:
            depth /= 1000.0

        H, W = depth.shape

        if info is not None:
            K = info.get_K_matrix()
            fx, fy, cx, cy = float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2])
        else:
            fx = fy = float(max(H, W)) / 2.0
            cx, cy  = W / 2.0, H / 2.0

        mask = (
            _gradient_mask(depth, self.config.gradient_threshold)
            & (depth > self.config.min_depth)
            & (depth < self.config.max_depth)
        )
        if not mask.any():
            return

        uu, vv  = np.meshgrid(np.arange(W, dtype=np.float32), np.arange(H, dtype=np.float32))
        dd      = depth[mask]
        xyz_opt = np.column_stack([
            (uu[mask] - cx) * dd / fx,
            (vv[mask] - cy) * dd / fy,
            dd,
        ]).astype(np.float32)
        xyz_cam = xyz_opt @ _R_OPT_TO_LINK.T

        tf = self.tf.get(
            self.config.world_frame, self.config.camera_frame, img.ts, self.config.tf_timeout
        )
        if tf is not None:
            self._last_tf = tf
        if self._last_tf is not None:
            R = self._last_tf.rotation.to_rotation_matrix().astype(np.float32)
            t = np.array(
                [self._last_tf.translation.x,
                 self._last_tf.translation.y,
                 self._last_tf.translation.z],
                dtype=np.float32,
            )
        else:
            R, t = np.eye(3, dtype=np.float32), np.zeros(3, dtype=np.float32)

        xyz_world = (xyz_cam @ R.T + t).astype(np.float32)

        # TF-primary floor detection; falls back to rolling low-Z percentile
        base_tf = self.tf.get(
            self.config.world_frame, self.config.base_frame, img.ts, self.config.tf_timeout
        )
        if base_tf is not None:
            floor_z = float(base_tf.translation.z)
        else:
            bot = xyz_world[xyz_world[:, 2] < np.percentile(xyz_world[:, 2], 10)]
            if len(bot):
                self._floor_buf.append(float(np.median(bot[:, 2])))
                if len(self._floor_buf) > 30:
                    self._floor_buf.pop(0)
            floor_z = float(np.percentile(self._floor_buf, 20)) if len(self._floor_buf) >= 5 else None

        vk      = np.floor(xyz_world / self.config.vox_size).astype(np.int32)
        _, ui   = np.unique(_pack(vk), return_index=True)
        xyz_vox = xyz_world[ui]
        if floor_z is not None:
            xyz_vox = xyz_vox[xyz_vox[:, 2] > floor_z + self.config.floor_margin]
        if len(xyz_vox) == 0:
            return

        self.frame_cloud.publish(
            PointCloud2.from_numpy(xyz_vox, frame_id=self.config.world_frame, timestamp=img.ts)
        )

        xyz_rel  = xyz_vox - t
        pts_snap = None

        with self._lock:
            if len(self._acc_pts) > 0:
                free = _raycast_free_keys(xyz_rel, self.config.global_vox_size)
                if len(free):
                    acc_rel  = self._acc_pts - t
                    keys_acc = _pack(np.floor(acc_rel / self.config.global_vox_size).astype(np.int32))
                    self._acc_pts = self._acc_pts[~np.isin(keys_acc, free)]

            self._acc_pts = (
                np.vstack([self._acc_pts, xyz_vox])
                if len(self._acc_pts) else xyz_vox.copy()
            )
            vk_g    = np.floor(self._acc_pts / self.config.global_vox_size).astype(np.int32)
            _, ui_g = np.unique(_pack(vk_g), return_index=True)
            self._acc_pts = self._acc_pts[ui_g]

            if len(self._acc_pts) > self.config.max_global_pts:
                keep = np.random.choice(len(self._acc_pts), self.config.max_global_pts, replace=False)
                self._acc_pts = self._acc_pts[keep]

            self._frame += 1
            if self._frame % self.config.publish_every == 0 and len(self._acc_pts):
                pts_snap = self._acc_pts.copy()

        if pts_snap is not None:
            self.global_map.publish(
                PointCloud2.from_numpy(pts_snap, frame_id=self.config.world_frame, timestamp=img.ts)
            )

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

"""Stereo depth → per-frame cloud + persistent global map for CostMapper.

Pose source (in priority order):
  1. TF lookup (world → camera_color_optical_frame) — used when a robot TF stack is running.
  2. ICP fallback — rotation-decoupled point-cloud odometry for translation when TF is
     unavailable. Rotation is assumed near-identity (no IMU in Module context).

Global map frame:
  - TF mode:  full world frame — TF translation is reliable so no rotation-only trick needed.
  - ICP mode: rotation-only frame (xyz_world - t) — strips ICP translation to prevent
    ±2 cm ICP centroid noise from accumulating duplicate voxels, matching the behaviour
    of the standalone realsense_stereo_nav.py script.

Floor calibration:
  Histogram-based one-shot estimator (ported from realsense_stereo_nav.py): skips the
  first SKIP_FRAMES, then histograms candidate floor-point Z values over CALIB_FRAMES
  and takes the mode.  More robust than a rolling percentile when the floor is partially
  occluded or when the camera tilts slightly at startup.
"""

from __future__ import annotations

import threading
from typing import Any

import numpy as np
from reactivex.disposable import Disposable
from scipy.ndimage import sobel
from scipy.spatial import cKDTree

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


# ── Helpers ───────────────────────────────────────────────────────────────────

def _pack(vkeys: np.ndarray) -> np.ndarray:
    v = (vkeys.astype(np.int64) + _VOFF) & _VMASK
    return (v[:, 0] << 36) | (v[:, 1] << 18) | v[:, 2]


def _gradient_mask(depth: np.ndarray, threshold: float) -> np.ndarray:
    valid   = np.isfinite(depth)
    depth_f = np.where(valid, depth, 0.0).astype(np.float64)
    grad    = np.hypot(sobel(depth_f, axis=1), sobel(depth_f, axis=0))
    return valid & (grad < threshold)


def _raycast_free_keys(
    origin: np.ndarray,
    surface_pts: np.ndarray,
    vox: float,
    n_rays: int = 400,
) -> np.ndarray:
    """World-frame voxel keys on rays from origin to surface_pts.

    Fully vectorised. Stops 1.5 voxels short of each surface so the surface
    voxel itself is never cleared. Keys use floor(pts/vox) world voxelisation —
    directly comparable with accumulated-point world-frame keys.
    """
    if len(surface_pts) == 0:
        return np.array([], dtype=np.int64)

    n    = min(len(surface_pts), n_rays)
    idx  = (np.random.choice(len(surface_pts), n, replace=False)
            if len(surface_pts) > n else np.arange(n))
    vecs = (surface_pts[idx] - origin).astype(np.float32)
    dist = np.linalg.norm(vecs, axis=1)
    ok   = dist > vox * 2
    vecs, dist = vecs[ok], dist[ok]
    if not len(vecs):
        return np.array([], dtype=np.int64)

    dirs      = vecs / dist[:, None]
    max_steps = int(np.ceil(dist.max() / vox))
    t_vals    = np.arange(max_steps, dtype=np.float32) * vox
    t_end     = (dist - vox * 1.5)[:, None]
    valid_m   = (t_vals[None, :] >= 0) & (t_vals[None, :] < t_end)

    pts = origin + dirs[:, None, :] * t_vals[None, :, None]
    vk  = np.floor(pts.reshape(-1, 3) / vox).astype(np.int32)
    return np.unique(_pack(vk)[valid_m.ravel()])


# ── Floor calibration ─────────────────────────────────────────────────────────

class _FloorCalibrator:
    """One-shot floor height estimator.

    Skips SKIP_FRAMES for sensor settle, then histograms candidate floor-point
    depths over CALIB_FRAMES and takes the mode bin centre. More robust than a
    rolling percentile when the floor is partially visible or the camera tilts.
    """

    SKIP_FRAMES  = 30
    CALIB_FRAMES = 30
    BIN_M        = 0.02

    def __init__(self) -> None:
        self._frame   = 0
        self._samples: list[float] = []
        self.floor_z:   float | None = None
        self.cam_height: float | None = None

    @property
    def ready(self) -> bool:
        return self.floor_z is not None

    def update(self, xyz_cam: np.ndarray) -> None:
        """xyz_cam: (N,3) points in camera_link frame (Z = up)."""
        self._frame += 1
        if self.ready or self._frame <= self.SKIP_FRAMES:
            return

        z    = xyz_cam[:, 2]
        dist = np.linalg.norm(xyz_cam[:, :2], axis=1)
        mask = (z < -0.30) & (dist < 4.0)
        if mask.sum() < 200:
            return

        z_floor = z[mask]
        lo, hi  = z_floor.min(), z_floor.max()
        if hi - lo < self.BIN_M:
            return
        bins          = np.arange(lo, hi + self.BIN_M, self.BIN_M)
        counts, edges = np.histogram(z_floor, bins=bins)
        mode_z        = float(edges[int(np.argmax(counts))] + self.BIN_M / 2)
        self._samples.append(mode_z)

        if len(self._samples) >= self.CALIB_FRAMES:
            self.floor_z    = float(np.median(self._samples))
            self.cam_height = -self.floor_z
            logger.info(f"StereoPointCloud: floor calibrated — camera {self.cam_height:.3f} m above floor")


# ── ICP translation odometry (fallback when TF unavailable) ──────────────────

class _PointCloudOdometry:
    """Frame-to-frame translation via rotation-decoupled ICP.

    Rotation is assumed near-identity (no IMU available in Module context).
    Used only when TF lookup fails — on a robot TF is always preferred.
    """

    ITERS    = 4
    MAX_DIST = 0.40
    N_SRC    = 1_500
    N_DST    = 8_000
    N_STORE  = 10_000

    def __init__(self) -> None:
        self._t          = np.zeros(3, dtype=np.float32)
        self._lock       = threading.Lock()
        self._prev_world: np.ndarray | None = None

    @property
    def t(self) -> np.ndarray:
        with self._lock:
            return self._t.copy()

    def update(self, xyz_cam: np.ndarray) -> np.ndarray:
        with self._lock:
            t_est = self._t.copy()

        if self._prev_world is None or len(xyz_cam) < 50:
            t_new = t_est
        else:
            n_src = min(self.N_SRC, len(xyz_cam))
            n_dst = min(self.N_DST, len(self._prev_world))
            src   = xyz_cam[np.random.choice(len(xyz_cam), n_src, replace=False)]
            dst   = self._prev_world[np.random.choice(len(self._prev_world), n_dst, replace=False)]
            tree  = cKDTree(dst)
            for _ in range(self.ITERS):
                dists, idx = tree.query(src + t_est, k=1, workers=1)
                mask = dists < self.MAX_DIST
                if mask.sum() < 30:
                    break
                delta = dst[idx[mask]].mean(axis=0) - (src[mask] + t_est).mean(axis=0)
                t_est = (t_est + 0.7 * delta).astype(np.float32)
            t_new = t_est

        n_st = min(self.N_STORE, len(xyz_cam))
        self._prev_world = (xyz_cam[np.random.choice(len(xyz_cam), n_st, replace=False)] + t_new).astype(np.float32)

        with self._lock:
            self._t = t_new
        return t_new


# ── Module ────────────────────────────────────────────────────────────────────

class Config(ModuleConfig):
    min_depth: float          = 0.3
    max_depth: float          = 8.0
    gradient_threshold: float = 0.30
    vox_size: float           = 0.020
    global_vox_size: float    = 0.020
    floor_margin: float       = 0.03
    max_global_pts: int       = 200_000
    publish_every: int        = 3
    world_frame: str          = "world"
    camera_frame: str         = "camera_color_optical_frame"
    base_frame: str           = "base_link"
    tf_timeout: float         = 0.2


class StereoPointCloud(Module):
    """Gradient filter → backproject → floor removal → voxel dedup → ray-cast ghost clearing → global_map.

    Mirrors the realsense_stereo_nav.py standalone pipeline as a dimos Module.
    Uses TF for pose when available; falls back to ICP odometry with rotation-only
    global map frame to prevent translation-drift accumulation.
    """

    config: Config

    depth_image:       In[Image]
    depth_camera_info: In[CameraInfo]

    frame_cloud: Out[PointCloud2]
    global_map:  Out[PointCloud2]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock                           = threading.Lock()
        self._latest_info: CameraInfo | None = None
        self._last_tf                        = None
        self._floor_calib                    = _FloorCalibrator()
        self._odom                           = _PointCloudOdometry()
        self._acc_pts: np.ndarray            = np.empty((0, 3), dtype=np.float32)
        self._map_ready                      = False
        self._world_floor_z: float           = 0.0
        self._frame                          = 0

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

        # ── Pose: TF primary, ICP fallback ───────────────────────────────────
        tf = self.tf.get(
            self.config.world_frame, self.config.camera_frame, img.ts, self.config.tf_timeout
        )
        if tf is not None:
            self._last_tf = tf

        use_tf = self._last_tf is not None
        if use_tf:
            R = self._last_tf.rotation.to_rotation_matrix().astype(np.float32)
            t = np.array([
                self._last_tf.translation.x,
                self._last_tf.translation.y,
                self._last_tf.translation.z,
            ], dtype=np.float32)
            xyz_world = (xyz_opt @ R.T + t).astype(np.float32)
            # xyz_cam approximation for floor calibration (R ~ optical→link not critical here)
            xyz_cam = xyz_opt
        else:
            # ICP fallback: no rotation (no IMU), translation from ICP
            t         = self._odom.t
            xyz_world = (xyz_opt + t).astype(np.float32)
            xyz_cam   = xyz_opt

        # ── Floor calibration (histogram-based, camera-frame Z) ───────────────
        self._floor_calib.update(xyz_cam)

        cam_z = float(t[2]) if use_tf else 0.0

        # ── Per-frame voxel map with floor filter ─────────────────────────────
        if self._floor_calib.ready:
            floor_z_world = cam_z + self._floor_calib.floor_z if use_tf else self._floor_calib.floor_z
            keep = xyz_world[:, 2] > floor_z_world + self.config.floor_margin
        else:
            keep = (xyz_world[:, 2] - cam_z >= -1.4) & (xyz_world[:, 2] - cam_z <= 1.5)

        xyz_kept = xyz_world[keep]
        if not len(xyz_kept):
            self._frame += 1
            return

        vk       = np.floor(xyz_kept / self.config.vox_size).astype(np.int32)
        _, first = np.unique(_pack(vk), return_index=True)
        xyz_vox  = xyz_kept[first]

        self.frame_cloud.publish(
            PointCloud2.from_numpy(xyz_vox, frame_id=self.config.world_frame, timestamp=img.ts)
        )

        # ── ICP update (runs even when TF available, for future fallback) ─────
        if not use_tf and len(xyz_cam[keep]) >= 50:
            self._odom.update(xyz_cam[keep])

        # ── Global map ────────────────────────────────────────────────────────
        if not self._floor_calib.ready:
            self._frame += 1
            return

        if not self._map_ready:
            self._map_ready   = True
            self._world_floor_z = floor_z_world if self._floor_calib.ready else 0.0
            logger.info("StereoPointCloud: global map started")

        if use_tf:
            # Full world frame — TF translation is reliable
            xyz_for_map = xyz_vox[xyz_vox[:, 2] > self._world_floor_z + self.config.floor_margin]
        else:
            # Rotation-only frame — strips ICP translation drift
            xyz_ronly = xyz_vox - t
            vk_r      = np.floor(xyz_ronly / self.config.vox_size).astype(np.int32)
            _, fr     = np.unique(_pack(vk_r), return_index=True)
            xyz_ronly = xyz_ronly[fr]
            xyz_for_map = xyz_ronly[xyz_ronly[:, 2] > self._world_floor_z + self.config.floor_margin]

        pts_snap = None
        with self._lock:
            if len(self._acc_pts) > 0 and len(xyz_for_map) > 0:
                origin_for_clear = t if use_tf else np.zeros(3, dtype=np.float32)
                free = _raycast_free_keys(origin_for_clear, xyz_for_map, self.config.global_vox_size)
                if len(free):
                    keys_acc  = _pack(np.floor(self._acc_pts / self.config.global_vox_size).astype(np.int32))
                    self._acc_pts = self._acc_pts[~np.isin(keys_acc, free)]

            if len(xyz_for_map):
                self._acc_pts = (
                    np.vstack([self._acc_pts, xyz_for_map])
                    if len(self._acc_pts) else xyz_for_map.copy()
                )
                vk_g    = np.floor(self._acc_pts / self.config.global_vox_size).astype(np.int32)
                _, ui_g = np.unique(_pack(vk_g), return_index=True)
                self._acc_pts = self._acc_pts[ui_g]

                if len(self._acc_pts) > self.config.max_global_pts:
                    keep_idx = np.random.choice(len(self._acc_pts), self.config.max_global_pts, replace=False)
                    self._acc_pts = self._acc_pts[keep_idx]

            self._frame += 1
            if self._frame % self.config.publish_every == 0 and len(self._acc_pts):
                pts_snap = self._acc_pts.copy()

        if pts_snap is not None:
            self.global_map.publish(
                PointCloud2.from_numpy(pts_snap, frame_id=self.config.world_frame, timestamp=img.ts)
            )

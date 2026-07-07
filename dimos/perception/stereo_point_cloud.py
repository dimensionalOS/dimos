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

"""D435i depth → per-frame voxel cloud + persistent global map (Madgwick + ICP VIO)."""

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

# Optical frame (X=right, Y=down, Z=depth) → camera_link (X=fwd, Y=left, Z=up)
_R_OPT_TO_LINK = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float32)


# ── Helpers ───────────────────────────────────────────────────────────────────

def _pack(vkeys: np.ndarray) -> np.ndarray:
    v = (vkeys.astype(np.int64) + _VOFF) & _VMASK
    return (v[:, 0] << np.int64(36)) | (v[:, 1] << np.int64(18)) | v[:, 2]


def _gradient_mask(depth: np.ndarray, threshold: float) -> np.ndarray:
    valid   = np.isfinite(depth)
    depth_f = np.where(valid, depth, 0.0).astype(np.float64)
    grad    = np.hypot(sobel(depth_f, axis=1), sobel(depth_f, axis=0))
    return valid & (grad < threshold)


def _raycast_free_keys(surface_pts: np.ndarray, vox_size: float, n_rays: int = 400) -> np.ndarray:
    """Free-space voxel keys on rays from camera origin (rotation-only frame) to surface_pts."""
    origin = np.zeros(3, dtype=np.float32)
    n = min(len(surface_pts), n_rays)
    if n == 0:
        return np.empty(0, dtype=np.int64)
    idx   = np.random.choice(len(surface_pts), n, replace=False) if len(surface_pts) > n else np.arange(n)
    pts   = surface_pts[idx]
    vecs  = pts - origin
    dists = np.linalg.norm(vecs, axis=1)
    ok    = dists > vox_size * 2
    vecs, dists = vecs[ok], dists[ok]
    if not len(vecs):
        return np.empty(0, dtype=np.int64)
    dirs      = vecs / dists[:, None]
    max_steps = int(np.ceil(dists.max() / vox_size))
    t_vals    = np.arange(max_steps, dtype=np.float32) * vox_size
    t_end     = (dists - vox_size * 1.5)[:, None]
    valid_m   = (t_vals[None, :] >= 0) & (t_vals[None, :] < t_end)
    ray_pts   = origin + dirs[:, None, :] * t_vals[None, :, None]
    vk_flat   = np.floor(ray_pts.reshape(-1, 3) / vox_size).astype(np.int32)
    return np.unique(_pack(vk_flat)[valid_m.ravel()])


# ── Floor calibration ─────────────────────────────────────────────────────────

class _FloorCalibrator:
    """Histogram-based one-shot floor height estimator in camera_link frame (Z=up)."""

    SKIP_FRAMES  = 30
    CALIB_FRAMES = 30
    BIN_M        = 0.02

    def __init__(self) -> None:
        self._frame   = 0
        self._samples: list[float] = []
        self.floor_z:    float | None = None
        self.cam_height: float | None = None

    @property
    def ready(self) -> bool:
        return self.floor_z is not None

    def update(self, xyz_cam: np.ndarray) -> None:
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
        self._samples.append(float(edges[int(np.argmax(counts))] + self.BIN_M / 2))
        if len(self._samples) >= self.CALIB_FRAMES:
            self.floor_z    = float(np.median(self._samples))
            self.cam_height = -self.floor_z
            logger.info(f"StereoPointCloud: floor calibrated — camera {self.cam_height:.3f} m above floor")


# ── Madgwick AHRS ─────────────────────────────────────────────────────────────

class MadgwickFilter:
    """Gyro + accel → orientation R (camera_link → world, Z-up). beta=0.033."""

    def __init__(self, beta: float = 0.033) -> None:
        self._q      = np.array([1., 0., 0., 0.], dtype=np.float64)
        self._beta   = beta
        self._t_prev: float | None = None

    def update(self, gyro: np.ndarray, accel: np.ndarray, t: float) -> None:
        if self._t_prev is None:
            self._t_prev = t
            return
        dt = min(t - self._t_prev, 0.1)
        self._t_prev = t
        if dt <= 0:
            return
        q0, q1, q2, q3 = self._q
        gx, gy, gz = gyro.astype(np.float64)
        a_n = np.linalg.norm(accel)
        if a_n > 0.5:
            ax, ay, az = accel.astype(np.float64) / a_n
            f1 = 2*(q1*q3 - q0*q2) - ax
            f2 = 2*(q0*q1 + q2*q3) - ay
            f3 = 2*(0.5 - q1**2 - q2**2) - az
            J  = np.array([
                [-2*q2,  2*q3, -2*q0, 2*q1],
                [ 2*q1,  2*q0,  2*q3, 2*q2],
                [    0, -4*q1, -4*q2,    0],
            ])
            grad = J.T @ np.array([f1, f2, f3])
            gn   = np.linalg.norm(grad)
            if gn > 1e-9:
                grad /= gn
        else:
            grad = np.zeros(4)
        q_dot = 0.5 * np.array([
            -q1*gx - q2*gy - q3*gz,
             q0*gx + q2*gz - q3*gy,
             q0*gy - q1*gz + q3*gx,
             q0*gz + q1*gy - q2*gx,
        ]) - self._beta * grad
        self._q = self._q + q_dot * dt
        self._q /= np.linalg.norm(self._q) + 1e-12

    @property
    def R(self) -> np.ndarray:
        q0, q1, q2, q3 = self._q
        return np.array([
            [1-2*(q2**2+q3**2), 2*(q1*q2-q0*q3),   2*(q1*q3+q0*q2)  ],
            [2*(q1*q2+q0*q3),   1-2*(q1**2+q3**2), 2*(q2*q3-q0*q1)  ],
            [2*(q1*q3-q0*q2),   2*(q2*q3+q0*q1),   1-2*(q1**2+q2**2)],
        ], dtype=np.float32)


# ── Point-cloud odometry ──────────────────────────────────────────────────────

class PointCloudOdometry:
    """Rotation-decoupled ICP for translation t. Takes R from Madgwick, converges in 3-4 iters."""

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

    def update(self, xyz_cam: np.ndarray, R: np.ndarray) -> np.ndarray:
        pts_ga = (xyz_cam @ R.T).astype(np.float32)
        with self._lock:
            t_est = self._t.copy()
        if self._prev_world is None or len(pts_ga) < 50:
            t_new = t_est
        else:
            n_src = min(self.N_SRC, len(pts_ga))
            n_dst = min(self.N_DST, len(self._prev_world))
            src   = pts_ga[np.random.choice(len(pts_ga), n_src, replace=False)]
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
        n_st  = min(self.N_STORE, len(pts_ga))
        idx_s = np.random.choice(len(pts_ga), n_st, replace=False)
        self._prev_world = (pts_ga[idx_s] + t_new).astype(np.float32)
        with self._lock:
            self._t = t_new
        return t_new


# ── Module ────────────────────────────────────────────────────────────────────

class Config(ModuleConfig):
    min_depth: float          = 0.1
    max_depth: float          = 8.0
    gradient_threshold: float = 0.30
    vox_size: float           = 0.020
    global_vox_size: float    = 0.020
    floor_margin: float       = 0.04
    max_global_pts: int       = 500_000
    publish_every: int        = 1
    world_frame: str          = "world"
    madgwick_beta: float      = 0.033


class StereoPointCloud(Module):
    """D435i depth → frame_cloud + global_map. Pose from Madgwick IMU + ICP odometry."""

    config: Config

    depth_image:       In[Image]
    depth_camera_info: In[CameraInfo]

    frame_cloud: Out[PointCloud2]
    global_map:  Out[PointCloud2]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock                       = threading.Lock()
        self._latest_info: CameraInfo | None = None
        self._floor_calib                = _FloorCalibrator()
        self._madgwick: MadgwickFilter | None = None
        self._odom                       = PointCloudOdometry()
        self._imu_lock                   = threading.Lock()
        self._last_accel                 = np.array([0.0, 0.0, -9.81], dtype=np.float32)
        self._R_imu_to_link: np.ndarray  = np.eye(3, dtype=np.float32)
        self._motion_sensor              = None
        self._acc_pts: np.ndarray        = np.empty((0, 3), dtype=np.float32)
        self._map_ready                  = False
        self._world_floor_z: float       = 0.0
        self._frame                      = 0

    @rpc
    def start(self) -> None:
        super().start()
        self._madgwick = MadgwickFilter(beta=self.config.madgwick_beta)
        if not self._init_imu():
            logger.warning("StereoPointCloud: no IMU — R=identity, translation from ICP only")
        self.register_disposable(Disposable(self.depth_camera_info.subscribe(self._on_info)))
        self.register_disposable(Disposable(self.depth_image.subscribe(self._on_depth)))

    @rpc
    def stop(self) -> None:
        if self._motion_sensor is not None:
            try:
                self._motion_sensor.stop()
                self._motion_sensor.close()
            except Exception:
                pass
            self._motion_sensor = None
        super().stop()

    def _init_imu(self) -> bool:
        try:
            import pyrealsense2 as rs
        except ImportError:
            return False
        ctx     = rs.context()
        devices = ctx.query_devices()
        if not devices:
            return False
        device = devices[0]
        depth_profile = None
        for sensor in device.query_sensors():
            if sensor.is_motion_sensor():
                continue
            for p in sensor.get_stream_profiles():
                if p.stream_type() == rs.stream.depth:
                    depth_profile = p
                    break
            if depth_profile is not None:
                break
        for sensor in device.query_sensors():
            if not sensor.is_motion_sensor():
                continue
            all_profiles   = sensor.get_stream_profiles()
            gyro_profiles  = [p for p in all_profiles if p.stream_type() == rs.stream.gyro]
            accel_profiles = [p for p in all_profiles if p.stream_type() == rs.stream.accel]
            if not gyro_profiles or not accel_profiles:
                break
            gyro_p  = max(gyro_profiles,  key=lambda p: p.fps())
            accel_p = max(accel_profiles, key=lambda p: p.fps())
            if depth_profile is not None:
                ext                 = accel_p.get_extrinsics_to(depth_profile)
                R_imu_to_depth      = np.array(ext.rotation, dtype=np.float32).reshape(3, 3)
                self._R_imu_to_link = _R_OPT_TO_LINK @ R_imu_to_depth
            sensor.open([gyro_p, accel_p])
            sensor.start(self._on_motion)
            self._motion_sensor = sensor
            logger.info(f"StereoPointCloud: IMU — gyro@{gyro_p.fps()}Hz accel@{accel_p.fps()}Hz")
            return True
        return False

    def _on_motion(self, frame: Any) -> None:
        try:
            import pyrealsense2 as rs
            st = frame.get_profile().stream_type()
            if st == rs.stream.gyro:
                g        = frame.as_motion_frame().get_motion_data()
                gyro_lnk = self._R_imu_to_link @ np.array([g.x, g.y, g.z], dtype=np.float32)
                ts_s     = frame.get_timestamp() / 1000.0
                with self._imu_lock:
                    if self._madgwick is not None:
                        self._madgwick.update(gyro_lnk, self._last_accel, ts_s)
            elif st == rs.stream.accel:
                a = frame.as_motion_frame().get_motion_data()
                with self._imu_lock:
                    self._last_accel = self._R_imu_to_link @ np.array([a.x, a.y, a.z], dtype=np.float32)
        except Exception:
            pass

    def _on_info(self, info: CameraInfo) -> None:
        with self._lock:
            self._latest_info = info

    def _on_depth(self, img: Image) -> None:  # noqa: C901
        with self._lock:
            info = self._latest_info

        depth = img.data
        if hasattr(depth, "get"):
            depth = depth.get()
        if depth.ndim == 3:
            depth = depth[:, :, 0]
        depth = depth.astype(np.float32)
        valid_d = depth[depth > 0]
        if len(valid_d) and np.median(valid_d) > 100:
            depth /= 1000.0
        invalid = (depth <= 0) | (depth < self.config.min_depth) | (depth > self.config.max_depth)
        depth[invalid] = np.nan

        H, W = depth.shape
        if info is not None:
            K = info.get_K_matrix()
            fx, fy, cx, cy = float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2])
        else:
            fx = fy = float(max(H, W)) / 2.0
            cx, cy  = W / 2.0, H / 2.0

        stable = _gradient_mask(depth, self.config.gradient_threshold)
        valid  = np.isfinite(depth) & stable
        if not valid.any():
            return

        uu, vv  = np.meshgrid(np.arange(W, dtype=np.float32), np.arange(H, dtype=np.float32))
        dd      = depth[valid]
        xyz_opt = np.column_stack([
            (uu[valid] - cx) * dd / fx,
            (vv[valid] - cy) * dd / fy,
            dd,
        ]).astype(np.float32)

        with self._imu_lock:
            R = self._madgwick.R.copy() if self._madgwick is not None else np.eye(3, dtype=np.float32)
        t = self._odom.t

        xyz_cam   = xyz_opt @ _R_OPT_TO_LINK.T
        xyz_world = (xyz_cam @ R.T + t).astype(np.float32)
        cam_z     = float(t[2])

        self._floor_calib.update(xyz_cam)

        if self._floor_calib.ready:
            keep = xyz_cam[:, 2] > (self._floor_calib.floor_z + self.config.floor_margin)
        else:
            keep = (xyz_cam[:, 2] >= -1.4) & (xyz_cam[:, 2] <= 1.5)

        xyz_world_kept = xyz_world[keep]
        xyz_cam_kept   = xyz_cam[keep]
        if not len(xyz_world_kept):
            self._frame += 1
            return

        vk       = np.floor(xyz_world_kept / self.config.vox_size).astype(np.int32)
        _, first = np.unique(_pack(vk), return_index=True)
        xyz_vox  = xyz_world_kept[first]

        self.frame_cloud.publish(
            PointCloud2.from_numpy(xyz_vox, frame_id=self.config.world_frame, timestamp=img.ts)
        )

        if len(xyz_cam_kept) >= 50:
            self._odom.update(xyz_cam_kept, R)

        if not self._floor_calib.ready:
            self._frame += 1
            return

        if not self._map_ready:
            self._map_ready     = True
            self._world_floor_z = cam_z + self._floor_calib.floor_z
            logger.info(f"StereoPointCloud: global map started — floor at Z ≈ {self._world_floor_z:.3f} m")

        # Rotation-only frame: strip ICP translation so ±2 cm t-noise doesn't shift voxel keys
        xyz_ronly  = xyz_vox - t
        vk_r       = np.floor(xyz_ronly / self.config.vox_size).astype(np.int32)
        _, first_r = np.unique(_pack(vk_r), return_index=True)
        xyz_vox_r  = xyz_ronly[first_r]
        xyz_for_map = xyz_vox_r[xyz_vox_r[:, 2] > self._world_floor_z + self.config.floor_margin]

        pts_snap = None
        with self._lock:
            if len(self._acc_pts) > 0 and len(xyz_for_map) > 0:
                free_keys = _raycast_free_keys(xyz_for_map, self.config.global_vox_size)
                if len(free_keys):
                    keys_acc      = _pack(np.floor(self._acc_pts / self.config.global_vox_size).astype(np.int32))
                    self._acc_pts = self._acc_pts[~np.isin(keys_acc, free_keys)]

            if len(xyz_for_map):
                self._acc_pts = (
                    np.vstack([self._acc_pts, xyz_for_map]) if len(self._acc_pts) else xyz_for_map.copy()
                )
                _, ui         = np.unique(
                    _pack(np.floor(self._acc_pts / self.config.global_vox_size).astype(np.int32)),
                    return_index=True,
                )
                self._acc_pts = self._acc_pts[ui]
                if len(self._acc_pts) > self.config.max_global_pts:
                    keep_idx      = np.random.choice(len(self._acc_pts), self.config.max_global_pts, replace=False)
                    self._acc_pts = self._acc_pts[keep_idx]

            self._frame += 1
            if self._frame % self.config.publish_every == 0 and len(self._acc_pts):
                pts_snap = self._acc_pts.copy()

        if pts_snap is not None:
            self.global_map.publish(
                PointCloud2.from_numpy(pts_snap, frame_id=self.config.world_frame, timestamp=img.ts)
            )

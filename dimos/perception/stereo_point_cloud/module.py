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

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.stereo_point_cloud.utils import (
    _FloorCalibrator,
    _R_OPT_TO_LINK,
    _gradient_mask,
    _pack,
    _raycast_free_keys,
)
from dimos.perception.stereo_point_cloud.vio import MadgwickFilter, PointCloudOdometry
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_DEPTH_MM_THRESHOLD = 100


class Config(ModuleConfig):
    min_depth: float          = 0.1
    max_depth: float          = 8.0
    gradient_threshold: float = 0.15
    vox_size: float           = 0.020
    global_vox_size: float    = 0.020
    floor_margin: float       = 0.03
    # Cut for the global-map floor filter, above the CALIBRATED floor plane.
    # Must stay below the thinnest obstacle we want to keep: a 2 cm mat's top
    # surface sits ~2 cm above the floor, so 0.015 keeps it while still
    # removing the floor itself. (Was 0.04 — that deleted the mat once the
    # floor height was estimated correctly.)
    global_floor_margin: float = 0.015
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
        self._warned_no_intrinsics       = False

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
        is_mm = len(valid_d) > 0 and np.median(valid_d) > _DEPTH_MM_THRESHOLD
        if is_mm:
            depth /= 1000.0
        invalid = (depth <= 0) | (depth < self.config.min_depth) | (depth > self.config.max_depth)
        depth[invalid] = np.nan

        H, W = depth.shape
        if info is not None:
            K = info.get_K_matrix()
            fx, fy, cx, cy = float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2])
        else:
            if not self._warned_no_intrinsics:
                logger.warning("StereoPointCloud: no camera intrinsics — falling back to rough guess, check depth_camera_info is connected")
                self._warned_no_intrinsics = True
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

        # R=identity: camera_link Z is already up for a level D435i. Madgwick
        # converges toward a 180° X-flip (gravity is -Z in camera_link), which
        # rotates xyz_ronly away from the true camera frame and makes raycasting
        # fire from the wrong origin — causing ghost-ray carving artifacts.
        R = np.eye(3, dtype=np.float32)
        t = self._odom.t

        xyz_cam   = xyz_opt @ _R_OPT_TO_LINK.T
        xyz_world = (xyz_cam + t).astype(np.float32)

        # Calibrate on xyz_cam (camera_link, Z=up). For a level mount the floor
        # is always at z_cam == -cam_height regardless of horizontal distance,
        # so floor_z is always negative and cam_height always positive.
        # Using xyz_world - t (Madgwick-rotated) biases the estimate as the
        # filter drifts from identity over the first ~5 s.
        self._floor_calib.update(xyz_cam)

        # FIX (rerun black plane): publish with the floor at z = 0. The rest
        # of dimos assumes this datum — the rerun viewer's ground grid/floor
        # mesh sits at z ≈ 0 and the occupancy algos treat z as height above
        # ground. With the old camera-origin datum the whole scene (floor at
        # z ≈ -1, chairs/tables/mat in between) rendered UNDER the viewer's
        # ground plane. Shift is 0 until calibration completes (~4 s), then
        # the cloud snaps up by cam_height.
        z_shift = (
            float(self._floor_calib.cam_height)  # type: ignore[arg-type]
            if self._floor_calib.ready
            else 0.0
        )
        z_off = np.array([0.0, 0.0, z_shift], dtype=np.float32)

        # Per-frame floor filter disabled — calibration cuts obstacles at wrong height.
        # Floor is removed by the global map filter below; frame cloud shows all points.
        # if self._floor_calib.ready:
        #     keep = xyz_cam[:, 2] > (self._floor_calib.floor_z + self.config.floor_margin)
        # else:
        #     keep = np.ones(len(xyz_cam), dtype=bool)

        xyz_world_kept = xyz_world
        xyz_cam_kept   = xyz_cam
        if not len(xyz_world_kept):
            self._frame += 1
            return

        vk       = np.floor(xyz_world_kept / self.config.vox_size).astype(np.int32)
        _, first = np.unique(_pack(vk), return_index=True)
        xyz_vox  = xyz_world_kept[first]

        self.frame_cloud.publish(
            PointCloud2.from_numpy(
                xyz_vox + z_off, frame_id=self.config.world_frame, timestamp=img.ts
            )
        )

        if len(xyz_cam_kept) >= PointCloudOdometry.MIN_PTS:
            self._odom.update(xyz_cam_kept, R)

        if not self._floor_calib.ready:
            self._frame += 1
            return

        if not self._map_ready:
            self._map_ready = True
            # FIX: the filter below runs on xyz_ronly = xyz_vox - t, which is
            # CAMERA-CENTERED — so the threshold must be the camera-relative
            # floor height (floor_z), not cam_z + floor_z. Adding the ICP z
            # (a different frame) shifted the cut plane by whatever the ICP
            # translation happened to be at calibration time.
            self._world_floor_z = float(self._floor_calib.floor_z)  # type: ignore[arg-type]
            logger.info(
                f"StereoPointCloud: global map started — floor at "
                f"Z ≈ {self._world_floor_z:.3f} m below camera (published at Z ≈ 0)"
            )

        # Rotation-only frame (= xyz_cam with R=identity): strip ICP translation
        # so voxel keys are stable across small t-noise. Raycasting from (0,0,0)
        # is the true camera origin in this frame, so free-space carving is correct.
        xyz_ronly  = xyz_vox - t
        vk_r       = np.floor(xyz_ronly / self.config.vox_size).astype(np.int32)
        _, first_r = np.unique(_pack(vk_r), return_index=True)
        xyz_vox_r  = xyz_ronly[first_r]
        xyz_for_map = xyz_vox_r[xyz_vox_r[:, 2] > self._world_floor_z + self.config.global_floor_margin]

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
                PointCloud2.from_numpy(
                    pts_snap + z_off, frame_id=self.config.world_frame, timestamp=img.ts
                )
            )

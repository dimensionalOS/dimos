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

"""D435i depth -> per-frame voxel cloud + trajectory (Madgwick + ICP VIO).

No persistent global map — current-frame view and odometry only, for speed.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Any

import numpy as np
from reactivex.disposable import Disposable
from scipy.spatial.transform import Rotation

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.stereo_point_cloud.utils import (
    _FloorCalibrator,
    _R_OPT_TO_LINK,
    _gradient_mask,
    _pack,
)
from dimos.perception.stereo_point_cloud.vio import MadgwickFilter, PointCloudOdometry
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_DEPTH_MM_THRESHOLD = 100
_MILLIMETERS_PER_METER = 1000.0
_MILLISECONDS_PER_SECOND = 1000.0
_GRAVITY_MS2 = 9.81

_TRAJECTORY_MAX_POSES = 20_000
_TRAJECTORY_PUBLISH_EVERY = 5
_PERF_LOG_INTERVAL_FRAMES = 30


class Config(ModuleConfig):
    min_depth: float          = 0.1
    max_depth: float          = 8.0
    gradient_threshold: float = 0.15
    vox_size: float           = 0.05
    world_frame: str          = "world"
    madgwick_beta: float      = 0.033


class StereoPointCloud(Module):
    """D435i depth -> frame_cloud + trajectory. Pose from Madgwick IMU + frame-to-frame ICP."""

    config: Config

    depth_image:       In[Image]
    depth_camera_info: In[CameraInfo]

    frame_cloud: Out[PointCloud2]
    trajectory:  Out[Path]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock                       = threading.Lock()
        self._latest_camera_info: CameraInfo | None = None
        self._floor_calib                = _FloorCalibrator()
        self._madgwick: MadgwickFilter | None = None
        self._odom                       = PointCloudOdometry()
        self._imu_lock                   = threading.Lock()
        self._last_accel                 = np.array([0.0, 0.0, -_GRAVITY_MS2], dtype=np.float32)
        self._imu_to_camera_link: np.ndarray = np.eye(3, dtype=np.float32)
        self._motion_sensor              = None
        self._warned_no_intrinsics       = False
        self._trajectory_poses: deque[PoseStamped] = deque(maxlen=_TRAJECTORY_MAX_POSES)
        self._frame_count = 0

    @rpc
    def start(self) -> None:
        super().start()
        self._madgwick = MadgwickFilter(beta=self.config.madgwick_beta)
        if not self._init_imu():
            logger.warning("StereoPointCloud: no IMU — R=identity, translation from ICP only")
        self.register_disposable(Disposable(self.depth_camera_info.subscribe(self._on_camera_info)))
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
        context = rs.context()
        devices = context.query_devices()
        if not devices:
            return False
        device = devices[0]
        depth_profile = None
        for sensor in device.query_sensors():
            if sensor.is_motion_sensor():
                continue
            for profile in sensor.get_stream_profiles():
                if profile.stream_type() == rs.stream.depth:
                    depth_profile = profile
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
            gyro_profile  = max(gyro_profiles,  key=lambda p: p.fps())
            accel_profile = max(accel_profiles, key=lambda p: p.fps())
            if depth_profile is not None:
                extrinsics             = accel_profile.get_extrinsics_to(depth_profile)
                imu_to_depth            = np.array(extrinsics.rotation, dtype=np.float32).reshape(3, 3)
                self._imu_to_camera_link = _R_OPT_TO_LINK @ imu_to_depth
            sensor.open([gyro_profile, accel_profile])
            sensor.start(self._on_motion)
            self._motion_sensor = sensor
            logger.info(f"StereoPointCloud: IMU — gyro@{gyro_profile.fps()}Hz accel@{accel_profile.fps()}Hz")
            return True
        return False

    def _on_motion(self, frame: Any) -> None:
        try:
            import pyrealsense2 as rs
            stream_type = frame.get_profile().stream_type()
            if stream_type == rs.stream.gyro:
                gyro_sample = frame.as_motion_frame().get_motion_data()
                gyro_camera_link = self._imu_to_camera_link @ np.array(
                    [gyro_sample.x, gyro_sample.y, gyro_sample.z], dtype=np.float32
                )
                timestamp_s = frame.get_timestamp() / _MILLISECONDS_PER_SECOND
                with self._imu_lock:
                    if self._madgwick is not None:
                        self._madgwick.update(gyro_camera_link, self._last_accel, timestamp_s)
            elif stream_type == rs.stream.accel:
                accel_sample = frame.as_motion_frame().get_motion_data()
                with self._imu_lock:
                    self._last_accel = self._imu_to_camera_link @ np.array(
                        [accel_sample.x, accel_sample.y, accel_sample.z], dtype=np.float32
                    )
        except Exception:
            pass

    def _on_camera_info(self, info: CameraInfo) -> None:
        with self._lock:
            self._latest_camera_info = info

    def _on_depth(self, img: Image) -> None:
        t_start = time.perf_counter()
        with self._lock:
            camera_info = self._latest_camera_info

        depth = img.data
        if hasattr(depth, "get"):
            depth = depth.get()
        if depth.ndim == 3:
            depth = depth[:, :, 0]
        depth = depth.astype(np.float32)
        valid_depth = depth[depth > 0]
        is_millimeters = len(valid_depth) > 0 and np.median(valid_depth) > _DEPTH_MM_THRESHOLD
        if is_millimeters:
            depth /= _MILLIMETERS_PER_METER
        out_of_range = (depth <= 0) | (depth < self.config.min_depth) | (depth > self.config.max_depth)
        depth[out_of_range] = np.nan

        height, width = depth.shape
        if camera_info is not None:
            k_matrix = camera_info.get_K_matrix()
            focal_x, focal_y = float(k_matrix[0, 0]), float(k_matrix[1, 1])
            principal_x, principal_y = float(k_matrix[0, 2]), float(k_matrix[1, 2])
        else:
            if not self._warned_no_intrinsics:
                logger.warning("StereoPointCloud: no camera intrinsics — falling back to rough guess, check depth_camera_info is connected")
                self._warned_no_intrinsics = True
            focal_x = focal_y = float(max(height, width)) / 2.0
            principal_x, principal_y = width / 2.0, height / 2.0

        stable = _gradient_mask(depth, self.config.gradient_threshold)
        valid  = np.isfinite(depth) & stable
        if not valid.any():
            return
        t_gradient = time.perf_counter()

        u_grid, v_grid = np.meshgrid(np.arange(width, dtype=np.float32), np.arange(height, dtype=np.float32))
        depth_values = depth[valid]
        xyz_optical = np.column_stack([
            (u_grid[valid] - principal_x) * depth_values / focal_x,
            (v_grid[valid] - principal_y) * depth_values / focal_y,
            depth_values,
        ]).astype(np.float32)

        t_unproject = time.perf_counter()

        with self._imu_lock:
            R = self._madgwick.R_at(img.ts) if self._madgwick is not None else np.eye(3, dtype=np.float32)

        xyz_cam = xyz_optical @ _R_OPT_TO_LINK.T
        t_odom_start = time.perf_counter()
        t = (
            self._odom.update(xyz_cam, R)
            if len(xyz_cam) >= PointCloudOdometry.MIN_PTS
            else self._odom.t
        )
        t_odom_end = time.perf_counter()
        xyz_world = (xyz_cam @ R.T + t).astype(np.float32)

        self._floor_calib.update(xyz_cam)

        # Floor sits at z=0 to match the rerun ground grid; 0 until calibrated (~4s).
        z_shift = (
            float(self._floor_calib.cam_height)  # type: ignore[arg-type]
            if self._floor_calib.ready
            else 0.0
        )
        z_offset = np.array([0.0, 0.0, z_shift], dtype=np.float32)

        if not len(xyz_world):
            return

        voxel_indices = np.floor(xyz_world / self.config.vox_size).astype(np.int32)
        _, first_index_per_voxel = np.unique(_pack(voxel_indices), return_index=True)
        xyz_voxelized = xyz_world[first_index_per_voxel]

        self.frame_cloud.publish(
            PointCloud2.from_numpy(
                xyz_voxelized + z_offset, frame_id=self.config.world_frame, timestamp=img.ts
            )
        )
        t_publish = time.perf_counter()

        quat = Rotation.from_matrix(R).as_quat()
        with self._lock:
            self._trajectory_poses.append(
                PoseStamped(
                    ts=img.ts,
                    frame_id=self.config.world_frame,
                    position=[float(t[0]), float(t[1]), float(t[2])],
                    orientation=[float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])],
                )
            )
            self._frame_count += 1
            traj_snapshot = (
                list(self._trajectory_poses)
                if self._frame_count % _TRAJECTORY_PUBLISH_EVERY == 0
                else None
            )

        if traj_snapshot is not None:
            self.trajectory.publish(Path(ts=img.ts, frame_id=self.config.world_frame, poses=traj_snapshot))

        if self._frame_count % _PERF_LOG_INTERVAL_FRAMES == 0:
            t_end = time.perf_counter()
            logger.info(
                f"StereoPointCloud perf: lag={time.time() - img.ts:.1f}s "
                f"total={(t_end - t_start) * 1000:.0f}ms "
                f"gradient={(t_gradient - t_start) * 1000:.0f}ms "
                f"unproject={(t_unproject - t_gradient) * 1000:.0f}ms "
                f"odom={(t_odom_end - t_odom_start) * 1000:.0f}ms "
                f"publish={(t_publish - t_odom_end) * 1000:.0f}ms "
                f"traj={(t_end - t_publish) * 1000:.0f}ms"
            )

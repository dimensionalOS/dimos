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
from dimos.perception.stereo_point_cloud.imu import RealSenseImuFeed
from dimos.perception.stereo_point_cloud.utils import (
    _FloorCalibrator,
    _R_OPT_TO_LINK,
    _gradient_mask,
    _pack,
)
from dimos.perception.stereo_point_cloud.vio import PointCloudOdometry
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_DEPTH_MM_THRESHOLD = 100
_MILLIMETERS_PER_METER = 1000.0

_TRAJECTORY_MAX_POSES = 4_000
_TRAJECTORY_PUBLISH_EVERY = 15
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
        self._imu                        = RealSenseImuFeed(beta=self.config.madgwick_beta)
        self._odom                       = PointCloudOdometry()
        self._warned_no_intrinsics       = False
        self._trajectory_poses: list[PoseStamped] = []
        self._frame_count = 0

    @rpc
    def start(self) -> None:
        super().start()
        if not self._imu.start():
            logger.warning("StereoPointCloud: no IMU — R=identity, translation from ICP only")
        self.register_disposable(Disposable(self.depth_camera_info.subscribe(self._on_camera_info)))
        self.register_disposable(Disposable(self.depth_image.subscribe(self._on_depth)))

    @rpc
    def stop(self) -> None:
        self._imu.stop()
        super().stop()

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

        R = self._imu.R_at(img.ts)

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
            if len(self._trajectory_poses) >= _TRAJECTORY_MAX_POSES:
                self._trajectory_poses = self._trajectory_poses[::2]
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

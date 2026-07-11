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
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.stereo_point_cloud.imu import RealSenseImuFeed
from dimos.perception.stereo_point_cloud.trajectory import TrajectoryRecorder
from dimos.perception.stereo_point_cloud.utils import (
    R_OPT_TO_LINK,
    _FloorCalibrator,
    _format_perf_log,
    _gradient_mask,
    _intrinsics_from_camera_info,
    _preprocess_depth,
    _unproject,
    _voxel_dedup,
)
from dimos.perception.stereo_point_cloud.vio import PointCloudOdometry
from dimos.spec import perception
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

TRAJECTORY_MAX_POSES = 4_000
TRAJECTORY_PUBLISH_EVERY = 5
PERF_LOG_INTERVAL_FRAMES = 30
FRAME_CLOUD_PUBLISH_INTERVAL_S = 1.0 / 10.0
MAX_ACCEPTABLE_LAG_S = 0.5


class Config(ModuleConfig):
    min_depth: float          = 0.1
    max_depth: float          = 8.0
    gradient_threshold: float = 0.15
    vox_size: float           = 0.05
    world_frame: str          = "world"
    camera_frame: str         = "camera_link"
    madgwick_beta: float      = 0.033


class StereoPointCloud(Module, perception.Odometry):
    """D435i depth -> frame_cloud + trajectory + odometry. Pose from Madgwick IMU + frame-to-frame ICP."""

    config: Config

    depth_image:       In[Image]
    depth_camera_info: In[CameraInfo]

    frame_cloud: Out[PointCloud2]
    trajectory:  Out[Path]
    odometry:    Out[Odometry]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock                       = threading.Lock()
        self._latest_camera_info: CameraInfo | None = None
        self._floor_calib                = _FloorCalibrator()
        self._imu                        = RealSenseImuFeed(beta=self.config.madgwick_beta)
        self._odom                       = PointCloudOdometry()
        self._trajectory                 = TrajectoryRecorder(
            self.config.world_frame, TRAJECTORY_MAX_POSES, TRAJECTORY_PUBLISH_EVERY
        )
        self._warned_no_intrinsics       = False
        self._frame_count = 0
        self._last_frame_cloud_publish = 0.0
        self._last_point_count = 0

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

        # Drop stale frames instead of building a backlog.
        lag = time.time() - img.ts
        if lag > MAX_ACCEPTABLE_LAG_S:
            logger.warning(f"StereoPointCloud: dropping stale depth frame, lag={lag:.1f}s")
            return

        with self._lock:
            camera_info = self._latest_camera_info

        depth = _preprocess_depth(img.data, self.config.min_depth, self.config.max_depth)
        height, width = depth.shape
        if camera_info is None and not self._warned_no_intrinsics:
            logger.warning("StereoPointCloud: no camera intrinsics — falling back to rough guess, check depth_camera_info is connected")
            self._warned_no_intrinsics = True
        focal_x, focal_y, principal_x, principal_y = _intrinsics_from_camera_info(camera_info, height, width)

        # Reject noisy edge pixels.
        valid = np.isfinite(depth) & _gradient_mask(depth, self.config.gradient_threshold)
        if not valid.any():
            return
        t_gradient = time.perf_counter()

        # Pixel + depth -> 3D point (optical frame).
        xyz_optical = _unproject(depth, valid, principal_x, principal_y, focal_x, focal_y)
        t_unproject = time.perf_counter()

        # IMU orientation, interpolated to this frame's timestamp.
        R = self._imu.R_at(img.ts)

        # Rotate to camera-link frame, then ICP for translation.
        xyz_cam = xyz_optical @ R_OPT_TO_LINK.T
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
        z_shift = float(self._floor_calib.cam_height) if self._floor_calib.ready else 0.0  # type: ignore[arg-type]
        z_offset = np.array([0.0, 0.0, z_shift], dtype=np.float32)

        if not len(xyz_world):
            return

        # Throttled at the source, not just downstream.
        now_monotonic = time.monotonic()
        if now_monotonic - self._last_frame_cloud_publish >= FRAME_CLOUD_PUBLISH_INTERVAL_S:
            self._last_frame_cloud_publish = now_monotonic
            xyz_voxelized = _voxel_dedup(xyz_world, self.config.vox_size)
            self._last_point_count = len(xyz_voxelized)
            self.frame_cloud.publish(
                PointCloud2.from_numpy(
                    xyz_voxelized + z_offset, frame_id=self.config.world_frame, timestamp=img.ts
                )
            )
        t_publish = time.perf_counter()

        self._frame_count += 1
        quat = Rotation.from_matrix(R).as_quat()

        # Standard Odometry format, for any downstream consumer.
        self.odometry.publish(
            Odometry(
                ts=img.ts,
                frame_id=self.config.world_frame,
                child_frame_id=self.config.camera_frame,
                pose=Pose(
                    position=[float(t[0]), float(t[1]), float(t[2])],
                    orientation=[float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])],
                ),
            )
        )

        traj_snapshot = self._trajectory.record(img.ts, t, quat, self._frame_count)
        if traj_snapshot is not None:
            self.trajectory.publish(Path(ts=img.ts, frame_id=self.config.world_frame, poses=traj_snapshot))

        if self._frame_count % PERF_LOG_INTERVAL_FRAMES == 0:
            t_end = time.perf_counter()
            logger.info(
                _format_perf_log(
                    img.ts, t_start, t_gradient, t_unproject,
                    t_odom_start, t_odom_end, t_publish, t_end, self._last_point_count,
                )
            )

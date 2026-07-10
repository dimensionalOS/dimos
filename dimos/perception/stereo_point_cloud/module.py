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

# Trajectory keyframe cadence — record a pose only when the camera has moved
# this far/rotated this much since the last one. Plenty of resolution for a
# path trail without recording every single frame.
_KEYFRAME_DIST_M    = 0.08
_KEYFRAME_ANGLE_RAD = np.deg2rad(8.0)

# Cap on the internal ICP reference buffer (not published/visualized).
_ICP_REF_MAX_PTS = 50_000


class Config(ModuleConfig):
    min_depth: float          = 0.1
    max_depth: float          = 8.0
    gradient_threshold: float = 0.15
    vox_size: float           = 0.020
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
        self._latest_info: CameraInfo | None = None
        self._floor_calib                = _FloorCalibrator()
        self._madgwick: MadgwickFilter | None = None
        self._odom                       = PointCloudOdometry()
        self._imu_lock                   = threading.Lock()
        self._last_accel                 = np.array([0.0, 0.0, -9.81], dtype=np.float32)
        self._R_imu_to_link: np.ndarray  = np.eye(3, dtype=np.float32)
        self._motion_sensor              = None
        self._warned_no_intrinsics       = False
        self._last_kf_t: np.ndarray | None = None
        self._last_kf_R: np.ndarray | None = None
        # Trajectory: the robot's actual path — a lightweight list of past
        # poses (not point-cloud data), cheap to keep in full over a long run.
        self._trajectory_poses: list[PoseStamped] = []
        # ICP reference buffer — NOT published/visualized, purely so odometry has
        # more than just the single previous frame to match against. Without this,
        # one low-overlap or fast-motion frame permanently freezes the pose
        # estimate (nothing to re-localize against), which silently stops the
        # trajectory from extending even though the robot keeps moving.
        self._icp_ref_pts: np.ndarray = np.empty((0, 3), dtype=np.float32)

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

    def _on_depth(self, img: Image) -> None:
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

        with self._imu_lock:
            R = self._madgwick.R.copy() if self._madgwick is not None else np.eye(3, dtype=np.float32)
        t = self._odom.t

        xyz_cam   = xyz_opt @ _R_OPT_TO_LINK.T
        xyz_world = (xyz_cam @ R.T + t).astype(np.float32)

        # Calibrate on xyz_cam (camera_link, Z=up). For a level mount the floor
        # is always at z_cam == -cam_height regardless of horizontal distance,
        # so floor_z is always negative and cam_height always positive.
        self._floor_calib.update(xyz_cam)

        # FIX (rerun black plane): publish with the floor at z = 0. The rest of
        # dimos assumes this datum — the rerun viewer's ground grid/floor mesh
        # sits at z ≈ 0. Shift is 0 until calibration completes (~4s), then the
        # cloud snaps up by cam_height.
        z_shift = (
            float(self._floor_calib.cam_height)  # type: ignore[arg-type]
            if self._floor_calib.ready
            else 0.0
        )
        z_off = np.array([0.0, 0.0, z_shift], dtype=np.float32)

        if not len(xyz_world):
            return

        vk       = np.floor(xyz_world / self.config.vox_size).astype(np.int32)
        _, first = np.unique(_pack(vk), return_index=True)
        xyz_vox  = xyz_world[first]

        self.frame_cloud.publish(
            PointCloud2.from_numpy(
                xyz_vox + z_off, frame_id=self.config.world_frame, timestamp=img.ts
            )
        )

        # ICP against a rolling reference buffer (not the single previous frame) so
        # one low-overlap/fast-motion frame can't permanently freeze odometry.
        # Simple by design: just append + random-cap, no filtering — this buffer
        # is never published/visualized, purely a stability aid for pose tracking.
        if len(xyz_cam) >= PointCloudOdometry.MIN_PTS:
            with self._lock:
                ref_pts = self._icp_ref_pts.copy() if len(self._icp_ref_pts) > 0 else None
            self._odom.update(xyz_cam, R, map_ref=ref_pts)

            with self._lock:
                self._icp_ref_pts = (
                    np.vstack([self._icp_ref_pts, xyz_vox])
                    if len(self._icp_ref_pts)
                    else xyz_vox.copy()
                )
                if len(self._icp_ref_pts) > _ICP_REF_MAX_PTS:
                    keep_idx = np.random.choice(len(self._icp_ref_pts), _ICP_REF_MAX_PTS, replace=False)
                    self._icp_ref_pts = self._icp_ref_pts[keep_idx]

        # Trajectory: lightweight pose history, same keyframe cadence as before.
        with self._lock:
            take_kf = True
            if self._last_kf_t is not None:
                t_moved = float(np.linalg.norm(t - self._last_kf_t))
                cos_a   = float((np.trace(R @ self._last_kf_R.T) - 1.0) / 2.0)
                r_moved = float(np.arccos(np.clip(cos_a, -1.0, 1.0)))
                take_kf = (t_moved > _KEYFRAME_DIST_M) or (r_moved > _KEYFRAME_ANGLE_RAD)

            traj_snap = None
            if take_kf:
                self._last_kf_t = t.copy()
                self._last_kf_R = R.copy()
                quat = Rotation.from_matrix(R).as_quat()
                self._trajectory_poses.append(
                    PoseStamped(
                        ts=img.ts,
                        frame_id=self.config.world_frame,
                        position=[float(t[0]), float(t[1]), float(t[2])],
                        orientation=[float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])],
                    )
                )
                traj_snap = list(self._trajectory_poses)

        if traj_snap is not None:
            self.trajectory.publish(Path(ts=img.ts, frame_id=self.config.world_frame, poses=traj_snap))

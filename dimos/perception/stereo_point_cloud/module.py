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
    _FAT_SHIFTS,
    _FloorCalibrator,
    _R_OPT_TO_LINK,
    _gradient_mask,
    _isolation_filter,
    _pack,
)
from dimos.perception.stereo_point_cloud.vio import MadgwickFilter, PointCloudOdometry
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_DEPTH_MM_THRESHOLD = 100
# Extra floor margin added per metre of horizontal range from the camera.
# Compensates ~0.5° residual camera pitch: tan(0.5°) ≈ 0.009 m/m.
# Result: near the robot (mat detection) margin stays at global_floor_margin;
# at 4 m the effective cut rises by ~3.6 cm, rejecting the apparent floor elevation.
_FLOOR_TILT_COMP = 0.0

_KEYFRAME_DIST_M    = 0.08               # add to global map only when camera moves >8 cm
_KEYFRAME_ANGLE_RAD = np.deg2rad(8.0)   # or rotates >8°

# Depth-reprojection consistency check: an existing map point is dropped if the
# camera's real depth at that pixel disagrees with the point's own reprojected
# depth by more than this margin (in either direction — nearer means something
# now occludes it and it's stale, farther means we see clean past it to a real
# surface). 10cm — wider than the fat-voxel insertion guard's ~8cm tolerance,
# since the system's own documented pose-noise floor (Madgwick rotation lag
# ~7cm at 2m range for 2° error, uncorrected) needs headroom above it or
# ordinary jitter during rotation erases correct, already-mapped points.
_REPROJ_MARGIN_M = 0.10


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
    global_floor_margin: float = 0.04
    # Hard cap on the accumulated map — once exceeded, points are discarded
    # UNIFORMLY AT RANDOM across the whole map. Cut hard: RerunBridgeModule
    # processes every topic through ONE serial message queue (subscribe_all ->
    # single callback), so a big/frequent global_map payload doesn't just cost
    # memory — it blocks frame_cloud messages queued behind it too, even though
    # frame_cloud itself is cheap. Priority on this branch is fast current-frame
    # feedback + not OOMing; global_map is now explicitly secondary/low-res.
    max_global_pts: int       = 30_000
    # self._frame increments every valid frame (~15/s), not just keyframes.
    # Raised back up — global_map publishing needs to be rare enough that it
    # doesn't compete with frame_cloud in the bridge's single serial queue.
    # ~30 ≈ once every 2s at 15fps.
    publish_every: int        = 30
    world_frame: str          = "world"
    madgwick_beta: float      = 0.033


class StereoPointCloud(Module):
    """D435i depth → frame_cloud + global_map. Pose from Madgwick IMU + ICP odometry."""

    config: Config

    depth_image:       In[Image]
    depth_camera_info: In[CameraInfo]

    frame_cloud: Out[PointCloud2]
    global_map:  Out[PointCloud2]
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
        self._acc_pts: np.ndarray        = np.empty((0, 3), dtype=np.float32)
        self._map_ready                  = False
        self._world_floor_z: float       = 0.0
        self._frame                      = 0
        self._warned_no_intrinsics       = False
        self._last_kf_t: np.ndarray | None = None
        self._last_kf_R: np.ndarray | None = None
        # Trajectory: the robot's actual path, distinct from the obstacle/wall
        # map — a lightweight list of past poses (not point-cloud data), cheap
        # to keep in full even over a long run.
        self._trajectory_poses: list[PoseStamped] = []

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
        # Same quality bar used to build new points, reused below so edge/gradient
        # noise can't masquerade as evidence when checking old map points against
        # this frame's depth.
        depth_checked = np.where(stable, depth, np.nan)

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
            with self._lock:
                map_ref = self._acc_pts.copy() if len(self._acc_pts) > 0 else None
            self._odom.update(xyz_cam_kept, R, map_ref=map_ref)

        if not self._floor_calib.ready:
            self._frame += 1
            return

        if not self._map_ready:
            self._map_ready = True
            # Floor threshold in world frame: floor_z is camera-relative (negative).
            # ICP vertical drift is small so adding t[2] keeps the cut plane accurate.
            self._world_floor_z = float(self._floor_calib.floor_z) + float(t[2])  # type: ignore[arg-type]
            logger.info(
                f"StereoPointCloud: global map started — floor at "
                f"Z ≈ {self._world_floor_z:.3f} m (world frame, published at Z ≈ 0)"
            )

        # Keyframe gate: only insert/clean when camera has moved/rotated enough.
        # Keeps global-map density equal to one clean frame per area explored.
        with self._lock:
            take_kf = True
            if self._last_kf_t is not None:
                t_moved = float(np.linalg.norm(t - self._last_kf_t))
                cos_a   = float((np.trace(R @ self._last_kf_R.T) - 1.0) / 2.0)
                r_moved = float(np.arccos(np.clip(cos_a, -1.0, 1.0)))
                take_kf = (t_moved > _KEYFRAME_DIST_M) or (r_moved > _KEYFRAME_ANGLE_RAD)

        # Depth-reprojection consistency check: project existing map points into
        # THIS frame's camera view and compare against the real depth the camera
        # reports there right now. Disagreement (either direction, past the
        # margin) means the old point is contradicted by fresh evidence — either
        # a ghost from pose drift, or an obstacle that's genuinely moved — so it
        # gets dropped. No evidence (point falls outside the frame, behind the
        # camera, or lands on an invalid/occluded pixel) leaves it untouched.
        # Gated to keyframes (not every frame) — this branch prioritizes fast
        # current-frame throughput; projecting the whole accumulated map every
        # single frame was competing with frame_cloud for CPU and slowing down
        # how often frame_cloud itself could actually be published.
        if take_kf:
            with self._lock:
                acc_pts = self._acc_pts
                if len(acc_pts) > 0:
                    acc_cam  = (acc_pts - t) @ R
                    acc_opt  = acc_cam @ _R_OPT_TO_LINK
                    d_pred   = acc_opt[:, 2]
                    in_front = d_pred > 0.05
                    safe_d   = np.where(in_front, d_pred, 1.0)
                    u = fx * acc_opt[:, 0] / safe_d + cx
                    v = fy * acc_opt[:, 1] / safe_d + cy
                    in_bounds = in_front & (u >= 0) & (u < W) & (v >= 0) & (v < H)
                    idx = np.where(in_bounds)[0]
                    if len(idx):
                        ui = u[idx].astype(np.int32)
                        vi = v[idx].astype(np.int32)
                        d_obs   = depth_checked[vi, ui]
                        d_exp   = d_pred[idx]
                        valid_obs    = np.isfinite(d_obs)
                        contradicted = valid_obs & (np.abs(d_obs - d_exp) > _REPROJ_MARGIN_M)
                        stale_idx = idx[contradicted]
                        if len(stale_idx):
                            keep = np.ones(len(acc_pts), dtype=bool)
                            keep[stale_idx] = False
                            self._acc_pts = acc_pts[keep]

        # World-frame accumulation: xyz_vox = xyz_cam @ R.T + t is a fixed frame.
        # Same physical point always has the same xyz_world key regardless of camera position.
        # The old xyz_ronly = xyz_vox - t was a MOVING frame — as t changed, the same
        # physical point mapped to a different key, causing chair-duplicates on every move.
        xy_dist     = np.linalg.norm(xyz_vox[:, :2] - t[:2], axis=1)
        floor_thresh = self._world_floor_z + self.config.global_floor_margin + _FLOOR_TILT_COMP * xy_dist
        xyz_for_map = xyz_vox[xyz_vox[:, 2] > floor_thresh]

        pts_snap = None
        traj_snap = None
        with self._lock:
            if take_kf and len(xyz_for_map):
                self._last_kf_t = t.copy()
                self._last_kf_R = R.copy()
                # Same keyframe cadence as the map (~8cm/8°) — plenty of resolution
                # for a path trail, no need for a separate gate.
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
                # Isolation filter runs on the full dense per-frame set (not the sparse
                # leftover frontier after covered-voxel subtraction below) — flying
                # pixels are isolated within a dense frame, but legitimate new frontier
                # points are naturally sparse relative to *each other* and would almost
                # all get killed if filtered after subtraction.
                # min_neighbors=2 (scipy's neighbor count includes the point itself) let a
                # tight cluster of exactly 3 noise points through — each counts itself + the
                # other 2 = 3, which is > 2. 6 still rejects small floating clusters (need 7+
                # points within 6cm to survive) while being less aggressive than 8.
                if len(xyz_for_map) > 3:
                    xyz_for_map = xyz_for_map[_isolation_filter(xyz_for_map, radius=0.06, min_neighbors=6)]

                vox = self.config.global_vox_size
                new_vk   = np.floor(xyz_for_map / vox).astype(np.int32)
                new_keys = _pack(new_vk)
                if len(self._acc_pts) > 0:
                    keys_acc_s = np.sort(_pack(np.floor(self._acc_pts / vox).astype(np.int32)))
                    shifted    = (new_keys[:, None] + _FAT_SHIFTS[None, :]).ravel()
                    i_s        = np.searchsorted(keys_acc_s, shifted)
                    i_s        = np.clip(i_s, 0, len(keys_acc_s) - 1)
                    covered    = (keys_acc_s[i_s] == shifted).reshape(len(new_keys), len(_FAT_SHIFTS)).any(axis=1)
                    xyz_to_add = xyz_for_map[~covered]
                else:
                    xyz_to_add = xyz_for_map
                if len(xyz_to_add):
                    self._acc_pts = (
                        np.vstack([self._acc_pts, xyz_to_add]) if len(self._acc_pts) else xyz_to_add.copy()
                    )
                    _, ui         = np.unique(
                        _pack(np.floor(self._acc_pts / vox).astype(np.int32)),
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

        if traj_snap is not None:
            self.trajectory.publish(Path(ts=img.ts, frame_id=self.config.world_frame, poses=traj_snap))

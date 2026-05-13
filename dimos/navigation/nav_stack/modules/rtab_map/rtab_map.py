# Copyright 2026 Dimensional Inc.
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

"""RTAB-Map SLAM module — alternative to PGO that consumes FastLIO2 odometry.

Mirrors the PGO contract (same input/output streams) plus two extra outputs
that surface OctoMap state for downstream consumers:

- ``octomap``: occupied voxel centroids in the map frame
- ``projected_2d_grid``: footprint of the OctoMap projected onto the floor

The module is a Python :class:`Module` (not a :class:`NativeModule`) because
the actual RTAB-Map runtime is wrapped behind a :class:`RtabRunner` seam, so
validation can exercise the wiring with a pure-Python lite runner without
requiring a C++ binary in the test loop.

User-set defaults from the spec are honored: 3D enabled, OctoMap enabled,
raycasting/point-clearing enabled.
"""

from __future__ import annotations

from typing import Any

import numpy as np
from reactivex.disposable import Disposable
from scipy.spatial.transform import Rotation

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseWithCovariance import PoseWithCovariance
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_stack.frames import FRAME_BODY, FRAME_MAP, FRAME_ODOM
from dimos.navigation.nav_stack.modules.rtab_map._rtab_runner import (
    LiteRtabRunner,
    LiteRtabRunnerConfig,
    RtabRunner,
    RunnerStepResult,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class RtabMapConfig(ModuleConfig):
    """Config for the RtabMap module.

    The OctoMap-relevant defaults match the user spec exactly: 3D on,
    OctoMap on, raycasting / point clearing on. Naming follows RTAB-Map's
    own ``Grid/*`` parameter namespace where possible.
    """

    # --- OctoMap / Grid defaults (the six required by the locked spec) ---
    grid_3d: bool = True
    grid_ray_tracing: bool = True
    grid_from_depth: bool = False  # LiDAR mode; RGBD is a follow-up
    grid_cell_size: float = 0.1
    grid_max_ground_angle: float = 45.0
    grid_ground_is_obstacle: bool = False
    grid_flat_obstacle_detected: bool = True

    # --- Loop closure tunables (analogous to PGOConfig) ---
    key_pose_delta_trans: float = 0.5
    key_pose_delta_deg: float = 10.0
    loop_search_radius: float = 2.0
    loop_score_thresh: float = 0.6
    submap_resolution: float = 0.1
    min_loop_detect_duration: float = 5.0

    # --- Frame names ---
    world_frame: str = FRAME_MAP
    local_frame: str = FRAME_ODOM
    body_frame: str = FRAME_BODY

    # --- Output publishing ---
    publish_global_map: bool = True
    publish_octomap: bool = True
    publish_projected_2d_grid: bool = True

    # --- Input handling ---
    unregister_input: bool = True  # input scans are in world frame; convert to body


class RtabMap(Module):
    """RTAB-Map-backed loop closure + occupancy mapping module.

    Plays the same role in the nav stack as :class:`PGO`: takes FastLIO2's raw
    odometry and the registered scan stream, detects loop closures, publishes
    a corrected odometry stream, a ``map -> odom`` TF correction, and the
    accumulated global cloud. Adds two OctoMap outputs.

    The actual algorithm lives in a :class:`RtabRunner` implementation that
    can be swapped at construction time (defaults to :class:`LiteRtabRunner`
    for tests and bring-up; will move to a system-rtabmap subprocess bridge
    in a follow-up).
    """

    config: RtabMapConfig

    registered_scan: In[PointCloud2]
    odometry: In[Odometry]

    corrected_odometry: Out[Odometry]
    global_map: Out[PointCloud2]
    rtab_tf: Out[Odometry]
    octomap: Out[PointCloud2]
    projected_2d_grid: Out[PointCloud2]

    def __init__(
        self,
        runner: RtabRunner | None = None,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        self._runner: RtabRunner = runner or LiteRtabRunner(self._lite_config_from(self.config))
        self._latest_odom: Odometry | None = None

    @classmethod
    def _lite_config_from(cls, config: RtabMapConfig) -> LiteRtabRunnerConfig:
        return LiteRtabRunnerConfig(
            cell_size=config.grid_cell_size,
            ray_tracing=config.grid_ray_tracing,
            max_ground_angle_deg=config.grid_max_ground_angle,
            ground_is_obstacle=config.grid_ground_is_obstacle,
            keyframe_delta_trans=config.key_pose_delta_trans,
            keyframe_delta_rad=float(np.deg2rad(config.key_pose_delta_deg)),
            revisit_radius_m=config.loop_search_radius,
            loop_fitness_threshold=config.loop_score_thresh,
            min_loop_detect_duration_s=config.min_loop_detect_duration,
        )

    # ------------- lifecycle -------------

    @rpc
    def start(self) -> None:
        super().start()
        # Subscribe to inputs. Scans are the primary trigger; odom is cached
        # so each scan step can look up the latest body pose.
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odometry)))
        self.register_disposable(
            Disposable(self.registered_scan.subscribe(self._on_registered_scan))
        )
        # Subscribe to our own rtab_tf to fan corrections out to the Python TF
        # bridge — same pattern PGO uses for pgo_tf.
        self.register_disposable(
            Disposable(self.rtab_tf.transport.subscribe(self._on_tf_correction, self.rtab_tf))
        )
        # Seed identity TF so downstream consumers can resolve `map -> body`
        # before the first loop closure shifts the frame.
        self._publish_tf(
            translation=(0.0, 0.0, 0.0),
            rotation=(0.0, 0.0, 0.0, 1.0),
            ts=0.0,
        )
        logger.info(
            "RtabMap started (grid_3d=%s ray_tracing=%s cell=%.3f)",
            self.config.grid_3d,
            self.config.grid_ray_tracing,
            self.config.grid_cell_size,
        )

    @rpc
    def stop(self) -> None:
        super().stop()

    # ------------- handlers -------------

    def _on_odometry(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def _on_registered_scan(self, scan: PointCloud2) -> None:
        odom = self._latest_odom
        if odom is None:
            # Nothing to correct against yet — wait for first odometry.
            return

        scan_points_map, _ = scan.as_numpy()
        if scan_points_map is None or len(scan_points_map) == 0:
            return

        odom_pose = _se3_from_odometry(odom)
        scan_points_body = (
            _world_to_body(scan_points_map, odom_pose)
            if self.config.unregister_input
            else scan_points_map
        )

        result = self._runner.process(
            scan_points_body=scan_points_body.astype(np.float64),
            odom_pose=odom_pose,
            timestamp=float(scan.ts),
        )

        self._publish_results(result, odom, scan_ts=float(scan.ts))

    def _on_tf_correction(self, msg: Odometry) -> None:
        self._publish_tf(
            translation=(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
            rotation=(
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ),
            ts=msg.ts,
        )

    # ------------- publishing -------------

    def _publish_results(
        self,
        result: RunnerStepResult,
        source_odom: Odometry,
        scan_ts: float,
    ) -> None:
        corrected = _odometry_from_se3(
            result.corrected_pose,
            frame_id=self.config.world_frame,
            child_frame_id=self.config.body_frame,
            ts=source_odom.ts,
        )
        self.corrected_odometry.publish(corrected)

        tf_msg = _odometry_from_se3(
            result.tf_correction,
            frame_id=self.config.world_frame,
            child_frame_id=self.config.local_frame,
            ts=source_odom.ts,
        )
        self.rtab_tf.publish(tf_msg)

        if self.config.publish_octomap:
            self.octomap.publish(
                PointCloud2.from_numpy(
                    result.octomap_voxels,
                    frame_id=self.config.world_frame,
                    timestamp=scan_ts,
                )
            )

        if self.config.publish_projected_2d_grid:
            self.projected_2d_grid.publish(
                PointCloud2.from_numpy(
                    result.projected_2d_voxels,
                    frame_id=self.config.world_frame,
                    timestamp=scan_ts,
                )
            )

        if self.config.publish_global_map:
            self.global_map.publish(
                PointCloud2.from_numpy(
                    result.global_map_points,
                    frame_id=self.config.world_frame,
                    timestamp=scan_ts,
                )
            )

    def _publish_tf(
        self,
        translation: tuple[float, float, float],
        rotation: tuple[float, float, float, float],
        ts: float,
    ) -> None:
        self.tf.publish(
            Transform(
                frame_id=self.config.world_frame,
                child_frame_id=self.config.local_frame,
                translation=Vector3(*translation),
                rotation=Quaternion(*rotation),
                ts=ts,
            )
        )


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


def _se3_from_odometry(msg: Odometry) -> np.ndarray:
    """Build a 4x4 SE(3) matrix from an :class:`Odometry` message's pose."""
    quat = msg.pose.orientation
    rot = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w]).as_matrix()
    pose = np.eye(4)
    pose[:3, :3] = rot
    pose[:3, 3] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    return pose


def _odometry_from_se3(
    pose: np.ndarray,
    *,
    frame_id: str,
    child_frame_id: str,
    ts: float,
) -> Odometry:
    """Pack a 4x4 SE(3) matrix back into an :class:`Odometry` message."""
    quat = Rotation.from_matrix(pose[:3, :3]).as_quat()  # [x, y, z, w]
    pose_msg = Pose(
        float(pose[0, 3]),
        float(pose[1, 3]),
        float(pose[2, 3]),
        float(quat[0]),
        float(quat[1]),
        float(quat[2]),
        float(quat[3]),
    )
    return Odometry(
        ts=ts,
        frame_id=frame_id,
        child_frame_id=child_frame_id,
        pose=PoseWithCovariance(pose_msg),
    )


def _world_to_body(points_world: np.ndarray, body_to_world: np.ndarray) -> np.ndarray:
    """Inverse transform world-frame points back into the body frame."""
    inv = np.eye(4)
    rot_t = body_to_world[:3, :3].T
    inv[:3, :3] = rot_t
    inv[:3, 3] = -rot_t @ body_to_world[:3, 3]
    rotated = points_world @ inv[:3, :3].T
    return rotated + inv[:3, 3]

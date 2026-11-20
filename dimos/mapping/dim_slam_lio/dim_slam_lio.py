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

"""dimSLAM's lidar-inertial odometry with no cuVSLAM: any number of lidars and IMUs (and other
odometry sources) fused in one filter, Point-LIO's estimator. Builds on boards with no CUDA
(the M20's RK3588). The camera path stays in ``dimos.mapping.dim_slam``."""

from __future__ import annotations

import math
from typing import Literal

from pydantic import BaseModel, Field, model_validator

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage

# [x, y, z, roll, pitch, yaw] (or [vx, vy, vz, wx, wy, wz]): <0 the message's covariance, 0 drops the axis, >0 fixed.
Variances = tuple[float, float, float, float, float, float]


class ImuConfig(BaseModel):
    """One IMU, identified by the frame_id its samples carry; tf places it relative to the output frame. Defaults are Point-LIO's."""

    frame_id: str
    # rad^2/s^2 and m^2/s^4: variance of one gyro / accelerometer reading.
    gyro_variance: float = 0.01
    accel_variance: float = 0.01
    # rad/s and g: readings within clip_fraction of these are clipped and say nothing; 0 trusts every reading.
    gyro_clip: float = 35.0
    accel_clip_g: float = 3.0
    clip_fraction: float = 0.99
    # Random-walk variances of the gyro and accelerometer biases, per s^2.
    gyro_bias_random_walk: float = 0.0001
    accel_bias_random_walk: float = 0.0001
    # Datasheet figures (rad/s/sqrt(Hz), rad/s^2/sqrt(Hz), m/s^2/sqrt(Hz), m/s^3/sqrt(Hz)); each one set (>0) replaces the
    # matching variance above, converted at the IMU's rate measured over its first init_samples: density^2 * rate.
    gyro_noise_density: float = 0.0
    gyro_random_walk: float = 0.0
    accel_noise_density: float = 0.0
    accel_random_walk: float = 0.0
    # Samples the first IMU averages to level the filter (and any IMU its rate is measured over).
    init_samples: int = Field(default=100, ge=1)
    # rad/s: levelling restarts while the first IMU turns faster than this; 0 never restarts.
    init_gyro_limit: float = Field(default=0.0, ge=0)
    # s added to this IMU's stamps to put them on the lidars' clock (e.g. a host-stamped IMU next to PTP lidars).
    time_offset_s: float = 0.0


class LidarConfig(BaseModel):
    """One lidar, identified by the frame_id its scans carry; tf places it relative to the output frame. Defaults are Point-LIO's."""

    frame_id: str
    # m^2: variance of one point's distance to its plane.
    point_variance: float = 0.01
    # Its own scan periods another lidar's scan waits for it before going on without it.
    max_wait_periods: float = Field(default=2.0, ge=0)
    # Thinning before fusion: points nearer than min_range m dropped, every keep_every-th usable point kept, then one
    # per scan_cube m cube; the filter's cost is linear in what is left.
    min_range: float = Field(default=0.5, ge=0)
    keep_every: int = Field(default=3, ge=1)
    scan_cube: float = Field(default=0.2, ge=0)
    # m: a point within this of the previous one on every axis is a repeat and dropped.
    duplicate_tolerance: float = Field(default=1e-7, ge=0)
    # Map neighbors a plane is fitted through, and how close (m) they must all lie to it.
    plane_points: int = Field(default=5, ge=3)
    plane_tolerance: float = Field(default=0.1, gt=0)
    # A point is used only when its range exceeds this times the square of its distance to its plane.
    range_per_distance_sq: float = 81.0
    # m: map points farther than this are not neighbors.
    neighbor_range: float = Field(default=5.0, gt=0)


class SourceConfig(BaseModel):
    """An external odometry source, identified by the transform its estimates carry."""

    parent_frame_id: str
    child_frame_id: str
    # pose: a non-drifting world of its own; twist: body velocity only; pose_delta: a drifting pose, fused by its motion.
    kind: Literal["pose", "twist", "pose_delta"] = "pose_delta"
    pose_variances: Variances = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    twist_variances: Variances = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    # Zero-velocity update: a twist at or under these speeds is fused as zero with still_variance; 0 disables.
    still_variance: float = 0.0
    still_speed: float = 0.0
    still_angular_speed: float = 0.0


class ProcessNoise(BaseModel):
    """Random-walk variances, per s^2, of velocity, angular velocity and specific force (Point-LIO's defaults)."""

    velocity: float = 20.0
    angular_velocity: float = 1000.0
    specific_force: float = 500.0


class InitialStds(BaseModel):
    """1-sigma uncertainty the filter starts with per state block (Point-LIO's: variance 0.01, gravity 1e-4, biases 1e-3)."""

    position: float = math.sqrt(0.01)
    velocity: float = math.sqrt(0.01)
    rotation: float = math.sqrt(0.01)
    bias: float = math.sqrt(0.001)
    mount_rotation: float = math.sqrt(0.01)
    mount_translation: float = math.sqrt(0.01)
    angular_velocity: float = math.sqrt(0.01)
    specific_force: float = math.sqrt(0.01)
    gravity: float = math.sqrt(0.0001)


class MapConfig(BaseModel):
    """The map scans are matched against (Point-LIO's defaults)."""

    # m: a scan point is added only where the map has none in its cube of this side.
    spacing: float = Field(default=0.5, gt=0)
    # m: side of the map's hash cubes; neighbors are searched in a point's cube and the six sharing its faces.
    cube: float = Field(default=2.0, gt=0)
    # Cubes kept; past this the least recently touched goes.
    capacity: int = Field(default=1_000_000, ge=1)
    # Points the first scans must give before the map is used.
    seed_points: int = Field(default=10, ge=1)
    # A scan grows the map only when it kept more than this many points.
    grow_min_points: int = Field(default=4, ge=0)


class DeskewConfig(BaseModel):
    """Each scan's points moved by the filter's own pose at each point's instant to the scan's end, on deskewed_cloud."""

    enabled: bool = False
    # output: the body (output_frame_id) at the scan's end; odom: odom_frame_id.
    frame: Literal["output", "odom"] = "output"
    # m: one point per cube of this side; 0 keeps every point.
    voxel: float = Field(default=0.0, ge=0)


class DimSlamLioConfig(NativeModuleConfig):
    source_dir: str | None = "dimos/mapping/dim_slam_lio/rust"
    executable: str = "result/bin/dim_slam_lio"
    build_command: str | None = "nix build -L path:."
    stdin_config: bool = True

    # Frame the odometry is stamped in (its parent), and the body the filter estimates (its child).
    odom_frame_id: str = "odom"
    output_frame_id: str = "base_link"
    # Off when something downstream owns odom_frame_id -> output_frame_id.
    publish_tf: bool = True
    # The first drives the filter between scans and levels it; samples from frames not listed are dropped.
    imus: list[ImuConfig] = Field(default_factory=list)
    # Scans from frames not listed are dropped.
    lidars: list[LidarConfig] = Field(default_factory=list)
    odom_sources: list[SourceConfig] = Field(default_factory=list)
    process_noise: ProcessNoise = Field(default_factory=ProcessNoise)
    # Every this many scans, log the handlers' time and the inputs' age on arrival (lidar clock vs host); 0 never.
    stats_every_scans: int = Field(default=100, ge=0)
    # CPUs the process runs on (Linux); empty leaves it to the scheduler.
    cpus: list[int] = Field(default_factory=list)
    # m/s^2 per g: IMU samples arrive in m/s^2 and are divided by this.
    standard_gravity: float = 9.80665
    # m/s^2: the world's gravity once the first IMU levels the filter, and what one g of an accelerometer reads.
    initial_gravity_estimate: float = 9.81
    initial_stds: InitialStds = Field(default_factory=InitialStds)
    map: MapConfig = Field(default_factory=MapConfig)
    # Gate on source measurements: <0 the 99% chi-squared bound of their rows, 0 no gate, >0 this many times their row
    # count in squared Mahalanobis distance.
    outlier_rejection_allowed_variance: float = -1.0
    # m: a pose farther than this from the origin is not published (the filter ran away); 0 disables.
    max_position_m: float = Field(default=0.0, ge=0)
    # Zero-twist [vx vy vz wx wy wz] fused after each source update: >0 pins that axis to zero with this variance.
    per_dimension_error_variance: Variances = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    # Publish every Nth pose the filter makes (1 = all): lidars split into short slices make a pose per slice.
    publish_every: int = Field(default=1, ge=1)
    # s between repeats of one throttled warning.
    warn_throttle_s: float = Field(default=10.0, ge=0)
    deskew: DeskewConfig = Field(default_factory=DeskewConfig)

    @model_validator(mode="after")
    def _frames_are_distinct(self) -> DimSlamLioConfig:
        for kind, frames in (
            ("imus", [imu.frame_id for imu in self.imus]),
            ("lidars", [lidar.frame_id for lidar in self.lidars]),
        ):
            if len(set(frames)) != len(frames):
                raise ValueError(f"{kind} repeat a frame_id: {frames}")
        return self


class DimSlamLio(NativeModule):
    """Every lidar publishes onto ``lidar`` and every IMU onto ``imu``, told apart by frame_id
    (clouds as Livox drivers write them: float32 x/y/z plus a uint32 ``offset_time``, ns after
    the stamp). Mounts come from tf against ``output_frame_id``. Publishes ``odometry`` and
    odom_frame_id -> output_frame_id on tf, and with ``deskew.enabled`` each scan deskewed on
    ``deskewed_cloud``."""

    config: DimSlamLioConfig

    lidar: In[PointCloud2]
    imu: In[Imu]
    odom_sources: In[Odometry]

    odometry: Out[Odometry]
    # With deskew.enabled: one cloud per input scan, stamped at its end.
    deskewed_cloud: Out[PointCloud2]
    tf: IO[TFMessage]

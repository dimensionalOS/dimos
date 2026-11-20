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

"""Native Rust dimSLAM module: one filter (Point-LIO's estimator, grown to take every sensor) fed by
cuVSLAM's camera tracking, any number of lidars and IMUs, and any odometry sources, in one process.
With no ``camera_mode`` it runs without cameras: lidar-inertial odometry for boards with no GPU (the
M20's RK3588 runs the CPU-only build)."""

from __future__ import annotations

import math
from typing import Any, Literal

from pydantic import BaseModel, Field, field_serializer, model_validator

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger
from dimos.utils.nvidia_env import driver_env, sdk_variant

logger = setup_logger()


class CameraConfig(BaseModel):
    """Settings for one camera, identified by the frame_id its images carry. A depth stream
    is a camera of its own here, and need not be a rig camera."""

    frame_id: str = ""
    # Asserted, not performed. cuVSLAM takes one flag for the whole rig, so the rig's
    # cameras have to agree.
    rectified: bool = True
    # Raw depth units per metre; 16-bit millimetre depth is 1000. Scaling one camera's depth
    # by another's factor yields a plausible-looking wrong map.
    depth_units_per_meter: float = 1000.0
    # Range gate on the published depth_cloud, metres. Stereo depth error grows as range
    # squared, so the far gate decides whether the cloud is worth mapping with; 0 leaves it
    # open.
    depth_cloud_min_range: float = 0.0
    depth_cloud_max_range: float = 0.0
    # One point per k x k depth block (median of in-gate depths). <= 1 emits every pixel.
    depth_cloud_decimation: int = 0
    # Multisensor only: this rig camera also receives depth images, already aligned to it.
    # Any depth-providing camera anchors the rig's metric scale.
    provides_depth: bool = False


class Covariance(BaseModel):
    """Per-axis variances. For a pose x/y/z are metres and roll/pitch/yaw radians; for a
    twist they are the linear (m/s) and angular (rad/s) rates about the same axes."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    def any_set(self) -> bool:
        return any(self.model_dump().values())


class ImuConfig(BaseModel):
    """One IMU, identified by the frame_id its samples carry; tf places it relative to the output frame. Each keeps its
    own noise figures and its own estimated biases, so a good gyro next to a cheap one keeps its say. The noise defaults
    are Point-LIO's; the init defaults are dimSLAM v0.4's."""

    frame_id: str = ""
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
    # Samples the first IMU averages to level the filter (and any IMU its rate is measured over). At 200 Hz this is one
    # second of standing still at startup.
    init_samples: int = Field(default=200, ge=1)
    # rad/s: levelling restarts while the first IMU turns faster than this, so it belongs above this gyro's own bias and
    # below any real rotation; 0 never restarts.
    init_gyro_limit: float = Field(default=0.05, ge=0)
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
    """One external odometry source and how much it is trusted. A variance below zero takes the message covariance, zero
    drops that dimension, above zero is a fixed variance. A drifting source's covariance describes its accumulated drift
    rather than the delta being fused, so a fixed value is usually the right answer."""

    # The transform this source's estimates carry: each message's header.frame_id and child_frame_id. Both halves are
    # needed because two sources can share a parent.
    parent_frame_id: str = ""
    child_frame_id: str = ""
    # pose: a non-drifting world of its own; twist: body velocity only; pose_delta: a drifting pose, fused by its motion.
    # None: a source whose parent is odom_frame_id is a pose, anything else a pose_delta (dimSLAM v0.4's rule).
    kind: Literal["pose", "twist", "pose_delta"] | None = None
    pose_variances: Covariance = Field(default_factory=Covariance)
    # Body frame.
    twist_variances: Covariance = Field(default_factory=Covariance)
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
    """1-sigma uncertainty the filter starts with per state block. position, velocity, rotation and bias keep dimSLAM
    v0.4's defaults; the blocks Point-LIO's state adds take Point-LIO's (variance 0.01, gravity 1e-4). bias applies to
    every IMU's gyro and accel biases."""

    position: float = 0.01
    velocity: float = 0.1
    rotation: float = 0.05
    bias: float = 0.05
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
    # Neighbors in Point-LIO's order, bit for bit; else a pruned exact five-nearest search (the same neighbors).
    point_lio_neighbor_order: bool = False


class DeskewConfig(BaseModel):
    """Each scan's points moved by the filter's own pose at each point's instant to the scan's end, on deskewed_cloud."""

    enabled: bool = False
    # output: the body (output_frame_id) at the scan's end; odom: odom_frame_id.
    frame: Literal["output", "odom"] = "output"
    # m: one point per cube of this side; 0 keeps every point.
    voxel: float = Field(default=0.0, ge=0)


class DimSlamConfig(NativeModuleConfig):
    source_dir: str | None = "dimos/mapping/dim_slam/rust"
    executable: str = "result/bin/dim_slam"
    # A literal so bin/native-modules can read it: CI builds `default`, which needs
    # no GPU. The variant this machine needs is chosen in model_post_init.
    build_command: str | None = "nix build -L path:."
    stdin_config: bool = True
    extra_env: dict[str, str] = Field(default_factory=driver_env)

    # multisensor (experimental in cuVSLAM): any mix of RGB and RGB-D cameras plus an
    # optional IMU; the cameras marked provides_depth anchor the rig's metric scale. None
    # tracks no camera (camera messages are dropped): lidar, IMU and sources only.
    camera_mode: Literal["mono", "stereo", "rgbd", "multisensor"] | None = None
    # In cuVSLAM's index order: the rig cameras first (two for stereo, the whole list for
    # multisensor, one otherwise), then any settings-only streams such as an rgbd depth
    # camera. Empty auto-discovers the rig from camera_info; an unlisted camera takes the
    # defaults. Multisensor requires this list: any rig size is legal there, so discovery
    # cannot know when it is done.
    cameras: list[CameraConfig] = Field(default_factory=list)
    # Off runs the deterministic CPU path, which needs a libcuvslam built
    # -DENFORCE_GPU=OFF. A build carrying only the other backend is used with a warning.
    use_gpu: bool = True

    # Frame the cuVSLAM rig is built in. Empty means output_frame_id.
    rig_frame_id: str = ""
    # Carried for whatever consumes the loop-closed pose; nothing here publishes map -> odom.
    map_frame_id: str = "map"

    # Stamp spread one frame set may span, milliseconds; the default is cuVSLAM's 1 ms contract
    # (v0.4 spelled that 0, which now means 0 ms). A software-triggered rig needs this widened:
    # Spot's images land within ~15 ms of each other and its depth trails its camera by up to
    # ~90 ms, so every set is dropped at 1 ms.
    max_skew_ms: float = Field(default=1.0, gt=0)
    # Translation std (m) above which the frame's motion is dropped and the path rebased;
    # 0 disables.
    covariance_gate_translation_std: float = 1.0
    # Frame-to-frame m/s and rad/s, catching teleports the covariance gate misses; 0 disables.
    speed_gate_max_linear: float = 5.0
    speed_gate_max_angular: float = 12.0

    odom_frame_id: str = "odom"
    output_frame_id: str = "base_link"
    # Off when something downstream owns odom -> output_frame_id.
    publish_tf: bool = True

    # The filter publishes as its inputs move it (a pose per scan with a lidar, per IMU sample
    # without one, per measurement with neither), not on a timer; this keeps every Nth.
    publish_every: int = Field(default=1, ge=1)
    # Gate on source measurements, in squared Mahalanobis distance per measured row: >0 this
    # many times the row count, 0 no gate, <0 the 99% chi-squared bound of the rows.
    outlier_rejection_allowed_variance: float = 25.0
    # m: a pose farther than this from the origin is not published (the filter ran away); 0 disables.
    max_position_m: float = Field(default=10000.0, ge=0)

    # The first drives the filter between scans and levels it; samples from frames not listed
    # are dropped. Empty: the filter is driven by its sources (and lidars) alone.
    imus: list[ImuConfig] = Field(default_factory=list)
    # Scans from frames not listed are dropped.
    lidars: list[LidarConfig] = Field(default_factory=list)
    process_noise: ProcessNoise = Field(default_factory=ProcessNoise)
    # m/s^2 per g: IMU samples arrive in m/s^2 and are divided by this.
    standard_gravity: float = 9.80665
    # m/s^2: the world's gravity once the first IMU levels the filter, and what one g of an
    # accelerometer reads. A config because gravity varies with latitude by ~0.05 m/s^2.
    initial_gravity_estimate: float = 9.8
    initial_stds: InitialStds = Field(default_factory=InitialStds)
    map: MapConfig = Field(default_factory=MapConfig)
    deskew: DeskewConfig = Field(default_factory=DeskewConfig)
    # s between repeats of one throttled warning.
    warn_throttle_s: float = Field(default=10.0, ge=0)
    # Every this many scans, log the handlers' time and the inputs' age on arrival (sensor
    # clock vs host); 0 never.
    stats_every_scans: int = Field(default=100, ge=0)
    # CPUs the process runs on (Linux); empty leaves it to the scheduler.
    cpus: list[int] = Field(default_factory=list)

    # Trust in the tracker's pose, per SourceConfig semantics; the module registers the
    # tracker as a fusion source itself, so it never appears in `odom_sources`. The tracker
    # is fused by pose deltas only.
    visual_odom_pose_variances: Covariance = Field(
        default_factory=lambda: Covariance(x=0.01, y=0.01, z=0.01, roll=0.05, pitch=0.05, yaw=0.05)
    )
    # One entry per external source (wheel odometry, ...), matched against each message's
    # header.frame_id and child_frame_id.
    odom_sources: list[SourceConfig] = Field(default_factory=list)
    # A virtual zero-twist measurement applied after every source message (needs an IMU), for
    # the directions the platform cannot move in. Above zero pulls that dimension toward zero
    # with this variance; zero leaves it free.
    per_dimension_error_variance: Covariance = Field(default_factory=Covariance)

    @model_validator(mode="after")
    def _multisensor_lists_its_rig(self) -> DimSlamConfig:
        if self.camera_mode == "multisensor" and not self.cameras:
            raise ValueError(
                "camera_mode='multisensor' requires cameras; auto-discovery cannot know "
                "how many cameras the rig has"
            )
        return self

    @model_validator(mode="after")
    def _sensors_are_named_once(self) -> DimSlamConfig:
        for kind, frames in (
            ("imus", [imu.frame_id for imu in self.imus]),
            ("lidars", [lidar.frame_id for lidar in self.lidars]),
        ):
            for index, frame in enumerate(frames):
                if not frame.strip():
                    raise ValueError(f"{kind}[{index}] needs a frame_id")
            if len(set(frames)) != len(frames):
                raise ValueError(f"{kind} repeat a frame_id: {frames}")
        return self

    @model_validator(mode="after")
    def _sources_are_fusable(self) -> DimSlamConfig:
        # Zero drops a dimension, so an all-zero source is fused in no dimension at all.
        if self.camera_mode is not None and not self.visual_odom_pose_variances.any_set():
            raise ValueError(
                "every visual_odom_pose_variances entry is zero, "
                "which fuses the tracker in no dimension at all"
            )
        for index, source in enumerate(self.odom_sources):
            if not source.parent_frame_id.strip() or not source.child_frame_id.strip():
                raise ValueError(
                    f"odom_sources[{index}] needs both parent_frame_id and child_frame_id"
                )
            if not source.pose_variances.any_set() and not source.twist_variances.any_set():
                raise ValueError(
                    f"odom_sources[{index}] ({source.parent_frame_id!r} -> "
                    f"{source.child_frame_id!r}) has every pose and twist variance at zero, "
                    "which fuses nothing; use a positive variance or drop the source"
                )
        return self

    @model_validator(mode="after")
    def _sources_have_a_kind(self) -> DimSlamConfig:
        for source in self.odom_sources:
            if source.kind is None:
                source.kind = (
                    "pose" if source.parent_frame_id == self.odom_frame_id else "pose_delta"
                )
        return self

    @field_serializer("camera_mode")
    def _no_camera_as_empty(self, camera_mode: str | None) -> str:
        # the module reads an empty camera_mode as no cameras
        return camera_mode or ""

    def model_post_init(self, context: Any, /) -> None:
        super().model_post_init(context)
        if self.build_command == "nix build -L path:.":
            self.build_command = f"nix build -L path:.#{sdk_variant()}"


class DimSlam(NativeModule):
    """Every lidar publishes onto ``lidar`` and every IMU onto ``imu``, told apart by frame_id
    (clouds as Livox drivers write them: float32 x/y/z plus a uint32 ``offset_time``, ns after
    the stamp). With ``deskew.enabled`` each scan comes back deskewed on ``deskewed_cloud``.

    Every camera publishes onto the same ``image`` and ``camera_info`` streams and is
    told apart by ``frame_id``; the ``cameras`` list fixes which frames are on the rig and
    in what order. Extrinsics come from tf against ``rig_frame_id``. ``depth_image`` feeds
    ``depth_cloud`` in every mode, and is additionally tracked against in ``rgbd``,
    reprojected onto the rig camera through ``depth_camera_info`` and tf when the depth
    sensor differs.

    The tracker's pose stream never touches the wire: it enters the filter directly
    as a drifting source. Any number of external sources
    (wheel odometry, ...) publish onto ``odom_sources`` and are told apart by
    ``header.frame_id`` and ``child_frame_id``. A late message is carried forward to the
    filter's instant by the filter's own motion since its stamp.
    """

    config: DimSlamConfig

    image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]
    depth_camera_info: In[CameraInfo]
    imu: In[Imu]
    lidar: In[PointCloud2]
    odom_sources: In[Odometry]

    odometry: Out[Odometry]
    depth_cloud: Out[PointCloud2]
    # With deskew.enabled: one cloud per input scan, stamped at its end.
    deskewed_cloud: Out[PointCloud2]
    tf: IO[TFMessage]

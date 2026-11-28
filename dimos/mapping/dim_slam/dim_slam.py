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

"""DimSlam: odometry from any mix of lidars, IMUs, cameras and other odometry sources, fused in one filter.

Wiring. Every sensor of a kind shares one input and is told apart by the frame_id its messages carry; tf must
place each sensor's frame relative to ``output_frame_id`` (the body being tracked).

- Lidar-inertial (no camera, runs on CPU-only boards): connect ``lidar`` (PointCloud2 with float32 x/y/z and a
  uint32 ``offset_time`` in ns after the stamp) and ``imu``; list each lidar in ``lidars`` and each IMU in ``imus``;
  leave ``camera_mode`` unset.
- Camera (visual odometry): connect ``image`` and ``camera_info`` (and ``depth_image``/``depth_camera_info`` for
  depth); set ``camera_mode`` and list the rig in ``cameras`` (or leave it empty to discover it from camera_info).
  Add ``imu`` and ``imus`` for visual-inertial.
- Camera + lidar: both of the above at once.
- Other odometry (wheels, another SLAM): connect ``odom_sources`` and list each in ``odom_sources``.

Outputs: ``odometry`` (and ``odom_frame_id`` -> ``output_frame_id`` on tf unless ``publish_tf`` is off),
``depth_cloud`` from depth images, and with ``deskew.enabled`` each lidar scan motion-corrected on
``deskewed_cloud``. ``publish_every`` thins the odometry (a pose per lidar scan, IMU sample or measurement).

Example, two lidars and one IMU::

    DimSlam.blueprint(
        odom_frame_id="map",
        output_frame_id="base_link",
        lidars=[LidarConfig(frame_id="front_lidar"), LidarConfig(frame_id="rear_lidar")],
        imus=[ImuConfig(frame_id="rear_lidar_imu", gyro_variance=1e-6, accel_variance=1e-4)],
        publish_every=4,
    )
"""

from __future__ import annotations

import math
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, field_serializer, model_validator

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


class _Strict(BaseModel):
    """A field this model does not have is an error, never silently dropped."""

    model_config = ConfigDict(extra="forbid")


# fields removed in dimSLAM v1.0, and what replaced each
REMOVED_FIELDS = {
    "publish_rate": "the filter publishes as its inputs move it, not on a timer; use publish_every to keep every Nth pose",
    "replay_buffer_seconds": "a late message is carried forward to the filter's instant by its own motion, so nothing is replayed",
}


class CameraConfig(_Strict):
    """One camera, identified by the frame_id its images carry; a depth stream is a camera of its own."""

    frame_id: str = ""
    # the rig's images are already rectified (one flag for the whole rig, so the rig's cameras must agree)
    rectified: bool = True
    # raw depth units per metre (16-bit millimetre depth is 1000)
    depth_units_per_meter: float = 1000.0
    # m: depth_cloud keeps points between these ranges; 0 leaves that side open
    depth_cloud_min_range: float = 0.0
    depth_cloud_max_range: float = 0.0
    # one depth_cloud point per k x k block of depth pixels (their median); <= 1 keeps every pixel
    depth_cloud_decimation: int = 0
    # multisensor only: this rig camera also receives depth images aligned to it, which set the rig's metric scale
    provides_depth: bool = False


class Covariance(_Strict):
    """Per-axis variances: m^2 and rad^2 for a pose, (m/s)^2 and (rad/s)^2 for a twist; <0 the message's, 0 unused."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    def any_set(self) -> bool:
        return any(self.model_dump().values())


class ImuConfig(_Strict):
    """One IMU, identified by the frame_id its samples carry; each keeps its own noise figures and estimated biases."""

    frame_id: str = ""
    # rad^2/s^2 and m^2/s^4: variance of one gyro / accelerometer reading
    gyro_variance: float = 0.01
    accel_variance: float = 0.01
    # rad/s and g: readings within clip_fraction of these are treated as clipped and ignored; 0 never clips
    gyro_clip: float = 35.0
    accel_clip_g: float = 3.0
    clip_fraction: float = 0.99
    # per s^2: random-walk variances of the gyro and accelerometer biases
    gyro_bias_random_walk: float = 0.0001
    accel_bias_random_walk: float = 0.0001
    # datasheet noise densities and bias random walks (per sqrt(Hz)); each one >0 replaces the matching value above
    gyro_noise_density: float = 0.0
    gyro_random_walk: float = 0.0
    accel_noise_density: float = 0.0
    accel_random_walk: float = 0.0
    # samples averaged at rest to level the filter (and to measure an IMU's rate for its densities)
    init_samples: int = Field(default=200, ge=1)
    # rad/s: levelling restarts while the first IMU turns faster than this; 0 never restarts
    init_gyro_limit: float = Field(default=0.05, ge=0)
    # s added to this IMU's stamps to put them on the other sensors' clock
    time_offset_s: float = 0.0


class LidarConfig(_Strict):
    """One lidar, identified by the frame_id its scans carry."""

    frame_id: str
    # m^2: variance of one point's distance to its plane
    point_variance: float = 0.01
    # in this lidar's own scan periods: how long another lidar's scan waits for it
    max_wait_periods: float = Field(default=2.0, ge=0)
    # m: points nearer than this are dropped
    min_range: float = Field(default=0.5, ge=0)
    # every keep_every-th usable point is kept
    keep_every: int = Field(default=3, ge=1)
    # m: then one point per cube of this side; 0 keeps every point
    scan_cube: float = Field(default=0.2, ge=0)
    # m: a point within this of the previous one on every axis is a repeat and dropped
    duplicate_tolerance: float = Field(default=1e-7, ge=0)
    # map neighbors a point's plane is fitted through
    plane_points: int = Field(default=5, ge=3)
    # m: the plane is used only when every neighbor lies this close to it
    plane_tolerance: float = Field(default=0.1, gt=0)
    # a point is used only when its range exceeds this times the square of its distance to its plane
    range_per_distance_sq: float = 81.0
    # m: map points farther than this are not neighbors
    neighbor_range: float = Field(default=5.0, gt=0)


class SourceConfig(_Strict):
    """One external odometry source, identified by the transform its messages carry (header.frame_id, child_frame_id)."""

    parent_frame_id: str = ""
    child_frame_id: str = ""
    # pose: a world that does not drift; twist: body velocity only; pose_delta: a drifting pose, fused by its motion
    # (unset: pose when parent_frame_id is odom_frame_id, else pose_delta)
    kind: Literal["pose", "twist", "pose_delta"] | None = None
    pose_variances: Covariance = Field(default_factory=Covariance)
    # body frame
    twist_variances: Covariance = Field(default_factory=Covariance)
    # a twist at or under these speeds (m/s, rad/s) is fused as zero with still_variance; 0 disables
    still_variance: float = 0.0
    still_speed: float = 0.0
    still_angular_speed: float = 0.0


class ProcessNoise(_Strict):
    """How fast the body's motion may change between measurements: random-walk variances per s^2."""

    velocity: float = 20.0
    angular_velocity: float = 1000.0
    specific_force: float = 500.0


class InitialStds(_Strict):
    """1-sigma uncertainty the filter starts with, per state (m, m/s, rad, rad/s, m/s^2); bias applies to every IMU."""

    position: float = 0.01
    velocity: float = 0.1
    rotation: float = 0.05
    bias: float = 0.05
    mount_rotation: float = math.sqrt(0.01)
    mount_translation: float = math.sqrt(0.01)
    angular_velocity: float = math.sqrt(0.01)
    specific_force: float = math.sqrt(0.01)
    gravity: float = math.sqrt(0.0001)


class MapConfig(_Strict):
    """The local map lidar scans are matched against."""

    # m: a scan point is added only where the map has none in its cube of this side
    spacing: float = Field(default=0.5, gt=0)
    # m: side of the map's hash cubes; neighbors are searched in a point's cube and the six sharing its faces
    cube: float = Field(default=2.0, gt=0)
    # cubes kept; past this the least recently touched is dropped
    capacity: int = Field(default=1_000_000, ge=1)
    # points the first scans must give before the map is used
    seed_points: int = Field(default=10, ge=1)
    # a scan grows the map only when it kept more than this many points
    grow_min_points: int = Field(default=4, ge=0)
    # neighbors from the full search in its original order; off: a pruned search finding the same neighbors faster
    unpruned_neighbor_order: bool = False


class DeskewConfig(_Strict):
    """Each lidar scan's points moved to where they lie at the scan's end, by the filter's pose at each point's time."""

    enabled: bool = False
    # output: the body (output_frame_id) at the scan's end; odom: odom_frame_id
    frame: Literal["output", "odom"] = "output"
    # m: one point per cube of this side; 0 keeps every point
    voxel: float = Field(default=0.0, ge=0)


class DimSlamConfig(NativeModuleConfig):
    source_dir: str | None = "dimos/mapping/dim_slam/rust"
    executable: str = "result/bin/dim_slam"
    # a literal so bin/native-modules can read it; model_post_init picks this machine's variant
    build_command: str | None = "nix build -L path:."
    stdin_config: bool = True
    extra_env: dict[str, str] = Field(default_factory=driver_env)

    # unset: no camera tracking (camera messages are dropped); multisensor is experimental
    camera_mode: Literal["mono", "stereo", "rgbd", "multisensor"] | None = None
    # rig cameras first (two for stereo, all for multisensor, one otherwise), then settings-only streams such as depth
    # (empty: discovered from camera_info; multisensor needs the list)
    cameras: list[CameraConfig] = Field(default_factory=list)
    # track on the GPU; off uses the CPU backend
    use_gpu: bool = True
    # frame the camera rig is built in; empty means output_frame_id
    rig_frame_id: str = ""
    # parent frame name for consumers of a loop-closed pose (nothing here publishes map -> odom)
    map_frame_id: str = "map"
    # ms: how far apart one frame set's stamps may be
    max_skew_ms: float = Field(default=1.0, gt=0)
    # m: a camera frame whose translation std exceeds this is dropped and the path rebased; 0 disables
    covariance_gate_translation_std: float = 1.0
    # m/s and rad/s: a camera step faster than these is dropped as a teleport; 0 disables
    speed_gate_max_linear: float = 5.0
    speed_gate_max_angular: float = 12.0

    # frame the odometry is expressed in (its parent)
    odom_frame_id: str = "odom"
    # the body being tracked (its child); every sensor is placed relative to it by tf
    output_frame_id: str = "base_link"
    # publish odom_frame_id -> output_frame_id on tf; off when something else owns that transform
    publish_tf: bool = True
    # publish every Nth pose the filter makes (it makes one per lidar scan, IMU sample or measurement)
    publish_every: int = Field(default=1, ge=1)
    # gate on source measurements, in squared Mahalanobis distance: >0 this times the row count, 0 none, <0 99% bound
    outlier_rejection_allowed_variance: float = 25.0
    # m: a pose farther than this from the origin is not published; 0 disables
    max_position_m: float = Field(default=10000.0, ge=0)

    # the first drives the filter between scans and levels it; samples from unlisted frames are dropped
    imus: list[ImuConfig] = Field(default_factory=list)
    # scans from unlisted frames are dropped
    lidars: list[LidarConfig] = Field(default_factory=list)
    process_noise: ProcessNoise = Field(default_factory=ProcessNoise)
    # m/s^2 per g: IMU samples arrive in m/s^2 and are divided by this
    standard_gravity: float = 9.80665
    # m/s^2: local gravity, set when the first IMU levels the filter
    initial_gravity_estimate: float = 9.8
    initial_stds: InitialStds = Field(default_factory=InitialStds)
    map: MapConfig = Field(default_factory=MapConfig)
    deskew: DeskewConfig = Field(default_factory=DeskewConfig)
    # s between repeats of one throttled warning
    warn_throttle_s: float = Field(default=10.0, ge=0)
    # log handler time and input age every this many lidar scans; 0 never
    stats_every_scans: int = Field(default=100, ge=0)
    # CPUs the process may run on (Linux); empty leaves it to the scheduler
    cpus: list[int] = Field(default_factory=list)

    # trust in the camera tracker's motion, fused as a pose_delta source (variance rules as Covariance)
    visual_odom_pose_variances: Covariance = Field(
        default_factory=lambda: Covariance(x=0.01, y=0.01, z=0.01, roll=0.05, pitch=0.05, yaw=0.05)
    )
    # external odometry (wheels, another SLAM), matched by each message's header.frame_id and child_frame_id
    odom_sources: list[SourceConfig] = Field(default_factory=list)
    # zero-twist [vx vy vz wx wy wz] fused after each source message (needs an IMU): >0 pins that axis, 0 frees it
    per_dimension_error_variance: Covariance = Field(default_factory=Covariance)

    @model_validator(mode="before")
    @classmethod
    def _no_removed_fields(cls, data: Any) -> Any:
        if isinstance(data, dict):
            removed = [name for name in REMOVED_FIELDS if name in data]
            if removed:
                raise ValueError(
                    "; ".join(
                        f"{name} was removed in dimSLAM v1.0: {REMOVED_FIELDS[name]}"
                        for name in removed
                    )
                )
        return data

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
    """Odometry from lidars, IMUs, cameras and odometry sources; see the module docstring for wiring and config."""

    config: DimSlamConfig

    # every camera's images and infos, told apart by frame_id
    image: In[Image]
    camera_info: In[CameraInfo]
    # depth images (and their camera's info when depth must be reprojected onto a rig camera)
    depth_image: In[Image]
    depth_camera_info: In[CameraInfo]
    # every IMU's samples, told apart by frame_id
    imu: In[Imu]
    # every lidar's scans, told apart by frame_id
    lidar: In[PointCloud2]
    # external odometry, told apart by header.frame_id and child_frame_id
    odom_sources: In[Odometry]

    odometry: Out[Odometry]
    # range-gated depth points, in the depth frame
    depth_cloud: Out[PointCloud2]
    # with deskew.enabled: one cloud per input scan, stamped at its end
    deskewed_cloud: Out[PointCloud2]
    tf: IO[TFMessage]

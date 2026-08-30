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

"""Native Rust dimSLAM module: cuVSLAM visual odometry fused with an IMU and any
number of odometry sources by an error-state Kalman filter, in one process."""

from __future__ import annotations

from pathlib import Path
from typing import Literal

from pydantic import BaseModel, Field, model_validator

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.nvidia_env import driver_env, sdk_variant
from dimos.core.stream import IO, In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

MODULE_DIR = Path(__file__).resolve().parent


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


# Datasheet values, in the continuous-time units the filter wants: rad/s/sqrt(Hz),
# rad/s^2/sqrt(Hz), m/s^2/sqrt(Hz), m/s^3/sqrt(Hz). Named so the config check can insist on them.
IMU_NOISE_FIGURES = (
    "gyro_noise_density",
    "gyro_random_walk",
    "accel_noise_density",
    "accel_random_walk",
)


class ImuConfig(BaseModel):
    """One physical IMU: its noise figures and how long it has to hold still to init."""

    # The frame_id the IMU's samples carry; samples from any other frame are dropped.
    frame_id: str = ""
    gyro_noise_density: float = 0.0
    gyro_random_walk: float = 0.0
    accel_noise_density: float = 0.0
    accel_random_walk: float = 0.0

    # Averaged while stationary to level the filter and take the gyro bias; leaving that
    # bias in cost 19.8 m of final error against 1.6 m on a 517 s Alfred drive. At 200 Hz
    # this is one second of standing still at startup.
    init_samples: int = 200
    # rad/s. Above this the robot is called moving and bias calibration restarts, so it
    # belongs above this gyro's own bias and below any real rotation. A noisy gyro that
    # reads above it at rest never finishes init.
    init_gyro_limit: float = 0.05


class SourceConfig(BaseModel):
    """One external odometry source and how much it is trusted. A variance below zero takes
    the message covariance, zero drops that dimension, above zero is a fixed variance. A
    drifting source's covariance describes its accumulated drift rather than the delta
    being fused, so a fixed value is usually the right answer."""

    # The transform this source's estimates carry: each message's header.frame_id and
    # child_frame_id. Both halves are needed because two sources can share a parent. A
    # source whose parent is odom_frame_id is fused absolutely; anything else is fused as
    # filter-anchored deltas, since its own pose has drifted.
    parent_frame_id: str = ""
    child_frame_id: str = ""
    # [x y z roll pitch yaw]
    pose_variances: list[float] = Field(
        default_factory=lambda: [0.0] * 6, min_length=6, max_length=6
    )
    # [vx vy vz wx wy wz], body frame
    twist_variances: list[float] = Field(
        default_factory=lambda: [0.0] * 6, min_length=6, max_length=6
    )


class DimSlamConfig(NativeModuleConfig):
    cwd: str | None = str(MODULE_DIR / "rust")
    executable: str = "result/bin/dim_slam"
    build_command: str | None = Field(
        default_factory=lambda: f"nix build -L 'path:.#{sdk_variant()}'"
    )
    stdin_config: bool = True
    extra_env: dict[str, str] = Field(default_factory=driver_env)

    # no default: this choice changes what inputs are required
    camera_mode: Literal["mono", "stereo", "rgbd"]
    # In cuVSLAM's index order: the rig cameras first (two for stereo, one otherwise), then
    # any settings-only streams such as an rgbd depth camera. Empty auto-discovers the rig
    # from camera_info; an unlisted camera takes the defaults.
    cameras: list[CameraConfig] = Field(default_factory=list)
    # Off runs the deterministic CPU path, which needs a libcuvslam built
    # -DENFORCE_GPU=OFF. A build carrying only the other backend is used with a warning.
    use_gpu: bool = True

    # Frame the cuVSLAM rig is built in. Empty means output_frame_id.
    rig_frame_id: str = ""
    # Carried for whatever consumes the loop-closed pose; nothing here publishes map -> odom.
    map_frame_id: str = "map"

    # Stamp spread one frame set may span, milliseconds; 0 keeps cuVSLAM's 1 ms contract. A
    # software-triggered rig needs this widened: Spot's images land within ~15 ms of each
    # other and its depth trails its camera by up to ~90 ms, so every set is dropped at 1 ms.
    max_skew_ms: float = 0.0
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

    # The filter itself steps at the IMU rate; this is only how often it emits.
    publish_rate: float = 50.0
    replay_buffer_seconds: float = 0.5
    # Outlier gate in variance units, per measurement dimension. Smaller is more
    # aggressive, 0 is no gate.
    outlier_rejection_allowed_variance: float = 25.0
    # Caps the filter's own state rather than an incoming reading. 0 disables it.
    max_position_m: float = 10000.0

    # Leaving frame_id empty disables the IMU: the filter is seeded level from the first
    # source message and holds its pose between them.
    imu: ImuConfig = Field(default_factory=ImuConfig)
    # m/s^2, seeding the filter rather than fixing it: a ZUPT is meant to refine it later.
    # Worth setting only on good hardware. Local gravity runs 9.780 at the equator to 9.832
    # at the poles, a 0.07 spread, and altitude is a tenth of that (Everest costs 0.027).
    # The BMI055 in a D455 has a 0.69 zero-g offset, ten times the whole spread, so there
    # the number is unmeasurable; an ADIS16505 repeats to 0.02 and can tell the difference.
    initial_gravity_estimate: float = 9.8

    initial_position_std: float = 0.01
    initial_velocity_std: float = 0.1
    initial_rotation_std: float = 0.05
    initial_bias_std: float = 0.05

    # Trust in the tracker's pose, per SourceConfig semantics; the module registers the
    # tracker as a fusion source itself, so it never appears in `odom_sources`.
    visual_odom_pose_variances: list[float] = Field(
        default_factory=lambda: [0.01, 0.01, 0.01, 0.05, 0.05, 0.05],
        min_length=6,
        max_length=6,
    )
    visual_odom_twist_variances: list[float] = Field(
        default_factory=lambda: [0.0] * 6, min_length=6, max_length=6
    )
    # One entry per external source (wheel odometry, ...), matched against each message's
    # header.frame_id and child_frame_id.
    odom_sources: list[SourceConfig] = Field(default_factory=list)
    # A virtual zero-twist measurement applied with every source message, for the
    # directions the platform cannot move in. Above zero pulls that dimension toward
    # zero with this variance; zero leaves it free.
    constraint_twist_variances: list[float] = Field(
        default_factory=lambda: [0.0] * 6, min_length=6, max_length=6
    )

    @model_validator(mode="after")
    def _imu_noise_is_set(self) -> DimSlamConfig:
        if not self.imu.frame_id:
            return self
        missing = [name for name in IMU_NOISE_FIGURES if getattr(self.imu, name) <= 0.0]
        if missing:
            raise ValueError(f"the IMU needs its noise figures set: {', '.join(missing)}")
        if self.imu.init_gyro_limit <= 0.0:
            raise ValueError("imu.init_gyro_limit must be above zero to ever init")
        return self

    @model_validator(mode="after")
    def _sources_are_fusable(self) -> DimSlamConfig:
        # Zero drops a dimension, so an all-zero source is fused in no dimension at all.
        if not any(self.visual_odom_pose_variances) and not any(self.visual_odom_twist_variances):
            raise ValueError(
                "every visual_odom pose and twist variance is zero, "
                "which fuses the tracker in no dimension at all"
            )
        for index, source in enumerate(self.odom_sources):
            if not source.parent_frame_id.strip() or not source.child_frame_id.strip():
                raise ValueError(
                    f"odom_sources[{index}] needs both parent_frame_id and child_frame_id"
                )
            if not any(source.pose_variances) and not any(source.twist_variances):
                raise ValueError(
                    f"odom_sources[{index}] ({source.parent_frame_id!r} -> "
                    f"{source.child_frame_id!r}) has every pose and twist variance at zero, "
                    "which fuses nothing; use a positive variance or drop the source"
                )
        return self


class DimSlam(NativeModule):
    """Every camera publishes onto the same ``image`` and ``camera_info`` streams and is
    told apart by ``frame_id``; the ``cameras`` list fixes which frames are on the rig and
    in what order. Extrinsics come from tf against ``rig_frame_id``. ``depth_image`` feeds
    ``depth_cloud`` in every mode, and is additionally tracked against in ``rgbd``,
    reprojected onto the rig camera through ``depth_camera_info`` and tf when the depth
    sensor differs.

    The tracker's pose stream never touches the wire: it enters the filter directly
    as a drifting source. Any number of external sources
    (wheel odometry, ...) publish onto ``odom_sources`` and are told apart by
    ``header.frame_id`` and ``child_frame_id``. Late messages roll the filter back and replay
    everything after.
    """

    config: DimSlamConfig

    image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]
    depth_camera_info: In[CameraInfo]
    imu: In[Imu]
    odom_sources: In[Odometry]

    odometry: Out[Odometry]
    depth_cloud: Out[PointCloud2]
    tf: IO[TFMessage]

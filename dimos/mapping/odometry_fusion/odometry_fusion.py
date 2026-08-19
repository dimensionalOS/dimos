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

"""Native Rust error-state Kalman filter fusing an IMU with any number of odometry sources."""

from __future__ import annotations

from pathlib import Path

from pydantic import Field

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.mapping.cuvslam.cuvslam import dimslam_build_command
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.tf2_msgs.TFMessage import TFMessage

MODULE_DIR = Path(__file__).resolve().parent


class OdometryFusionConfig(NativeModuleConfig):
    cwd: str | None = str(MODULE_DIR)
    executable: str = "result/bin/odometry_fusion"
    build_command: str | None = Field(default_factory=dimslam_build_command)
    stdin_config: bool = True

    odom_frame: str = "odom"
    base_frame: str = "base_link"
    # The filter itself steps at the IMU rate; this is only how often it emits.
    publish_rate: float = 50.0
    replay_buffer_seconds: float = 0.5
    # Standard deviations per measurement dimension before a reading is called an
    # outlier. 0 disables the gate.
    mahalanobis_gate: float = 5.0

    # With this off the filter is seeded level from the first source message and coasts
    # at constant world velocity between measurements instead of propagating on IMU.
    use_imu: bool = True

    # Bosch BMI055 figures for the D455's IMU, in the continuous-time units the filter
    # wants: rad/s/sqrt(Hz), rad/s^2/sqrt(Hz), m/s^2/sqrt(Hz), m/s^3/sqrt(Hz).
    imu_gyro_noise_density: float = 0.0018
    imu_gyro_random_walk: float = 2e-5
    imu_accel_noise_density: float = 0.02
    imu_accel_random_walk: float = 3e-3
    gravity: float = 9.81
    # Averaged while stationary to level the filter and take the gyro bias. Offline on a
    # 517 s Alfred drive, leaving that bias in cost 19.8 m of final error against 1.6 m
    # with it removed, so this is the single most load-bearing number here. At 200 Hz
    # this is one second of standing still at startup.
    imu_init_samples: int = 200

    initial_position_std: float = 0.01
    initial_velocity_std: float = 0.1
    initial_rotation_std: float = 0.05
    initial_bias_std: float = 0.05

    # One entry per source, matched against each message's header.frame_id. A source
    # whose frame is odom_frame is fused absolutely; anything else is fused as
    # filter-anchored deltas, since its own pose has drifted.
    source_frames: list[str] = Field(default_factory=list)
    # 6 per source, [x y z roll pitch yaw] then [vx vy vz wx wy wz]: below zero takes the
    # message covariance, zero drops the dimension, above zero is a fixed variance.
    source_pose_variances: list[float] = Field(default_factory=list)
    source_twist_variances: list[float] = Field(default_factory=list)
    # A virtual zero-twist measurement applied with every source message, for the
    # directions the platform cannot move in. Above zero pulls that dimension toward
    # zero with this variance; zero leaves it free.
    constraint_twist_variances: list[float] = Field(default_factory=lambda: [0.0] * 6)


class OdometryFusion(NativeModule):
    """Fuses IMU propagation with any number of odometry sources into one pose.

    Every source publishes onto the same ``sources`` stream and is told apart by
    ``header.frame_id``, so adding one is a config change rather than a port change.
    Late messages roll the filter back to their own slot and replay everything after,
    which is what lets a slow source be fused without holding the output back.

    ``odometry`` and ``tf`` both carry ``odom_frame`` -> ``base_frame``.
    """

    config: OdometryFusionConfig

    imu: In[Imu]
    sources: In[Odometry]

    odometry: Out[Odometry]
    tf: IO[TFMessage]

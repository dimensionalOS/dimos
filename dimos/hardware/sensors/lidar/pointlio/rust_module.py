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

"""Python NativeModule wrapper for the message-input Point-LIO rust module
(issue #2821, option 1).

Unlike :class:`dimos.hardware.sensors.lidar.pointlio.module.PointLio` (which
binds the Livox SDK and consumes network packets), this module consumes
``raw_lidar`` (PointCloud2) + ``imu`` (Imu) messages from any driver — or a
replayed recording — and publishes ``lidar`` + ``odometry`` under the same
output port names as the existing PointLio, so downstream consumers wire
identically.

SCAFFOLD: the estimator is a passthrough (identity odometry) until the
implementation direction on #2821 is settled; see rust/src/main.rs.

Build::

    cd dimos/hardware/sensors/lidar/pointlio/rust && cargo build --release
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.cmu_nav.frames import FRAME_ODOM


class PointLioRustConfig(NativeModuleConfig):
    cwd: str | None = "rust"
    executable: str = "target/release/pointlio"
    build_command: str | None = "cargo build --release"
    stdin_config: bool = True

    # Odometry is published map_frame_id (fixed) -> sensor_frame_id (moving);
    # clouds are stamped with sensor_frame_id. Mirrors PointLioConfig, with
    # "frame_id" spelled map_frame_id (reserved on NativeModuleConfig).
    map_frame_id: str = FRAME_ODOM
    sensor_frame_id: str = "mid360_link"


class PointLioRust(NativeModule):
    """Message-input Point-LIO: raw_lidar + imu in, lidar + odometry out."""

    config: PointLioRustConfig

    raw_lidar: In[PointCloud2]
    imu: In[Imu]
    lidar: Out[PointCloud2]
    odometry: Out[Odometry]


# Verify the module constructs (mirrors the other native-module wrappers).
if TYPE_CHECKING:
    PointLioRust()

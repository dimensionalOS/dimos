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

"""A Yesense IMU read straight off its serial port by the native ``yesense_imu`` module (the yesense-imu crate), every frame.

The device owns its serial port, so nothing else may read it at once (on the M20, stop the vendor ``yesense_node``
first). Samples are stamped with their host receive time. With ``body_axes`` (the default) the vectors and orientation
are rotated into the parent frame's axes by the mount, and the frame is parent-aligned; without it they stay in the
sensor's axes and the mount rotation goes into the transform.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from pydantic import Field

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


class YesenseImuConfig(NativeModuleConfig):
    source_dir: str | None = "dimos/hardware/sensors/imu/yesense/rust"
    executable: str = "result/bin/yesense_imu"
    build_command: str | None = "nix build -L path:."
    stdin_config: bool = True
    base_fields: frozenset[str] = frozenset({"frame_id"})

    # The serial port the Yesense streams its frames of accel, gyro and Euler angles on, and its rate.
    device: str = "/dev/ttyUSB0"
    baud: int = Field(default=115200, ge=1, le=4294967295)
    frame_id: str | None = "imu_link"
    parent_frame_id: str = "base_link"
    # The sensor's position (m) and roll, pitch, yaw (rad) in parent_frame_id.
    imu_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    mount_rpy_rad: tuple[float, float, float] = (0.0, 0.0, 0.0)
    body_axes: bool = True
    read_chunk_bytes: int = Field(default=256, ge=1)
    # Bytes the decoder holds waiting for a frame before dropping the oldest.
    max_buffered_bytes: int = Field(default=4096, ge=7)
    retry_s: float = Field(default=2.0, ge=0.1, le=60.0)
    # Period of the static base_link -> imu_link transform.
    tf_period_s: float = Field(default=1.0, ge=0.01, le=60.0)


class YesenseImu(NativeModule):
    """A Yesense IMU as ``sensor_msgs/Imu``, every frame, plus its static transform."""

    config: YesenseImuConfig

    imu: Out[Imu]
    tf: Out[TFMessage]


if TYPE_CHECKING:
    YesenseImu()

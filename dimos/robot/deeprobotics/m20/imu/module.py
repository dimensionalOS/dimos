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

"""The M20's body IMU read straight off its UART by the native ``m20_imu`` module (the yesense-imu crate), every frame, in place of the vendor ``yesense_node``.

The device owns one serial port, so this module and the vendor ``yesense_node`` cannot run at once: stop the
vendor node first. Samples are stamped with their host receive time. With ``body_axes`` (the default) the
vectors and orientation are rotated into base_link's axes as the vendor driver does, and ``imu_link`` is
base_link-aligned; without it they stay in the sensor's axes and the mount rotation goes into the transform.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from pydantic import Field

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.deeprobotics.m20.constants import IMU_FRAME, IMU_MOUNT_RPY_RAD, IMU_XYZ_M


class M20ImuConfig(NativeModuleConfig):
    source_dir: str | None = "dimos/robot/deeprobotics/m20/imu/rust"
    executable: str = "result/bin/m20_imu"
    build_command: str | None = "nix build -L path:."
    stdin_config: bool = True
    base_fields: frozenset[str] = frozenset({"frame_id"})

    # The Yesense on the vendor AOS board: a 16550 UART, 200 Hz frames of accel, gyro and Euler angles.
    device: str = "/dev/ttyS9"
    baud: int = Field(default=115200, ge=1, le=4294967295)
    frame_id: str | None = IMU_FRAME
    parent_frame_id: str = "base_link"
    imu_xyz: tuple[float, float, float] = IMU_XYZ_M
    mount_rpy_rad: tuple[float, float, float] = IMU_MOUNT_RPY_RAD
    body_axes: bool = True
    read_chunk_bytes: int = Field(default=256, ge=1)
    # Bytes the decoder holds waiting for a frame before dropping the oldest.
    max_buffered_bytes: int = Field(default=4096, ge=7)
    retry_s: float = Field(default=2.0, ge=0.1, le=60.0)
    # Period of the static base_link -> imu_link transform.
    tf_period_s: float = Field(default=1.0, ge=0.01, le=60.0)


class M20Imu(NativeModule):
    """The M20's body IMU as ``sensor_msgs/Imu``, every frame, plus its static transform."""

    config: M20ImuConfig

    imu: Out[Imu]
    tf: Out[TFMessage]


if TYPE_CHECKING:
    M20Imu()

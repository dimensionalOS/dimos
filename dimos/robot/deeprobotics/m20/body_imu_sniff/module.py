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

"""The M20's body IMU copied passively off the vendor ``yesense_node``'s DDS samples by the native ``m20_body_imu_sniff``.

Unlike ``YesenseImu`` (``--choice M20BodyImu=serial``) it leaves the serial port to the vendor node and runs beside it: a receive-only raw capture on
loopback (no DDS participant, nothing sent) picks the participant named ``participant`` out of the SPDP
announcements and decodes its ``sensor_msgs/Imu`` samples, which are already in base_link's axes and stamped by the
vendor. The capture needs CAP_NET_RAW, so the host running it needs that capability.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from pydantic import Field

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.deeprobotics.m20.constants import IMU_FRAME, IMU_XYZ_M


class M20BodyImuSniffConfig(NativeModuleConfig):
    source_dir: str | None = "dimos/robot/deeprobotics/m20/body_imu_sniff/rust"
    executable: str = "result/bin/m20_body_imu_sniff"
    build_command: str | None = "nix build -L path:."
    stdin_config: bool = True
    base_fields: frozenset[str] = frozenset({"frame_id"})

    # yesense_node's participant is loopback-only and named "yesense"; its readers sit in the DDS user port range.
    interface: str = "lo"
    participant: str = "yesense"
    # domain 0's SPDP multicast port (Fast-DDS 7400 + 250 * domain)
    discovery_port: int = Field(default=7400, ge=1, le=65535)
    port_min: int = Field(default=7400, ge=1, le=65535)
    port_max: int = Field(default=7700, ge=1, le=65535)
    # RTPS heartbeats / acks here are 132 bytes of UDP and yesense_node's other messages 228; an Imu DATA is 472
    min_udp_bytes: int = Field(default=300, ge=0, le=65535)
    frame_id: str | None = IMU_FRAME
    parent_frame_id: str = "base_link"
    imu_xyz: tuple[float, float, float] = IMU_XYZ_M
    recv_buffer_bytes: int = Field(default=4 << 20, ge=65536, le=2147483647)
    retry_s: float = Field(default=2.0, ge=0.1, le=60.0)
    # Period of the static base_link -> imu_link transform.
    tf_period_s: float = Field(default=1.0, ge=0.01, le=60.0)


class M20BodyImuSniff(NativeModule):
    """The M20's body IMU as ``sensor_msgs/Imu`` at the vendor rate, sniffed off yesense_node, plus its transform."""

    config: M20BodyImuSniffConfig

    imu: Out[Imu]
    # rt/IMU_DATA's roll, pitch, yaw (deg) as yesense_node writes them (what rl_deploy's gravity uses)
    imu_rpy: Out[Vector3]
    tf: Out[TFMessage]


if TYPE_CHECKING:
    M20BodyImuSniff()

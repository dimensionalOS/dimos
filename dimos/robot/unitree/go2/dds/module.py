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

"""GO2DDS: the Go2 over CycloneDDS, as a native module on the robot side.

The rust binary (``rust/``) terminates ``cmd_vel`` as sport ``Move`` and ``command`` verbs
as sport requests, and streams the robot's own odometry (``odom -> base_link``, with the
tf edge), the head L1 cloud and the front camera. Same profile as :class:`GO2Zenoh`, so a
blueprint written against one runs against the other.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Literal

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.BatteryState import BatteryState
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.go2.base import Go2Base, Go2BaseConfig


class GO2DDSConfig(NativeModuleConfig, Go2BaseConfig):
    cwd: str | None = "rust"
    executable: str = "target/release/go2_dds"
    build_command: str | None = "nix develop path:nix -c cargo build --release"
    stdin_config: bool = True

    # Every field below crosses to the rust `Config` verbatim (test_module.py).
    # eth0 on the Go2 itself, the Go2 link on the Jetson.
    iface: str = "eth0"
    domain_id: int = 0
    odom_topic: str = "rt/utlidar/robot_odom"
    # Publish odometry's odom -> base_link edge on tf; off when another LIO owns odom.
    odom_tf: bool = True
    lidar_topic: str = "rt/utlidar/cloud_deskewed"
    # Spin the head L1 up at start (park it otherwise) and stream its deskewed cloud.
    lidar_on: bool = True
    # Also the undeskewed sensor-frame cloud and the L1's own IMU.
    lidar_raw_on: bool = False
    # Body joint states, IMU and battery off rt/lowstate, decimated to lowstate_hz.
    lowstate_on: bool = True
    lowstate_hz: float = 50.0
    # h264 off the RTP multicast onto `video`, or jpeg polled at `video_fps` onto `image`.
    video_on: bool = True
    video_encoding: Literal["h264", "jpeg"] = "h264"
    video_fps: float = 15.0
    video_group: str = "230.1.1.1"
    video_port: int = 1720
    # StopMove once cmd_vel has been silent this long.
    deadman_ms: int = 500

    def _ignore_fields(self) -> set[str]:
        # The rust struct rejects the python-only mount and camera fields.
        return super()._ignore_fields() | set(Go2BaseConfig.model_fields)


class GO2DDS(NativeModule, Go2Base):
    """The Go2 over DDS: sport control in; odometry, clouds, camera and body state out."""

    config: GO2DDSConfig

    image: Out[CompressedImage]  # front camera JPEG, `video_encoding="jpeg"` only
    lidar_raw: Out[PointCloud2]  # undeskewed L1 cloud in its sensor frame, `lidar_raw_on`
    lidar_imu: Out[Imu]  # the L1's own IMU, `lidar_raw_on`
    joint_state: Out[JointState]  # 12 leg joints, Unitree order
    imu: Out[Imu]  # the body IMU, base_link
    battery: Out[BatteryState]  # ~1 Hz
    joy: Out[Joy]  # the handheld remote: 4 stick axes, 16 key bits

    @rpc
    def stop(self) -> None:
        self.liedown()
        time.sleep(0.5)  # let the verb reach the process before it is killed
        super().stop()


if TYPE_CHECKING:
    GO2DDS()

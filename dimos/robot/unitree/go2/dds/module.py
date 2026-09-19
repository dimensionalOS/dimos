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
from typing import TYPE_CHECKING

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.robot.unitree.go2.base import Go2Base, Go2BaseConfig


class GO2DDSConfig(NativeModuleConfig, Go2BaseConfig):
    cwd: str | None = "rust"
    executable: str = "target/release/go2_dds"
    build_command: str | None = "./build.sh"
    stdin_config: bool = True

    # Every field below crosses to the rust `Config` verbatim (test_module.py).
    # The interface CycloneDDS binds: eth0 on the Go2 itself, the Go2 link on the Jetson.
    iface: str = "eth0"
    domain_id: int = 0
    odom_topic: str = "rt/utlidar/robot_odom"
    lidar_topic: str = "rt/utlidar/cloud_deskewed"
    # Spin the head L1 up at start (park it otherwise) and stream its cloud.
    lidar_on: bool = True
    # Join the videohub RTP multicast and stream the front camera.
    video_on: bool = True
    video_group: str = "230.1.1.1"
    video_port: int = 1720
    # Hard clamp on cmd_vel (m/s, m/s, rad/s): the web teleop's fastest.
    max_vx: float = 1.5
    max_vy: float = 0.8
    max_vyaw: float = 1.4
    # StopMove once cmd_vel has been silent this long.
    deadman_ms: int = 500

    def _ignore_fields(self) -> set[str]:
        # The mount and camera fields are python's; the rust struct rejects unknowns.
        return super()._ignore_fields() | set(Go2BaseConfig.model_fields)


class GO2DDS(NativeModule, Go2Base):
    """The Go2 over DDS: sport control in, odometry, L1 cloud and video out."""

    config: GO2DDSConfig

    @rpc
    def stop(self) -> None:
        self.liedown()
        time.sleep(0.5)  # let the verb reach the process before it is killed
        super().stop()


if TYPE_CHECKING:
    GO2DDS()

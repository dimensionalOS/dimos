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

"""NativeModule declaration for the M20's local ROS 2/DrDDS adapter."""

from __future__ import annotations

from typing import TYPE_CHECKING

from pydantic import Field

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.deeprobotics.m20.constants import (
    MAX_ANGULAR_Z_RAD_S,
    MAX_LINEAR_X_M_S,
    MAX_LINEAR_Y_M_S,
)
from dimos.spec import perception


class M20ROSBridgeConfig(NativeModuleConfig):
    """Robot-local ROS topic names, frames, and command watchdog settings."""

    cwd: str | None = "cpp"
    executable: str = "build/m20_ros_bridge"
    build_command: str | None = "./build.sh"
    stdin_config: bool = True

    lidar_topic: str = "/LIDAR/POINTS"
    odom_topic: str = "/ODOM"
    nav_cmd_topic: str = "/NAV_CMD"
    location_status_topic: str = "/LOCATION_STATUS"
    hes_status_topic: str = "/HES_STATUS"
    node_name: str = "dimos_m20_bridge"

    cloud_frame: str = "base_link"
    world_frame: str = "odom"
    base_frame: str = "base_link"

    enable_command_output: bool = False
    command_rate_hz: float = Field(default=10.0, gt=0.0)
    command_timeout_s: float = Field(default=0.4, gt=0.0)
    safety_timeout_s: float = Field(default=2.5, gt=0.0)
    max_linear_x: float = Field(default=MAX_LINEAR_X_M_S, gt=0.0)
    max_linear_y: float = Field(default=MAX_LINEAR_Y_M_S, gt=0.0)
    max_angular_z: float = Field(default=MAX_ANGULAR_Z_RAD_S, gt=0.0)


class M20ROSBridge(NativeModule, perception.Lidar, perception.Odometry):
    """Bridge M20 ROS 2 topics to local typed DimOS streams.

    This process runs on GOS and links against the robot's installed Foxy and
    ``drdds`` packages. It does not route through Zenoh or another robot host.
    """

    config: M20ROSBridgeConfig

    safe_cmd_vel: In[Twist]
    command_ready: Out[Bool]
    lidar: Out[PointCloud2]
    odom: Out[PoseStamped]
    odometry: Out[Odometry]
    tf: Out[TFMessage]


if TYPE_CHECKING:
    M20ROSBridge()

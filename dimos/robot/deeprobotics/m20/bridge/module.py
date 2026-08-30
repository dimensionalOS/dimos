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

"""NativeModule declaration for the M20 command/state ROS 2 adapter."""

from __future__ import annotations

from typing import TYPE_CHECKING

from pydantic import Field

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.std_msgs.Int32 import Int32
from dimos.msgs.std_msgs.UInt32 import UInt32
from dimos.robot.deeprobotics.m20.constants import (
    MAX_ANGULAR_Z_RAD_S,
    MAX_LINEAR_X_M_S,
    MAX_LINEAR_Y_M_S,
)


class M20ROSBridgeConfig(NativeModuleConfig):
    """Robot-local command/state topics and watchdog settings."""

    cwd: str | None = "cpp"
    executable: str = "build/m20_ros_bridge"
    build_command: str | None = "./build.sh"
    stdin_config: bool = True
    # GOS installs the vendor ROS 2/Foxy and drdds libraries here. NativeModule
    # workers do not inherit an interactive shell's ROS setup, so make the
    # runtime dependency explicit and reproducible.
    extra_env: dict[str, str] = Field(
        default_factory=lambda: {
            "FASTRTPS_DEFAULT_PROFILES_FILE": "/opt/robot/fastdds.xml",
            "LD_LIBRARY_PATH": "/opt/ros/foxy/lib",
            "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
        }
    )

    nav_cmd_topic: str = "/NAV_CMD"
    motion_state_topic: str = "/MOTION_STATE"
    motion_info_topic: str = "/MOTION_INFO"
    gait_topic: str = "/GAIT"
    hes_status_topic: str = "/HES_STATUS"
    node_name: str = "dimos_m20_bridge"

    command_rate_hz: float = Field(default=10.0, gt=0.0)
    command_timeout_s: float = Field(default=0.4, gt=0.0)
    safety_timeout_s: float = Field(default=2.5, gt=0.0)
    max_linear_x: float = Field(default=MAX_LINEAR_X_M_S, gt=0.0)
    max_linear_y: float = Field(default=MAX_LINEAR_Y_M_S, gt=0.0)
    max_angular_z: float = Field(default=MAX_ANGULAR_Z_RAD_S, gt=0.0)


class M20ROSBridge(NativeModule):
    """Bridge M20 command and state topics to typed DimOS streams.

    This process runs on GOS and links against the robot's installed Foxy and
    ``drdds`` packages. M20PointLio subscribes to lidar and IMU directly; this
    bridge carries only low-bandwidth command and robot-state traffic.
    """

    config: M20ROSBridgeConfig

    safe_cmd_vel: In[Twist]
    motion_state_cmd: In[Int32]
    gait_cmd: In[UInt32]
    command_ready: Out[Bool]
    motion_state: Out[Int32]
    gait_state: Out[UInt32]


if TYPE_CHECKING:
    M20ROSBridge()

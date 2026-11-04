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

# will be deleted on the arrival of sim2

"""A simulated Go2 (MuJoCo, from dim-go2-sim) behind connections/webrtc.py's ports and RPCs, so a blueprint swaps
one for the other. Its lidar is the sim's Mid-360 scan (mid360_link), not the L1 map."""

from __future__ import annotations

from typing import Any

from dim_go2_sim import Go2Sim
from dim_go2_sim.config import UrdfConfig
from dim_go2_sim.module import Go2SimConfig
from dim_go2_sim.policy import JOINT_NAMES as SIM_JOINT_NAMES
from pydantic import Field

from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.BatteryState import BatteryState
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.String import String
from dimos.robot.unitree.go2.config import JOINT_NAMES
from dimos.spec.perception import Camera, Pointcloud
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class Go2WebRtcSimConfig(Go2SimConfig):  # type: ignore[misc]
    # UrdfManagerNative owns the URDF and the leg tf; the sim feeds it joint_state instead
    urdf: UrdfConfig = Field(default_factory=lambda: UrdfConfig(enabled=False))


class Go2WebRtcSim(Go2Sim, Camera, Pointcloud):  # type: ignore[misc]
    """dim-go2-sim's Go2 with webrtc.py's extra ports (command, battery, joint_state); verbs the sim has no
    counterpart for succeed as no-ops."""

    config: Go2WebRtcSimConfig
    command: In[String]
    pointcloud: Out[PointCloud2]
    battery: Out[BatteryState]
    joint_state: Out[JointState]

    def _publish_pose(self, stamp: float) -> None:
        super()._publish_pose(stamp)
        # the sim's legs, in the lowstate's motor order under the URDF's names
        qpos = dict(
            zip(SIM_JOINT_NAMES, self.world.data.qpos[self.world._qpos].tolist(), strict=True)
        )
        self.joint_state.publish(
            JointState(ts=stamp, name=list(JOINT_NAMES), position=[qpos[n] for n in JOINT_NAMES])
        )

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Send movement command to robot (held for the sim's command_timeout)."""
        self._on_cmd_vel(twist)
        return True

    @rpc
    def stop_movement(self) -> None:
        """Zero the base immediately."""
        self._on_cmd_vel(Twist())

    @rpc
    def standup(self) -> bool:
        """The sim always stands."""
        return True

    @rpc
    def liedown(self) -> bool:
        """The sim has no lie-down."""
        return True

    @rpc
    def balance_stand(self) -> bool:
        """The sim always balances."""
        return True

    @rpc
    def set_rage_mode(self, enable: bool) -> bool:
        """No Rage Mode in the sim."""
        return True

    @rpc
    def sport_command(self, api_id: int) -> bool:
        """No sport commands in the sim."""
        return True

    @rpc
    def set_light(self, level: int) -> bool:
        """No head light in the sim."""
        return True

    @rpc
    def set_obstacle_avoidance(self, enabled: bool = True) -> bool:
        """No onboard obstacle avoidance in the sim."""
        return True

    @rpc
    def switch_joystick(self, enable: bool = True) -> bool:
        """No joystick in the sim."""
        return True

    @rpc
    def battery_soc(self) -> int | None:
        """The sim has no battery."""
        return None

    @rpc
    def publish_request(self, topic: str, data: dict[str, Any]) -> None:
        """The sim answers no RTC requests."""
        logger.info(f"sim Go2 ignores RTC request {topic} {data.get('api_id')}")

# Copyright 2025-2026 Dimensional Inc.
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

"""Republishes a twist base's wheel odometry (x, y, theta, carried on
ControlCoordinator's joint state via hardware_interface.py's read_odometry())
as a standard nav_msgs.Odometry message. Works for any TwistBaseAdapter that
implements read_odometry() — FlowBase, mock, etc — not hardware-specific."""

from __future__ import annotations

import math

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.JointState import JointState


class Config(ModuleConfig):
    base_frame: str = "base"
    world_frame: str = "world"


class TwistBaseOdometry(Module):
    """ControlCoordinator joint state (x, y, theta) -> nav_msgs.Odometry."""

    config: Config

    joint_state: In[JointState]
    odometry:    Out[Odometry]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(self.joint_state.subscribe(self._on_joint_state))

    def _on_joint_state(self, msg: JointState) -> None:
        try:
            x = msg.position[msg.name.index(f"{self.config.base_frame}/vx")]
            y = msg.position[msg.name.index(f"{self.config.base_frame}/vy")]
            theta = msg.position[msg.name.index(f"{self.config.base_frame}/wz")]
        except ValueError:
            return

        half_theta = theta / 2.0
        self.odometry.publish(
            Odometry(
                ts=msg.ts,
                frame_id=self.config.world_frame,
                child_frame_id=self.config.base_frame,
                pose=Pose(
                    position=[x, y, 0.0],
                    orientation=[0.0, 0.0, math.sin(half_theta), math.cos(half_theta)],
                ),
            )
        )

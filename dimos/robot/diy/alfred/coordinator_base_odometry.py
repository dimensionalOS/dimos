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

"""The pose the coordinator's twist base reports, as the FlowBase's wheel odometry.

On the robot ``AlfredHighLevel`` polls the FlowBase's odometry and publishes it on
``wheel_odometry``, which ``WheelOdometryRelay`` turns into ``odom -> base_link`` and
``/flowbase/odom``. In a simulation the base is the coordinator's ``mock_twist_base`` with
``integrate_odometry``, whose pose is its three joints' positions (x, y, yaw) in
``coordinator_joint_state``. This publishes that pose the way AlfredHighLevel does (same
message, frames and rate), so the relay and everything after it run unchanged.
"""

from __future__ import annotations

import math

from pydantic import Field, FiniteFloat
from reactivex.disposable import Disposable

from dimos.control.components import make_twist_base_joints
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.JointState import JointState


class CoordinatorBaseOdometryConfig(ModuleConfig):
    # The base's joints, in (x, y, yaw) order: their positions are its integrated pose.
    base_joints: list[str] = Field(default_factory=lambda: make_twist_base_joints("flowbase"))
    # AlfredHighLevel's defaults: the rate it polls the FlowBase at and the frames it names.
    wheel_odometry_hz: FiniteFloat = Field(50.0, gt=0.0)
    wheel_odom_frame_id: str = "wheel_odom"
    base_frame_id: str = "base_link"


class CoordinatorBaseOdometry(Module):
    """``coordinator_joint_state`` in, the twist base's pose as ``wheel_odometry`` out."""

    config: CoordinatorBaseOdometryConfig

    coordinator_joint_state: In[JointState]
    wheel_odometry: Out[Odometry]

    _previous: tuple[float, float, float, float] | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._previous = None
        self.register_disposable(Disposable(self.coordinator_joint_state.subscribe(self._relay)))

    def _relay(self, state: JointState) -> None:
        positions = dict(zip(state.name, state.position, strict=False))
        if not all(joint in positions for joint in self.config.base_joints):
            return
        x, y, yaw = (float(positions[joint]) for joint in self.config.base_joints)
        ts = float(state.ts)
        previous = self._previous
        # The coordinator publishes every tick (100 Hz); the FlowBase is polled at 50.
        if previous is not None and ts - previous[0] < 0.9 / self.config.wheel_odometry_hz:
            return
        twist = Twist()
        if previous is not None and ts > previous[0]:
            # As AlfredHighLevel does: the controller reports no velocity, so difference poses.
            last_ts, last_x, last_y, last_yaw = previous
            dt = ts - last_ts
            turn = math.atan2(math.sin(yaw - last_yaw), math.cos(yaw - last_yaw))
            heading = last_yaw + turn / 2.0
            dx, dy = x - last_x, y - last_y
            twist = Twist(
                linear=Vector3(
                    (math.cos(heading) * dx + math.sin(heading) * dy) / dt,
                    (-math.sin(heading) * dx + math.cos(heading) * dy) / dt,
                    0.0,
                ),
                angular=Vector3(0.0, 0.0, turn / dt),
            )
        self._previous = (ts, x, y, yaw)
        self.wheel_odometry.publish(
            Odometry(
                ts=ts,
                frame_id=self.config.wheel_odom_frame_id,
                child_frame_id=self.config.base_frame_id,
                pose=Pose(
                    position=Vector3(x, y, 0.0),
                    orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
                ),
                twist=twist,
            )
        )

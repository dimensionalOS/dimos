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

"""The FlowBase's own odometry as the base's pose, for a stack without a lidar.

``alfred-full`` gets ``odom -> base_link`` from Point-LIO and relays it to the coordinator's
base adapter as ``start_pose``. ``alfred-manipulation`` runs no lidar: this relay turns
AlfredHighLevel's ``wheel_odometry`` into the same ``start_pose`` and puts ``odom -> base_link``
on tf, so a whole-body plan's base segment has a pose to track and the Controller a fixed frame.
"""

from __future__ import annotations

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


class WheelOdometryRelayConfig(ModuleConfig):
    world_frame: str = "odom"
    base_frame: str = "base_link"


class WheelOdometryRelay(Module):
    """``wheel_odometry`` in, ``start_pose`` and the ``odom -> base_link`` edge out."""

    config: WheelOdometryRelayConfig

    wheel_odometry: In[Odometry]
    start_pose: Out[PoseStamped]
    tf: Out[TFMessage]

    @rpc
    def start(self) -> None:
        super().start()
        self._unsubscribe = self.wheel_odometry.subscribe(self._relay)

    @rpc
    def stop(self) -> None:
        self._unsubscribe()
        super().stop()

    def _relay(self, odometry: Odometry) -> None:
        pose = odometry.pose.pose
        stamped = PoseStamped(
            odometry.ts,
            self.config.world_frame,
            position=pose.position,
            orientation=pose.orientation,
        )
        self.start_pose.publish(stamped)
        edge = Transform.from_pose(self.config.base_frame, stamped)
        edge.frame_id = self.config.world_frame
        edge.ts = odometry.ts
        self.tf.publish(TFMessage(edge))


wheel_odometry_relay = WheelOdometryRelay.blueprint

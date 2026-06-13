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

"""Adapter at the robot/sim -> navigation boundary.

Robot and sim layers publish the base pose as ``PoseStamped`` on ``/odom``
(the dimos convention: a lightweight pose, no velocity or covariance). The
nav_stack modules (planners, costmappers, terrain analysis) are ROS-derived
and consume ``nav_msgs.Odometry`` instead. This Module upcasts the former to
the latter — empty twist/covariance, ``child_frame_id="base_link"`` — so a
PoseStamped-producing robot can feed the Odometry-consuming nav stack without
either side changing its contract.
"""

from __future__ import annotations

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry


class PoseStampedToOdometry(Module):
    """Bridge PoseStamped odometry streams to nav_msgs.Odometry for native nav modules."""

    odom: In[PoseStamped]
    odometry: Out[Odometry]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))

    def _on_odom(self, msg: PoseStamped) -> None:
        self.odometry.publish(
            Odometry(
                ts=msg.ts,
                frame_id=msg.frame_id,
                child_frame_id="base_link",
                pose=msg,
            )
        )


__all__ = ["PoseStampedToOdometry"]

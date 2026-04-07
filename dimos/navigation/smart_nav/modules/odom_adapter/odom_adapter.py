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

"""OdomAdapter: bidirectional PoseStamped <-> Odometry converter.

Bridges GO2Connection (PoseStamped odom) with PGO (Odometry).
Also converts PGO's corrected Odometry back to PoseStamped for
downstream consumers (ReplanningAStarPlanner, WavefrontFrontierExplorer).
"""

from __future__ import annotations

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry


class OdomAdapter(Module[ModuleConfig]):
    """Bidirectional PoseStamped <-> Odometry adapter."""

    default_config = ModuleConfig

    raw_odom: In[PoseStamped]
    odometry: Out[Odometry]
    corrected_odometry: In[Odometry]
    odom: Out[PoseStamped]

    @rpc
    def start(self) -> None:
        self.raw_odom.subscribe(self._on_raw_odom)
        self.corrected_odometry.subscribe(self._on_corrected_odom)
        print("[OdomAdapter] Started")

    def _on_raw_odom(self, msg: PoseStamped) -> None:
        odom = Odometry(
            ts=msg.ts,
            frame_id=msg.frame_id,
            pose=Pose(
                position=[msg.x, msg.y, msg.z],
                orientation=[
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w,
                ],
            ),
        )
        self.odometry.publish(odom)

    def _on_corrected_odom(self, msg: Odometry) -> None:
        ps = PoseStamped(
            ts=msg.ts,
            frame_id=msg.frame_id,
            position=[msg.x, msg.y, msg.z],
            orientation=[
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w,
            ],
        )
        self.odom.publish(ps)

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

"""Static tf edges a Go2 + Mid-360 pcap replay needs on top of PointLio's odometry.

PointLio already rebroadcasts its odometry as the moving ``odom -> body`` tf edge,
so a replay only lacks the fixed edges: the ``map -> odom`` root (identity until a
downstream PGO overwrites it with the loop-closure correction), the ``body ->
base_link`` bridge (Point-LIO's body frame and the rig's base_link are the same
physical frame), and the Go2/Mid-360 sensor mounts. Publishing all of these lets a
recorder capture a fully connected ``map -> odom -> body(=base_link) -> front_camera
-> {mid360_link, camera_optical}`` tree.

The stock ``StaticTfPublisher`` stamps each cycle with wall-clock ``time.time()``.
Under a rate-scaled replay that clock diverges from PointLio's data-time odometry
stamps, so the static edges would only cover the wall duration of the run and leave
the tree disconnected across the rest of the data-time trajectory. Instead this
module re-stamps the fixed edges off the odometry stream, so they track the exact
data-time span PointLio produces (throttled so consecutive edges stay well inside a
tf buffer's match tolerance).
"""

from __future__ import annotations

from pydantic import Field

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.navigation.cmu_nav.frames import FRAME_BODY, FRAME_MAP, FRAME_ODOM
from dimos.protocol.tf.static_tf_publisher import frames_to_edge_transforms
from dimos.robot.unitree.go2.go2_mid360_static_transforms import FRAMES

# The rig's base_link and Point-LIO's body frame are the same physical frame.
FRAME_BASE_LINK = "base_link"


def _identity_edge(parent: str, child: str) -> Transform:
    return Transform(
        translation=Vector3(0.0, 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id=parent,
        child_frame_id=child,
    )


def _replay_static_edges() -> list[Transform]:
    return [
        *frames_to_edge_transforms(FRAMES),
        _identity_edge(FRAME_MAP, FRAME_ODOM),
        _identity_edge(FRAME_BODY, FRAME_BASE_LINK),
    ]


class Go2Mid360ReplayStaticTfConfig(ModuleConfig):
    # Min data-time gap between republishing the static edges; keep well under a tf
    # buffer's match tolerance so any query lands next to a stamped edge.
    republish_min_interval_s: float = Field(default=0.2, gt=0.0)


class Go2Mid360ReplayStaticTf(Module):
    """Republishes the Go2/Mid-360 mount tree plus the two identity edges PointLio's
    odom tf omits, stamped on the odometry data clock."""

    config: Go2Mid360ReplayStaticTfConfig

    odometry: In[Odometry]

    _edges: list[Transform] = []
    _last_emit_ts: float | None = None

    async def handle_odometry(self, value: Odometry) -> None:
        if not self._edges:
            self._edges = _replay_static_edges()
        if self._last_emit_ts is not None:
            if value.ts - self._last_emit_ts < self.config.republish_min_interval_s:
                return
        self._last_emit_ts = value.ts
        for edge in self._edges:
            edge.ts = value.ts
        self.tf.publish(*self._edges)

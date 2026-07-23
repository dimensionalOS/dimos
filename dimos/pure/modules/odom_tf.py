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

"""Odometry -> tf: assert the frame_id<-body_frame edge from each odom pose.

A world-frame odom pose *is* the frame_id<-body_frame transform; this restamps
it as a ``tf_out`` assertion so downstream tf() samplers (a planner's position,
a mapper's pose) resolve the robot pose through the shared buffer. Generic over
any odom source whose pose is already in ``frame_id`` (go2 LIO, sim, ...).
"""

from __future__ import annotations

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.pure import pm


class OdomTf(pm.PureModule):
    """Restamp each odom pose as a frame_id<-body_frame tf assertion."""

    frame_id: str = "world"  # frame the odom pose is expressed in
    body_frame: str = "base_link"  # robot frame the pose locates

    class In(pm.In):
        odom: PoseStamped = pm.tick(expect_hz=18)

    class Out(pm.Out):
        tf: Transform = pm.tf_out("{frame_id}", "{body_frame}")

    def step(self, i: In) -> Out:
        """Emit the pose as a frame_id<-body_frame transform at the odom ts."""
        tf = Transform(
            translation=i.odom.position,
            rotation=i.odom.orientation,
            frame_id=self.frame_id,
            child_frame_id=self.body_frame,
            ts=i.ts,
        )
        tf.ts = i.ts  # ctor swaps an explicit ts=0.0 for wall clock; force the data-time ts
        return OdomTf.Out(tf=tf)

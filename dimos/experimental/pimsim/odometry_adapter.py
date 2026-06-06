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

"""Adapters bridging pimsim's ``/odom`` to what the nav stack expects.

The browser publishes the integrated base pose as ``geometry_msgs/PoseStamped``
on ``/odom`` (frame ``map``). The nav stack wants two things the sim doesn't
provide directly:
- ``nav_msgs/Odometry`` on ``/odometry`` (TerrainAnalysis, planners, PGO);
- a TF ``map -> body`` so the (python) SimplePlanner can read the robot pose.

``PoseStampedToOdometry`` and ``OdomTfBroadcaster`` supply those so a
``create_nav_stack`` blueprint can autoconnect onto the babylon sim.
"""

from __future__ import annotations

from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry

DEFAULT_WORLD_FRAME = "map"
# Nav stack frame names (dimos.navigation.nav_stack.frames); kept as literals so
# this pimsim helper doesn't import the nav stack. The Odometry child frame is
# the same "body" frame the TF below names, so a later frame lookup is consistent.
NAV_BODY_FRAME = "body"
NAV_SENSOR_FRAME = "sensor"
DEFAULT_CHILD_FRAME = NAV_BODY_FRAME
# Lidar sensor mount offset on the body, in metres. Shared with the
# SceneLidarModule config (see blueprints/_factory.py) so the TF here and the
# lidar's own sensor offset can't drift apart.
LIDAR_SENSOR_X = 0.15
LIDAR_SENSOR_Z = 0.10


class PoseStampedToOdometry(Module):
    """Republish a ``PoseStamped`` stream as ``nav_msgs/Odometry``."""

    pose: In[PoseStamped]
    odometry: Out[Odometry]

    async def handle_pose(self, value: PoseStamped) -> None:
        pose = Pose()
        pose.position = value.position
        pose.orientation = value.orientation
        self.odometry.publish(
            Odometry(
                pose=pose,
                frame_id=value.frame_id or DEFAULT_WORLD_FRAME,
                child_frame_id=DEFAULT_CHILD_FRAME,
                ts=value.ts,
            )
        )


class OdomTfBroadcaster(Module):
    """Broadcast TF ``map -> body`` and ``body -> sensor`` from ``/odom``."""

    pose: In[PoseStamped]

    async def handle_pose(self, value: PoseStamped) -> None:
        body = Transform.from_pose(NAV_BODY_FRAME, value)
        sensor = Transform(
            translation=Vector3(LIDAR_SENSOR_X, 0.0, LIDAR_SENSOR_Z),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id=NAV_BODY_FRAME,
            child_frame_id=NAV_SENSOR_FRAME,
            ts=value.ts,
        )
        self.tf.publish(body, sensor)

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

"""Accumulate an odometry stream into the path it has travelled."""

from __future__ import annotations

import math
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path


class OdometryPathConfig(ModuleConfig):
    # Frame the path is stamped in. Left empty it follows the odometry's own
    # ``frame_id``, which is what puts the line under the right node of the tf
    # tree in a viewer.
    frame_id: str = ""
    # Poses closer together than this are dropped. A stationary robot otherwise
    # piles thousands of identical points onto the same spot, and every one of
    # them is re-encoded on every publish.
    min_step_m: float = 0.02
    # Oldest poses fall off past this. Decimating instead would keep the whole
    # history, but a trail whose shape changes under you is worse than one with a
    # known, honest horizon.
    max_poses: int = 20000
    # Publish rate ceiling, in seconds between messages. The path is republished
    # whole, so at 30 Hz odometry an unbounded rate spends more time serializing
    # the trail than tracking.
    min_publish_interval_s: float = 0.1


class OdometryPath(Module):
    """``odometry`` in, the trail it has drawn out.

    A viewer shows odometry as a pose: where the robot is now, and nothing about
    where it has been. This keeps the history and republishes it as a
    ``nav_msgs/Path``, which renders as a line.

    The trail inherits the odometry's drift -- it is where the *estimator* thinks
    the robot went. Feeding it a SLAM-corrected pose instead makes the trail jump
    at every loop closure, so prefer the continuous odometry and let the viewer's
    ``map`` -> ``odom`` edge carry the correction.
    """

    config: OdometryPathConfig

    odometry: In[Odometry]

    path: Out[Path]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._poses: list[PoseStamped] = []
        self._last_publish_ts = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odometry)))

    def _on_odometry(self, msg: Odometry) -> None:
        position = msg.pose.position
        previous = self._poses[-1] if self._poses else None
        if previous is not None and (
            math.dist(
                (previous.x, previous.y, previous.z),
                (position.x, position.y, position.z),
            )
            < self.config.min_step_m
        ):
            return

        orientation = msg.pose.orientation
        self._poses.append(
            PoseStamped(
                ts=msg.ts,
                frame_id=self.config.frame_id or msg.frame_id,
                position=[position.x, position.y, position.z],
                orientation=[orientation.x, orientation.y, orientation.z, orientation.w],
            )
        )
        if len(self._poses) > self.config.max_poses:
            del self._poses[: len(self._poses) - self.config.max_poses]

        if msg.ts - self._last_publish_ts < self.config.min_publish_interval_s:
            return
        self._last_publish_ts = msg.ts
        # A copy, because Path holds the list by reference and the next pose would
        # otherwise mutate a message already on its way out.
        self.path.publish(
            Path(
                ts=msg.ts,
                frame_id=self.config.frame_id or msg.frame_id,
                poses=list(self._poses),
            )
        )

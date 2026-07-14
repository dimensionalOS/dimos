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

"""Alternate clicks between start and goal pose streams for downstream planners."""

from __future__ import annotations

from typing import Any

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class ClickStartGoalRouterConfig(ModuleConfig):
    world_frame: str = "map"


class ClickStartGoalRouter(Module):
    """Alternates between sending start and goal poses on clicks."""

    config: ClickStartGoalRouterConfig

    clicked_point: In[PointStamped]
    start_pose: Out[PoseStamped]
    goal_pose: Out[PoseStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._next_is_start: bool = True

    async def handle_clicked_point(self, msg: PointStamped) -> None:
        pose = PoseStamped(
            ts=msg.ts,
            frame_id=self.config.world_frame,
            position=[msg.x, msg.y, msg.z],
            orientation=[0.0, 0.0, 0.0, 1.0],
        )
        if self._next_is_start:
            self._next_is_start = False
            logger.info("Click set start; next click will set goal", x=msg.x, y=msg.y, z=msg.z)
            self.start_pose.publish(pose)
            return
        self._next_is_start = True
        logger.info("Click set goal", x=msg.x, y=msg.y, z=msg.z)
        self.goal_pose.publish(pose)

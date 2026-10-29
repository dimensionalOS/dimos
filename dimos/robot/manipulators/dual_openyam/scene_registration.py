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

"""Scene registration whose scene cloud has the robot's own returns removed.

The pick's grasp contact check reads the full scene cloud from scene registration and
stands the arm in it. An overhead camera sees the arms' bases and links, so without
this the arm meets its own image at every grasp (scene 56 replay, 2026-10-29). The
planner's map already drops those returns through UrdfManagerPointClearing; this
applies the same body filter to the scene the check reads.
"""

from __future__ import annotations

import threading
from typing import Any

import numpy as np
from pydantic import Field

from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.experimental.object_scene_registration import (
    ObjectSceneRegistrationConfig,
    ObjectSceneRegistrationModule,
)
from dimos.robot.assets.model import RobotModel
from dimos.robot.roboplan_body_filter import RoboPlanBodyFilter
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class SelfClearedSceneRegistrationConfig(ObjectSceneRegistrationConfig):
    # The robot whose returns leave the scene cloud: the arms the camera can see.
    model: RobotModel
    # How far past the model's collision geometry a return still counts as the robot.
    padding_m: float = Field(default=0.01, ge=0.0)
    # Oldest joint state that may stand for the cloud's capture time; the arm holds
    # still while scanning.
    state_tolerance_s: float = Field(default=0.5, gt=0.0)


class SelfClearedSceneRegistrationModule(ObjectSceneRegistrationModule):
    """Object scene registration whose full scene cloud leaves out the robot's body."""

    config: SelfClearedSceneRegistrationConfig

    coordinator_joint_state: In[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._state_lock = threading.Lock()
        self._joint_state: JointState | None = None
        self._body_filter: RoboPlanBodyFilter | None = None
        self._root_link = self.config.model.load().root_link

    @rpc
    def start(self) -> None:
        super().start()
        self._body_filter = RoboPlanBodyFilter(self.config.model.load(), self.config.padding_m)

    async def handle_coordinator_joint_state(self, state: JointState) -> None:
        with self._state_lock:
            self._joint_state = state

    def _state_for(self, stamp: float) -> dict[str, float] | None:
        with self._state_lock:
            state = self._joint_state
        if state is None or abs(float(state.ts) - stamp) > self.config.state_tolerance_s:
            return None
        return dict(zip(state.name, state.position, strict=True))

    @rpc
    def get_full_scene_pointcloud(
        self,
        exclude_object_id: str | None = None,
        depth_trunc: float = 2.0,
        voxel_size: float = 0.01,
    ) -> PointCloud2 | None:
        """The registration module's scene cloud without the points on the robot.

        None when the robot's pose at the capture is unknown: the check then reports
        itself skipped rather than running against the robot's own image.
        """
        scene = super().get_full_scene_pointcloud(exclude_object_id, depth_trunc, voxel_size)
        if scene is None or self._body_filter is None:
            return scene
        stamp = float(scene.ts)
        joints = self._state_for(stamp)
        root_from_scene = self.tfbuffer.get(
            self._root_link, scene.frame_id, stamp, self.config.state_tolerance_s
        )
        if joints is None or root_from_scene is None:
            logger.warning("Scene cloud dropped: robot pose at the capture time is unknown")
            return None
        points = scene.as_numpy()[0].astype(np.float64)
        matrix = root_from_scene.to_matrix()
        keep = self._body_filter.keep_mask(points @ matrix[:3, :3].T + matrix[:3, 3], joints, {})
        if keep is None:
            logger.warning("Scene cloud dropped: joint state is missing a model joint")
            return None
        return PointCloud2.from_numpy(
            points[keep].astype(np.float32), frame_id=scene.frame_id, timestamp=stamp
        )

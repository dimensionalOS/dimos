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

"""Localize whose scene cloud has the robot's own returns removed.

The pick's grasp contact check reads the scene cloud from localize and stands the arm
in it. An overhead camera sees the arms' bases and links, so without this the arm meets
its own image at every grasp. The planner's map already drops those returns through
UrdfManagerPointClearing; this applies the same body filter to the scene the check reads.
"""

from __future__ import annotations

import threading
from typing import Any

import numpy as np
from numpy.typing import NDArray
from pydantic import Field

from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.manipulation.manipulation_localize import (
    ManipulationLocalize,
    ManipulationLocalizeConfig,
)
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.assets.model import RobotModel
from dimos.robot.roboplan_body_filter import RoboPlanBodyFilter
from dimos.types.timestamped import TimestampedBufferCollection
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class SelfClearedManipulationLocalizeConfig(ManipulationLocalizeConfig):
    # The robot whose returns leave the scene cloud: the arms the camera can see.
    model: RobotModel
    # How far past the model's collision geometry a return still counts as the robot.
    padding_m: float = Field(default=0.01, ge=0.0)
    # How far from a depth view's capture time a joint state may be to stand for it.
    state_tolerance_s: float = Field(default=0.5, gt=0.0)
    # How long joint states are kept: a window as old as horizon_s can be asked for.
    state_history_s: float = Field(default=600.0, gt=0.0)


class SelfClearedManipulationLocalize(ManipulationLocalize):
    """ManipulationLocalize whose scene cloud leaves out the robot's body."""

    config: SelfClearedManipulationLocalizeConfig

    coordinator_joint_state: In[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._states = TimestampedBufferCollection[JointState](self.config.state_history_s)
        # Joint states arrive on the module's loop; scene_cloud reads them from an RPC thread.
        self._states_lock = threading.Lock()
        self._body_filter: RoboPlanBodyFilter | None = None
        # RPCs run on a thread pool, and the filter is one native scene: one call at a time.
        self._filter_lock = threading.Lock()
        self._root_link = self.config.model.load().root_link

    @rpc
    def start(self) -> None:
        super().start()
        self._body_filter = RoboPlanBodyFilter(self.config.model.load(), self.config.padding_m)

    async def handle_coordinator_joint_state(self, state: JointState) -> None:
        with self._states_lock:
            self._states.add(state)

    def _state_at(self, stamp: float) -> dict[str, float] | None:
        with self._states_lock:
            state = self._states.find_closest(stamp, self.config.state_tolerance_s)
        if state is None:
            return None
        return dict(zip(state.name, state.position, strict=True))

    @rpc
    def scene_cloud(self, start: float = -5.0, duration: float = 5.0) -> PointCloud2 | None:
        """ManipulationLocalize's scene cloud without the points on the robot.

        The window's depth views are from different times and the arm may move between
        them, so each view loses the robot's points at the joint state of its own
        capture, before the views merge. A view whose robot pose is unknown is left out.
        None when no view is left: the check then reports itself skipped rather than
        running against the robot's own image.

        Args:
            start: Where the window starts, seconds, as in ``localize``.
            duration: How long the window is, seconds.
        """
        if self._body_filter is None or not self._ready.is_set():
            return None
        if self.config.geometry != "depth":
            return None
        # The model root is fixed in the world, so the latest edge is the capture-time one.
        root_from_world = self.rig.tf.get(self._root_link, self.rig.world_frame)
        if root_from_world is None:
            logger.warning("Scene cloud dropped: the robot model's root frame is not on tf")
            return None
        matrix = root_from_world.to_matrix()
        # The window and the merge are ManipulationLocalize.scene_cloud's.
        first, head = self.index.get_time_range()
        lo = max(first, head + start if start < 0 else first + start)
        cleared: list[NDArray[np.float32]] = []
        newest = 0.0
        unknown = 0
        with self._filter_lock:
            for obs in self.index.time_range(lo, lo + duration):
                for frame in self.rig.cameras:
                    view = self.rig.backdrop(obs.ts, frame, self.config.scene_depth_trunc_m)
                    if view is None or not len(view):
                        continue
                    points = self._off_robot(view, matrix)
                    if points is None:
                        unknown += 1
                        continue
                    cleared.append(points)
                    newest = max(newest, view.ts or 0.0)
        if unknown:
            logger.warning(
                "Scene views left out: robot pose at their capture time is unknown",
                left_out=unknown,
                kept=len(cleared),
            )
        if not cleared:
            return None
        return PointCloud2.from_numpy(
            np.concatenate(cleared), frame_id=self.rig.world_frame, timestamp=newest
        ).voxel_downsample(self.config.scene_voxel_m)

    def _off_robot(
        self, view: PointCloud2, root_from_world: NDArray[np.float64]
    ) -> NDArray[np.float32] | None:
        """One depth view's points that are not on the robot as it stood at the capture.

        Args:
            view: The view's cloud in the world frame, stamped with its capture time.
            root_from_world: 4x4 transform from the world frame to the model's root frame.

        Returns None when no joint state is close enough to the view's capture time, which
        an unstamped view never has, or the joint state lacks a joint the model moves.
        """
        if self._body_filter is None:
            return None
        joints = self._state_at(view.ts or 0.0)
        if joints is None:
            return None
        # Thinned to the scene's cell before the filter, as the check reads it: a raw
        # 640x480 view is up to 300,000 points and the window holds several.
        points = view.voxel_downsample(self.config.scene_voxel_m).points_f32()
        root_points = np.asarray(
            points @ root_from_world[:3, :3].T + root_from_world[:3, 3], dtype=np.float64
        )
        keep = self._body_filter.keep_mask(root_points, joints, {})
        if keep is None:
            return None
        return np.asarray(points[keep], dtype=np.float32)

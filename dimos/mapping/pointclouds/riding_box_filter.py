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

"""Leave what rides on the robot, such as its gripper and what it holds, out of a camera's cloud."""

from __future__ import annotations

import asyncio
import math
from threading import RLock
import time

import numpy as np
from numpy.typing import NDArray
from pydantic import BaseModel

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.pointclouds.box_filter import Box, inside_boxes
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import pose_to_matrix

logger = setup_logger()

UNPLACED_WARNING_PERIOD_S = 5.0


class RidingBox(BaseModel):
    """A box on a moving frame, such as the gripper on its tool frame."""

    name: str
    frame_id: str
    # Its centre in that frame and its full edge lengths, metres.
    center: tuple[float, float, float] = (0.0, 0.0, 0.0)
    size: tuple[float, float, float]


class RidingBoxFilterConfig(ModuleConfig):
    # Boxes from the start, such as the gripper; ride_box and drop_box work on them too.
    boxes: list[RidingBox] = []
    # How far from a cloud's capture time its boxes' frames may be read, seconds.
    tf_tolerance_s: float = 0.1


class RidingBoxFilter(Module):
    """Pass a camera's cloud on without the points in boxes riding on moving frames.

    Each box is placed where its frame was when the cloud was captured. It
    belongs before a mapper, as the robot's self filter does: a mapper keeps
    what it is given, so a gripper passed on once would stay in the map where
    it was, long after it moved. While a box's frame is not in tf at a cloud's
    capture time, the cloud is dropped rather than passed with the box's points.
    """

    config: RidingBoxFilterConfig

    cloud: In[PointCloud2]
    tf: In[TFMessage]
    filtered_cloud: Out[PointCloud2]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._lock = RLock()
        self._boxes: dict[str, Box] = {
            box.name: Box(Pose(position=Vector3(*box.center)), Vector3(*box.size), box.frame_id)
            for box in self.config.boxes
        }
        self._unplaced_warned_at = -math.inf

    @rpc
    def start(self) -> None:
        self.tfbuffer  # noqa: B018 - Initialize TF before binding input handlers.
        super().start()

    async def handle_cloud(self, cloud: PointCloud2) -> None:
        await asyncio.to_thread(self._on_cloud, cloud)

    @rpc
    def ride_box(self, name: str, pose: Pose, size: Vector3, frame_id: str) -> bool:
        """Leave the points in a box on a moving frame out. Setting a name again replaces it.

        Args:
            name: The caller's label for the box; drop it with the same name.
            pose: The box's centre and orientation in ``frame_id``, metres.
            size: Full edge lengths along the box's own axes, metres.
            frame_id: The frame the box rides on, such as the tool tip.
        """
        if not name:
            raise ValueError("Box name must not be empty")
        if any(edge <= 0.0 or not math.isfinite(edge) for edge in size.to_tuple()):
            raise ValueError("Box size must have three positive, finite edges")
        with self._lock:
            self._boxes[name] = Box(pose, size, frame_id)
        return True

    @rpc
    def drop_box(self, name: str) -> bool:
        """Let the points in a named box through again. False if there is no such box."""
        with self._lock:
            return self._boxes.pop(name, None) is not None

    def _on_cloud(self, cloud: PointCloud2) -> None:
        with self._lock:
            filtered = self._filtered(cloud)
        if filtered is not None:
            self.filtered_cloud.publish(filtered)

    def _filtered(self, cloud: PointCloud2) -> PointCloud2 | None:
        points = cloud.points_f32()
        if not self._boxes or not len(points):
            return cloud
        placed = [self._placed(box, cloud) for box in self._boxes.values()]
        boxes = [box for box in placed if box is not None]
        if len(boxes) < len(placed):
            return None
        keep = ~inside_boxes(points, boxes)
        filtered = PointCloud2(frame_id=cloud.frame_id, ts=cloud.ts, seq=cloud.seq)
        for name, values in cloud.pointcloud_tensor.point.items():
            filtered.pointcloud_tensor.point[name] = values[keep]
        return filtered

    def _placed(
        self, box: Box, cloud: PointCloud2
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]] | None:
        """The box's pose in the cloud's frame at its capture time, and its edge lengths;
        None while its frame is not in tf then."""
        assert box.frame_id is not None
        transform = (
            self.tfbuffer.get(
                cloud.frame_id, box.frame_id, cloud.ts, self.config.tf_tolerance_s, warn=False
            )
            if box.frame_id != cloud.frame_id
            else None
        )
        if box.frame_id != cloud.frame_id and transform is None:
            # Every few seconds, not once: a misspelt frame drops every cloud.
            now = time.monotonic()
            if now - self._unplaced_warned_at >= UNPLACED_WARNING_PERIOD_S:
                self._unplaced_warned_at = now
                logger.warning(
                    "No transform from %s to %s at a cloud's capture time; clouds are "
                    "dropped until there is",
                    cloud.frame_id,
                    box.frame_id,
                )
            return None
        pose = pose_to_matrix(box.pose)
        if transform is not None:
            pose = transform.to_matrix() @ pose
        return pose, np.asarray(box.size.to_tuple(), dtype=np.float64)

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

"""Leave the points inside named boxes out of a point cloud."""

from __future__ import annotations

import asyncio
from collections.abc import Iterable
from dataclasses import dataclass
import math
from threading import RLock

import numpy as np
from numpy.typing import NDArray

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import pose_to_matrix

logger = setup_logger()


@dataclass(frozen=True)
class Box:
    """A box to leave out of a cloud.

    Args:
        pose: The box's centre and orientation, in ``frame_id``.
        size: Full edge lengths along the box's own axes, metres.
        frame_id: The frame the box is fixed in, or None for the cloud's own
            frame. A moving frame, such as a robot's tool tip, carries the box.
    """

    pose: Pose
    size: Vector3
    frame_id: str | None = None


def inside_boxes(
    points: NDArray[np.floating], boxes: Iterable[tuple[NDArray[np.float64], NDArray[np.float64]]]
) -> NDArray[np.bool_]:
    """Which points lie inside any of the boxes.

    Args:
        points: N x 3 positions, metres.
        boxes: Each box as (its 4x4 pose in the points' frame, its full edge
            lengths in metres).
    """
    inside = np.zeros(len(points), dtype=bool)
    for pose, size in boxes:
        # Each point in the box's own axes, R^T (p - t), written for row vectors.
        local = (np.asarray(points, dtype=np.float64) - pose[:3, 3]) @ pose[:3, :3]
        inside |= np.all(np.abs(local) <= size / 2.0, axis=1)
    return inside


class PointCloudBoxFilter(Module):
    """Pass a point cloud on with the points inside named boxes left out.

    A box is fixed in the cloud's frame or rides on another frame from tf.
    Changing the boxes re-sends the last cloud at once, so a consumer sees
    the change before the next cloud arrives.
    """

    cloud: In[PointCloud2]
    tf: In[TFMessage]
    filtered_cloud: Out[PointCloud2]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._lock = RLock()
        self._boxes: dict[str, Box] = {}
        self._last_cloud: PointCloud2 | None = None

    @rpc
    def start(self) -> None:
        self.tfbuffer  # noqa: B018 - Initialize TF before binding input handlers.
        super().start()

    async def handle_cloud(self, cloud: PointCloud2) -> None:
        await asyncio.to_thread(self._on_cloud, cloud)

    @rpc
    def set_box(self, name: str, pose: Pose, size: Vector3, frame_id: str | None = None) -> bool:
        """Leave the points inside a box out. Setting a name again replaces that box.

        Args:
            name: The caller's label for the box; clear it with the same name.
            pose: The box's centre and orientation in ``frame_id``, metres.
            size: Full edge lengths along the box's own axes, metres.
            frame_id: The frame the box is fixed in, or None for the cloud's frame.
        """
        if not name:
            raise ValueError("Box name must not be empty")
        if any(edge <= 0.0 or not math.isfinite(edge) for edge in size.to_tuple()):
            raise ValueError("Box size must have three positive, finite edges")
        with self._lock:
            self._boxes[name] = Box(pose, size, frame_id)
            self._resend()
        return True

    @rpc
    def clear_box(self, name: str) -> bool:
        """Let the points inside a named box through again. False if there is no such box."""
        with self._lock:
            removed = self._boxes.pop(name, None) is not None
            if removed:
                self._resend()
        return removed

    def _on_cloud(self, cloud: PointCloud2) -> None:
        with self._lock:
            self._last_cloud = cloud
            self.filtered_cloud.publish(self._filtered(cloud))

    def _resend(self) -> None:
        if self._last_cloud is not None:
            self.filtered_cloud.publish(self._filtered(self._last_cloud))

    def _filtered(self, cloud: PointCloud2) -> PointCloud2:
        points = cloud.points_f32()
        boxes = [placed for box in self._boxes.values() if (placed := self._placed(box, cloud))]
        if not boxes or not len(points):
            return cloud
        keep = ~inside_boxes(points, boxes)
        filtered = PointCloud2(frame_id=cloud.frame_id, ts=cloud.ts, seq=cloud.seq)
        for name, values in cloud.pointcloud_tensor.point.items():
            filtered.pointcloud_tensor.point[name] = values[keep]
        return filtered

    def _placed(
        self, box: Box, cloud: PointCloud2
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]] | None:
        """The box's pose in the cloud's frame and its edge lengths; None while its frame is unknown."""
        pose = pose_to_matrix(box.pose)
        if box.frame_id is not None and box.frame_id != cloud.frame_id:
            transform = self.tfbuffer.get(cloud.frame_id, box.frame_id, warn=False)
            if transform is None:
                logger.warning(
                    "No transform from %s to %s yet; box points stay in the cloud",
                    cloud.frame_id,
                    box.frame_id,
                )
                return None
            pose = transform.to_matrix() @ pose
        return pose, np.asarray(box.size.to_tuple(), dtype=np.float64)

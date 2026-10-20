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

"""Shared capture ordering and stream handling for point cloud self filters."""

from __future__ import annotations

import asyncio
from threading import RLock

import numpy as np
from numpy.typing import NDArray
from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.types.timestamped import TimestampedBufferCollection
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class PointCloudSelfFilterConfig(ModuleConfig):
    tf_tolerance_s: float = Field(default=0.02, ge=0.0)
    tf_forward_tolerance_s: float = Field(default=0.05, ge=0.0)
    state_tolerance_s: float = Field(default=0.02, ge=0.0)
    state_history_s: float = Field(default=5.0, gt=0.0)


class PointCloudSelfFilter(Module):
    """Shared self-filter streams; subclasses supply a capture-time keep mask."""

    config: PointCloudSelfFilterConfig

    pointcloud: In[PointCloud2]
    tf: In[TFMessage]
    coordinator_joint_state: In[JointState]
    filtered_pointcloud: Out[PointCloud2]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._filter_lock = RLock()
        self._states = TimestampedBufferCollection[JointState](self.config.state_history_s)
        self._last_capture: float | None = None

    @rpc
    def start(self) -> None:
        self.tfbuffer  # noqa: B018 - Initialize TF before binding input handlers.
        super().start()

    async def handle_pointcloud(self, cloud: PointCloud2) -> None:
        """Filter the latest capture without starving TF transport callbacks."""
        await asyncio.to_thread(self._on_pointcloud, cloud)

    async def handle_coordinator_joint_state(self, state: JointState) -> None:
        await asyncio.to_thread(self.add_joint_state, state)

    def add_joint_state(self, state: JointState) -> None:
        """Buffer full model state; out-of-order arrivals remain timestamped."""
        with self._filter_lock:
            if np.isfinite(state.ts):
                self._states.remove_by_timestamp(state.ts)
                self._states.add(JointState(state))
                latest = self._states.last()
                if latest is not None:
                    self._states.prune_old(latest.ts - self.config.state_history_s)

    def filter_cloud(self, cloud: PointCloud2) -> PointCloud2 | None:
        """Filter one aligned capture without changing its frame or point fields."""
        with self._filter_lock:
            if not np.isfinite(cloud.ts) or (
                self._last_capture is not None and cloud.ts < self._last_capture
            ):
                logger.warning("Dropping cloud: invalid or out-of-order capture timestamp")
                return None
            points = cloud.points_f32()
            if not np.isfinite(points).all():
                return None
            keep = self._compute_keep_mask(cloud, points)
            if keep is None:
                return None
            filtered = PointCloud2(frame_id=cloud.frame_id, ts=cloud.ts, seq=cloud.seq)
            for name, values in cloud.pointcloud_tensor.point.items():
                filtered.pointcloud_tensor.point[name] = values[keep]
            self._last_capture = cloud.ts
            return filtered

    def _on_pointcloud(self, cloud: PointCloud2) -> None:
        with self._filter_lock:
            filtered = self.filter_cloud(cloud)
            if filtered is not None:
                self.filtered_pointcloud.publish(filtered)

    def _compute_keep_mask(
        self, cloud: PointCloud2, points: NDArray[np.float32]
    ) -> NDArray[np.bool_] | None:
        """Return one keep flag per point, or None when the capture cannot be filtered.

        Called under the filter lock. Implementations must align any required
        state and transforms to cloud.ts and preserve the input point order.
        """
        raise NotImplementedError("Choose a concrete point cloud self filter")

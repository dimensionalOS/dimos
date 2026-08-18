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

"""Capture-triggered conversion from point-cloud TF to mapper Odometry."""

from __future__ import annotations

import asyncio

from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class PointCloudTfPoseSourceConfig(ModuleConfig):
    fixed_frame: str = Field(default="world", min_length=1)
    tf_tolerance_s: float = Field(default=0.02, ge=0.0)
    tf_forward_tolerance_s: float = Field(default=0.05, ge=0.0)


class PointCloudTfPoseSource(Module):
    """Publish one capture-stamped sensor pose for each point cloud."""

    config: PointCloudTfPoseSourceConfig  # type: ignore[assignment]
    pointcloud: In[PointCloud2]
    tf: In[TFMessage]
    odometry: Out[Odometry]

    @rpc
    def start(self) -> None:
        # Subscribe to TF before accepting clouds. The producer publishes a
        # capture-aligned TF immediately before each cloud.
        _ = self.tfbuffer
        super().start()

    async def handle_pointcloud(self, cloud: PointCloud2) -> None:
        """Resolve capture-time TF without blocking the transport receive thread."""
        await asyncio.to_thread(self.on_pointcloud, cloud)

    def on_pointcloud(self, cloud: PointCloud2) -> bool:
        """Publish the fixed-frame sensor pose accepted for this capture."""
        if not cloud.frame_id:
            logger.warning("Point-cloud pose unavailable: cloud frame is empty")
            return False
        transform = self.tfbuffer.get(
            self.pose_config.fixed_frame,
            cloud.frame_id,
            time_point=cloud.ts,
            time_tolerance=self.pose_config.tf_tolerance_s,
            forward_tolerance=self.pose_config.tf_forward_tolerance_s,
        )
        if transform is None:
            return False
        self.odometry.publish(
            Odometry(
                ts=cloud.ts,
                frame_id=self.pose_config.fixed_frame,
                child_frame_id=cloud.frame_id,
                pose=Pose(transform.translation, transform.rotation),
            )
        )
        return True

    @property
    def pose_config(self) -> PointCloudTfPoseSourceConfig:
        return self.config  # type: ignore[return-value]


point_cloud_tf_pose_source = PointCloudTfPoseSource.blueprint

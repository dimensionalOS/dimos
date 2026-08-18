# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Capture-triggered conversion from point-cloud TF to mapper Odometry."""

from __future__ import annotations

from pydantic import Field
from reactivex.disposable import Disposable

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
        super().start()
        self.register_disposable(Disposable(self.pointcloud.subscribe(self.on_pointcloud)))

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

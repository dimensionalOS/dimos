# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

from __future__ import annotations

from typing import Literal

import numpy as np
from pydantic import BaseModel, Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class SelfFilterRegion(BaseModel):
    """A primitive, anchored in a TF frame, to exclude from a cloud."""

    shape: Literal["sphere", "box"]
    frame_id: str
    radius: float | None = None
    size: tuple[float, float, float] | None = None
    center: tuple[float, float, float] = (0.0, 0.0, 0.0)


class PointCloudSelfFilterConfig(ModuleConfig):
    regions: list[SelfFilterRegion] = Field(default_factory=list)
    tf_tolerance_s: float = 0.1
    drop_cloud_on_missing_tf: bool = False


class PointCloudSelfFilter(Module):
    """Remove robot-body points from a point cloud using TF-anchored regions."""

    config: PointCloudSelfFilterConfig  # type: ignore[assignment]

    pointcloud: In[PointCloud2]
    filtered_pointcloud: Out[PointCloud2]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        for region in self.filter_config.regions:
            if region.shape == "sphere" and (region.radius is None or region.radius < 0.0):
                raise ValueError("sphere regions require a non-negative radius")
            if region.shape == "box" and (
                region.size is None or any(edge < 0.0 for edge in region.size)
            ):
                raise ValueError("box regions require non-negative size=(x, y, z)")

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.pointcloud.subscribe(self._on_pointcloud)))

    @rpc
    def stop(self) -> None:
        super().stop()

    def filter_cloud(self, cloud: PointCloud2) -> PointCloud2 | None:
        """Filter one cloud; missing TF either skips its region or drops the cloud."""
        points = cloud.points_f32()
        keep = np.ones(len(points), dtype=bool)

        for region in self.filter_config.regions:
            if len(points) == 0:
                break
            transform = self.tf.get(
                cloud.frame_id,
                region.frame_id,
                time_point=cloud.ts,
                time_tolerance=self.filter_config.tf_tolerance_s,
            )
            if transform is None:
                logger.warning(
                    "Missing TF for PointCloudSelfFilter region %s -> %s",
                    cloud.frame_id,
                    region.frame_id,
                )
                if self.filter_config.drop_cloud_on_missing_tf:
                    return None
                continue

            points_h = np.column_stack((points, np.ones(len(points), dtype=np.float32)))
            local = (np.linalg.inv(transform.to_matrix()) @ points_h.T).T[:, :3]
            local -= np.asarray(region.center, dtype=np.float32)
            if region.shape == "sphere":
                assert region.radius is not None
                inside = np.einsum("ij,ij->i", local, local) <= region.radius**2
            else:
                assert region.size is not None
                inside = np.all(np.abs(local) <= np.asarray(region.size) / 2.0, axis=1)
            keep &= ~inside

        intensities = cloud.intensities_f32()
        filtered = PointCloud2.from_numpy(
            points[keep],
            frame_id=cloud.frame_id,
            timestamp=cloud.ts,
            intensities=intensities[keep] if intensities is not None else None,
        )
        # PointCloud2 currently exposes frame/timestamp and point attributes as
        # its metadata. Preserve optional attributes not accepted by from_numpy.
        for name, values in cloud.pointcloud_tensor.point.items():
            if name not in ("positions", "intensities"):
                filtered.pointcloud_tensor.point[name] = values[keep]
        return filtered

    def _on_pointcloud(self, cloud: PointCloud2) -> None:
        filtered = self.filter_cloud(cloud)
        if filtered is not None:
            self.filtered_pointcloud.publish(filtered)

    @property
    def filter_config(self) -> PointCloudSelfFilterConfig:
        return self.config  # type: ignore[return-value]


point_cloud_self_filter = PointCloudSelfFilter.blueprint

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

from __future__ import annotations

from typing import Literal

import numpy as np
from pydantic import BaseModel, Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class SelfFilterRegion(BaseModel):
    """TF-anchored primitive exclusion region."""

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
    """Remove points inside configured TF-anchored self-filter primitives."""

    config: PointCloudSelfFilterConfig  # type: ignore[assignment]

    pointcloud: In[PointCloud2]
    tf: In[TFMessage]
    filtered_pointcloud: Out[PointCloud2]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        for region in self.self_filter_config.regions:
            if region.shape == "sphere" and (region.radius is None or region.radius < 0.0):
                raise ValueError("sphere regions require a non-negative radius")
            if region.shape == "box" and (
                region.size is None or any(edge < 0.0 for edge in region.size)
            ):
                raise ValueError("box regions require non-negative size=(x, y, z)")

    @rpc
    def start(self) -> None:
        super().start()
        unsub = self.pointcloud.subscribe(self._on_pointcloud)
        self.register_disposable(Disposable(unsub))

    @rpc
    def stop(self) -> None:
        super().stop()

    def filter_cloud(self, cloud: PointCloud2) -> PointCloud2 | None:
        """Return a filtered cloud, or None when configured to drop on missing TF."""

        points = cloud.points_f32()
        if len(points) == 0 or not self.self_filter_config.regions:
            return PointCloud2.from_numpy(
                points,
                frame_id=cloud.frame_id,
                timestamp=cloud.ts,
                intensities=cloud.intensities_f32(),
            )

        keep = np.ones(len(points), dtype=bool)
        ones = np.ones((len(points), 1), dtype=np.float32)
        points_h = np.column_stack((points, ones))

        for region in self.self_filter_config.regions:
            transform = self.tfbuffer.get(
                cloud.frame_id,
                region.frame_id,
                time_point=cloud.ts,
                time_tolerance=self.self_filter_config.tf_tolerance_s,
            )
            if transform is None:
                logger.warning(
                    "Missing TF for PointCloudSelfFilter region %s -> %s",
                    cloud.frame_id,
                    region.frame_id,
                )
                if self.self_filter_config.drop_cloud_on_missing_tf:
                    return None
                continue

            region_from_cloud = np.linalg.inv(transform.to_matrix())
            local = (region_from_cloud @ points_h.T).T[:, :3] - np.asarray(region.center)
            if region.shape == "sphere":
                assert region.radius is not None
                inside = np.einsum("ij,ij->i", local, local) <= region.radius * region.radius
            else:
                assert region.size is not None
                half_size = np.asarray(region.size, dtype=np.float32) / 2.0
                inside = np.all(np.abs(local) <= half_size, axis=1)
            keep &= ~inside

        intensities = cloud.intensities_f32()
        filtered_intensities = intensities[keep] if intensities is not None else None
        return PointCloud2.from_numpy(
            points=points[keep],
            frame_id=cloud.frame_id,
            timestamp=cloud.ts,
            intensities=filtered_intensities,
        )

    def _on_pointcloud(self, cloud: PointCloud2) -> None:
        filtered = self.filter_cloud(cloud)
        if filtered is not None:
            self.filtered_pointcloud.publish(filtered)

    @property
    def self_filter_config(self) -> PointCloudSelfFilterConfig:
        return self.config  # type: ignore[return-value]


point_cloud_self_filter = PointCloudSelfFilter.blueprint

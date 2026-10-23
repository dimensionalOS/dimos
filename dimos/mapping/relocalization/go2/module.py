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

"""Go2 relocalization: lidar relocalization plus the premap merged into the live map."""

from copy import deepcopy

import numpy as np
from reactivex import combine_latest

from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.mapping.relocalization.lidar.module import LidarConfig, LidarWindowRelocalization
from dimos.mapping.voxels.grid import VoxelGrid
from dimos.message_utils.pointcloud import (
    concatenate_clouds,
    pointcloud_from_xyz,
    pointcloud_xyz,
    transform_cloud,
)
from dimos.message_utils.time import to_seconds
from dimos.messages.geometry_msgs.msg import TransformStamped
from dimos.messages.sensor_msgs.msg import PointCloud2
from dimos.utils.reactive import backpressure


class Go2Config(LidarConfig):
    use_carving: bool = True


class Go2Relocalization(LidarWindowRelocalization):
    """Lidar relocalization that also publishes premap + live scan as `merged_map` for the Go2 costmap."""

    config: Go2Config
    # The costmap wants everything mapped so far, not the relocalizer's window.
    global_map: In[PointCloud2]
    merged_map: Out[PointCloud2]

    @rpc
    def start(self) -> None:
        super().start()
        if self.premap is None:
            return
        self.register_disposable(
            backpressure(
                combine_latest(
                    self.global_map.observable(),  # type: ignore[no-untyped-call]
                    self.fixes,
                )
            ).subscribe(self._on_merge_input)
        )

    def _on_merge_input(self, pair: tuple[PointCloud2, TransformStamped]) -> None:
        local, tf = pair
        assert self.premap is not None
        premap_in_world = transform_cloud(self.premap, tf)
        if self.config.use_carving:
            grid = VoxelGrid(
                carve_columns=True, frame_id=local.header.frame_id, show_startup_log=False
            )
            try:
                grid.add_points(
                    np.asarray(pointcloud_xyz(premap_in_world), dtype=np.float32),
                    timestamp=to_seconds(premap_in_world.header.stamp),
                )
                grid.add_points(
                    np.asarray(pointcloud_xyz(local), dtype=np.float32),
                    timestamp=to_seconds(local.header.stamp),
                )
                self.merged_map.publish(
                    pointcloud_from_xyz(
                        grid.get_global_pointcloud().point["positions"].numpy(),
                        header=deepcopy(local.header),
                    )
                )
            finally:
                grid.dispose()
        else:
            self.merged_map.publish(concatenate_clouds(local, premap_in_world))

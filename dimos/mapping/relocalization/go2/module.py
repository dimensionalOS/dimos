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

from reactivex import combine_latest

from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.mapping.relocalization.lidar.module import LidarConfig, LidarWindowRelocalization
from dimos.mapping.voxels.grid import VoxelGrid
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
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
                    self._world_to_map,
                )
            ).subscribe(self._on_merge_input)
        )

    def _on_merge_input(self, pair: tuple[PointCloud2, Transform]) -> None:
        local, tf = pair
        assert self.premap is not None
        premap_in_world = self.premap.transform(tf)
        if self.config.use_carving:
            grid = VoxelGrid(carve_columns=True, frame_id=local.frame_id, show_startup_log=False)
            try:
                grid.add_frame(premap_in_world)
                grid.add_frame(local)
                self.merged_map.publish(grid.get_global_pointcloud2())
            finally:
                grid.dispose()
        else:
            self.merged_map.publish(local + premap_in_world)

#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.voxels import VoxelGridMapper
from dimos.memory2.embed import EmbedImages
from dimos.memory2.module import Recorder
from dimos.memory2.transform import QualityWindow
from dimos.models.embedding.clip import CLIPModel
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)
from dimos.navigation.patrolling.module import PatrollingModule
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic

unitree_go2 = autoconnect(
    unitree_go2_basic,
    VoxelGridMapper.blueprint(),
    CostMapper.blueprint(),
    ReplanningAStarPlanner.blueprint(),
    WavefrontFrontierExplorer.blueprint(),
    PatrollingModule.blueprint(),
).global_config(n_workers=9, robot_model="unitree_go2")


class Go2Memory(Recorder):
    color_image: In[Image]
    lidar: In[PointCloud2]

    @rpc
    def start(self) -> None:
        super().start()

        embedded = self._store.stream("color_image_embedded", Image)
        clip = self.register_disposable(CLIPModel())

        # fmt: off
        self.register_disposable(
            self._store.streams.color_image \
               .live() \
               .filter(lambda obs: obs.data.brightness > 0.1) \
               .transform(QualityWindow(lambda img: img.sharpness, window=0.5)) \
               .transform(EmbedImages(clip, batch_size=2)) \
               .save(embedded) \
               .drain())
        # fmt: on

    @rpc
    def stop(self) -> None:
        super().stop()


unitree_go2_memory = autoconnect(
    unitree_go2,
    Go2Memory.blueprint(),
).global_config(n_workers=10)

__all__ = ["unitree_go2", "unitree_go2_memory"]

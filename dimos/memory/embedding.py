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

from collections.abc import Callable
from dataclasses import dataclass, field
from typing import cast

import reactivex as rx
from reactivex import operators as ops
from reactivex.observable import Observable

from dimos.core import In, rpc
from dimos.core.module import Module, ModuleConfig
from dimos.models.embedding.base import Embedding, EmbeddingModel
from dimos.models.embedding.clip import CLIPModel
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.sensor_msgs.Image import Image, sharpness_barrier
from dimos.utils.reactive import getter_hot


@dataclass
class Config(ModuleConfig):
    embedding_model: EmbeddingModel = field(default_factory=CLIPModel)


@dataclass
class SpatialEntry:
    image: Image
    pose: PoseStamped


@dataclass
class SpatialEmbedding(SpatialEntry):
    embedding: Embedding


class EmbeddingMemory(Module[Config]):
    default_config = Config
    config: Config
    color_image: In[Image]
    global_costmap: In[OccupancyGrid]

    _costmap_getter: Callable[[], OccupancyGrid] | None = None

    def get_costmap(self) -> OccupancyGrid:
        if self._costmap_getter is None:
            self._costmap_getter = getter_hot(self.global_costmap.pure_observable())
            self._disposables.add(self._costmap_getter)
        return self._costmap_getter()

    @rpc
    def query_costmap(self, text: str) -> OccupancyGrid:
        costmap = self.get_costmap()
        # overlay costmap with embedding heat
        return costmap

    @rpc
    def start(self) -> None:
        # would be cool if this sharpness_barrier was somehow self-calibrating
        #
        # we need a Governor system, sharpness_barrier frequency shouldn't
        # be a fixed float but an observable that adjusts based on downstream load
        #
        # (also voxel size for mapper for example would benefit from this)
        self.color_image.pure_observable().pipe(
            sharpness_barrier(0.5),
            ops.flat_map(self._try_create_spatial_entry),
            ops.map(self._embed_spatial_entry),
            ops.map(self._store_spatial_entry),
        ).subscribe(print)

    def _try_create_spatial_entry(self, img: Image) -> Observable[SpatialEntry]:
        pose = self.tf.get_pose("world", "base_link")
        if not pose:
            return rx.empty()
        return rx.of(SpatialEntry(image=img, pose=pose))

    def _embed_spatial_entry(self, spatial_entry: SpatialEntry) -> SpatialEmbedding:
        embedding = cast("Embedding", self.config.embedding_model.embed(spatial_entry.image))
        return SpatialEmbedding(
            image=spatial_entry.image,
            pose=spatial_entry.pose,
            embedding=embedding,
        )

    def _store_spatial_entry(self, spatial_embedding: SpatialEmbedding) -> SpatialEmbedding:
        return spatial_embedding

    def query_text(self, query: str) -> list[SpatialEmbedding]:
        self.config.embedding_model.embed_text(query)
        results: list[SpatialEmbedding] = []
        return results

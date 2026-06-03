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

import pickle
from typing import TypeVar

from dimos.mapping.occupancy.inflation import simple_inflate
from dimos.mapping.pointclouds.occupancy import general_occupancy
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.transform import normalize, smooth
from dimos.memory2.vis.color import Color
from dimos.memory2.vis.space.elements import Point
from dimos.memory2.vis.space.space import Space
from dimos.memory2.vis.utils import mosaic
from dimos.models.embedding.clip import CLIPModel
from dimos.utils.data import get_data

T = TypeVar("T")

store = SqliteStore(path=get_data("go2_bigoffice.db"))
global_map = pickle.loads(get_data("unitree_go2_bigoffice_map.pickle").read_bytes())
costmap = simple_inflate(general_occupancy(global_map), 0.05)

clip = CLIPModel()
embedded = store.streams.color_image_embedded

drawing = Space()
# drawing.add(costmap)
# drawing.add(global_map)

search_text = "robot"
search_vector = clip.embed_text(search_text)

# store.streams.color_image.transform(speed()).transform(smooth(30)).transform(normalize()).tap(
#    lambda obs: drawing.add(Point(obs.pose_stamped, color=Color.from_cmap("turbo", obs.data)))
# ).drain()

store.streams.color_image.map(lambda obs: obs.derive(data=obs.data.brightness)).transform(
    smooth(30)
).transform(normalize()).tap(
    lambda obs: drawing.add(Point(obs.pose_stamped, color=Color.from_cmap("turbo", obs.data)))
).drain()


# # fmt: off
# embedded.search(search_vector, k=10) \
#         .tap(drawing.add) \
#         .tap(lambda obs: drawing.add(store.streams.lidar.at(obs.ts).first().data)).drain()
# # fmt: on

from dimos.models.vl.moondream import MoondreamVlModel

moondream = MoondreamVlModel()
moondream.start()

from dimos.models.vl.florence import Florence2Model

florence = Florence2Model()
florence.start()

search_results = (
    embedded.search(search_vector, k=18)
    .tap(lambda obs: drawing.add(obs.derive(data=florence.caption(obs.data))))
    .map(lambda obs: obs.derive(data=moondream.query_detections(obs.data, search_text)))
    .materialize()
)

drawing.add(mosaic(search_results))

# fmt: off
search_results \
        .tap(drawing.add) \
        .tap(lambda obs: drawing.add(store.streams.lidar.at(obs.ts).first().data)) \
        .drain()
# fmt: on

drawing.to_rerun()

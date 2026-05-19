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

# import Transform
from dimos.memory2.transform import Transformer
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def relocalize(current_map: PointCloud2, loaded_map: PointCloud2) -> Transform | None:
    return Transform(parent_frame="map", child_frame="world")


# load all lidar frames captured in the readius around the semantic peaks
# feed them into a global mapper to get a single pointcloud around our areas of interest
map_to_world_transform = lidar.transform(VoxelMapTransformer()).transform(
    RelocalizationTransformer()
)


class RelocalizationTransformer(Transformer[PointCloud2, Transform]):
    def __init__(self, loaded_map: PointCloud2):
        self.loaded_map = loaded_map

    def __call__(
        self, upstream: Iterator[Observation[PointCloud2]]
    ) -> Iterator[Observation[Transform]]:
        for current_map in upstream:
            transform = relocalize(current_map, self.loaded_map)
            if transform:
                yield transform


class RelocalizationModule:
    global_map: In[PointCloud2]
    loaded_map: Out[PointCloud2]

    def start(self):
        super().start()

        loaded_map = load_file("map.pcd")
        self.loaded_map.publish(loaded_map)
        self.relocalization_transformer = RelocalizationTransformer(loaded_map)

        for global_map in self.global_map:
            transform = self.relocalization_transformer(global_map)
            if transform:
                self.tf.publish(transform)


class GlobalLookupModule:
    loaded_map: In[PointCloud2]

    object_locations: {
        "self_charging_dock": PoseStamped(frame_id="map", pose=Pose(10, 0, 0)),
        "plant": PoseStamped(frame_id="map", pose=Pose(10, 10, 0)),
    }

    def start(self):
        super().start()
        self._map = None
        self.loaded_map.subscribe(self._on_map)

    def _on_map(self, msg: PointCloud2):
        self._map = msg

    # gives you relative pose of object in base_link frame, or None if not found
    def lookup(self, query: str) -> Transform | None:
        if not self._map:
            # no relocalization until we have a map
            return None

        return Transform.from_pose(self.object_locations[query], frame_id="base_link")

        return Transform.from_pose(self.object_locations[query], frame_id="base_link")
        return Transform.from_pose(self.object_locations[query], frame_id="base_link")

        return Transform.from_pose(self.object_locations[query], frame_id="base_link")
        return Transform.from_pose(self.object_locations[query], frame_id="base_link")

        return Transform.from_pose(self.object_locations[query], frame_id="base_link")
        return Transform.from_pose(self.object_locations[query], frame_id="base_link")

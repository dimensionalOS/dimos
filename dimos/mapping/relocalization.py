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

import threading, time
from collections.abc import Iterator
from typing import Any

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.memory2.type.observation import Observation
from dimos.memory2.transform import Transformer
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger
from dimos.utils.data import get_data

from dimos.mapping.relocalize import relocalize as _relocalize

logger = setup_logger()

FRAME_MAP = "map"
FRAME_WORLD = "world"
FRAME_BODY = "base_link"

  
def relocalize(global_map: PointCloud2, local_map: PointCloud2) -> Transform:
    """Estimate the body→world transform mapping ``local_map`` into ``global_map``."""
    T = _relocalize(global_map.pointcloud, local_map.pointcloud)  # 4x4 np.ndarray

    return Transform(
        translation=Vector3(*T[:3, 3]),
        rotation=Quaternion.from_rotation_matrix(T[:3, :3]),
        frame_id=FRAME_MAP,
        child_frame_id=FRAME_BODY,
    )


# TODO wire the relocalize function inside the module

# TODO figure out how to do rerun intra-frame transform

# load all lidar frames captured in the readius around the semantic peaks
# feed them into a global mapper to get a single pointcloud around our areas of interest

# map_to_world_transform = lidar.transform(VoxelMapTransformer()).transform(
#     RelocalizationTransformer()
# )


# class RelocalizationTransformer(Transformer[PointCloud2, Transform]):
#     def __init__(self, loaded_map: PointCloud2):
#         self.loaded_map = loaded_map

#     def __call__(
#         self, upstream: Iterator[Observation[PointCloud2]]
#     ) -> Iterator[Observation[Transform]]:
#         for current_map in upstream:
#             transform = relocalize(current_map, self.loaded_map)
#             if transform:
#                 yield transform

class RelocalizationModule(Module):
    # global_map: In[PointCloud2]
    loaded_map: Out[PointCloud2]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        self._map_data: PointCloud2 | None = None
        self._running = False
        self._thread: threading.Thread | None = None
        self._last_published_time = 0.0

    @rpc
    def start(self):
        super().start()

        self._map_data = PointCloud2.lcm_decode(
            get_data("go2_hongkong_office_twopass_map.pc2.lcm").read_bytes()
        )
        self._map_data.frame_id = FRAME_MAP
        self._running = True
        # borrow the periodic publish code from dimos/mapping/pgo.py
        self._last_published_time = 0.0
        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()

        logger.info(
            "Relocalization module started"
        )

        self.tf.publish(Transform(
            translation=Vector3(0.0, 0.0, 10.), child_frame_id=FRAME_MAP, frame_id=FRAME_WORLD))

        # self.relocalization_transformer = RelocalizationTransformer(loaded_map)

        # for global_map in self.global_map:
        #     transform = self.relocalization_transformer(global_map)
        #     if transform:
        #         self.tf.publish(transform)

    @rpc
    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()

    def _publish_loop(self) -> None:
        rate = 0.5 # Hz
        interval = 1.0 / rate if rate > 0 else 2.0

        while self._running:
            t0 = time.monotonic()

            if t0 - self._last_published_time > interval:

                self.loaded_map.publish(self._map_data)

                self.tf.publish(Transform(
                    translation=Vector3(0.0, 0.0, 10.), child_frame_id=FRAME_MAP, frame_id=FRAME_WORLD))

                self._last_published_time = t0


# class GlobalLookupModule:
#     loaded_map: In[PointCloud2]

#     object_locations: {
#         "self_charging_dock": PoseStamped(frame_id="map", pose=Pose(10, 0, 0)),
#         "plant": PoseStamped(frame_id="map", pose=Pose(10, 10, 0)),
#     }

#     def start(self):
#         super().start()
#         self._map = None
#         self.loaded_map.subscribe(self._on_map)

#     def _on_map(self, msg: PointCloud2):
#         self._map = msg

#     # gives you relative pose of object in base_link frame, or None if not found
#     def lookup(self, query: str) -> Transform | None:
#         if not self._map:
#             # no relocalization until we have a map
#             return None

#         return Transform.from_pose(self.object_locations[query], frame_id="base_link")

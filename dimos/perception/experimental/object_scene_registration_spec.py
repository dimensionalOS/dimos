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

from typing import Protocol

import numpy as np
from numpy.typing import NDArray

from dimos.msgs.perception_msgs.RegisteredObject import RegisteredObject
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.spec.utils import Spec


class ObjectSceneRegistrationSpec(Spec, Protocol):
    def get_object_by_object_id(self, object_id: str) -> RegisteredObject | None: ...
    def get_registered_objects(self) -> list[RegisteredObject]: ...
    def set_prompts(
        self,
        text: list[str] | None = None,
        bboxes: NDArray[np.float64] | None = None,
    ) -> None: ...
    def get_object_pointcloud_by_name(self, name: str) -> PointCloud2 | None: ...
    def get_object_pointcloud_by_object_id(self, object_id: str) -> PointCloud2 | None: ...
    def get_full_scene_pointcloud(
        self,
        exclude_object_id: str | None = None,
        depth_trunc: float = 2.0,
        voxel_size: float = 0.01,
    ) -> PointCloud2 | None: ...

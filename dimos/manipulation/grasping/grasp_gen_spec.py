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

from dimos_generated.dimos_msgs.msg import GraspCandidateArray
from dimos_generated.geometry_msgs.msg import PoseArray
from dimos_generated.sensor_msgs.msg import PointCloud2

from dimos.spec.utils import Spec


class LegacyGraspGenSpec(Spec, Protocol):
    def generate_grasps(
        self,
        pointcloud: PointCloud2,
        scene_pointcloud: PointCloud2 | None = None,
    ) -> PoseArray | None: ...


class GraspGenSpec(Spec, Protocol):
    def propose_grasps(self, object_pointcloud: PointCloud2) -> GraspCandidateArray: ...

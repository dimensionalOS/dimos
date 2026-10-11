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

from dataclasses import dataclass
from typing import Protocol

from dimos.msgs.geometry_msgs.PoseArray import PoseArray
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.spec.utils import Spec


class LegacyGraspGenSpec(Spec, Protocol):
    def generate_grasps(
        self,
        pointcloud: PointCloud2,
        scene_pointcloud: PointCloud2 | None = None,
    ) -> PoseArray | None: ...


class GraspGenSpec(Spec, Protocol):
    def propose_grasps(self, object_pointcloud: PointCloud2) -> GraspCandidateArray: ...


@dataclass(frozen=True)
class SceneGrasps:
    """Grasps for an object, minus those whose gripper would overlap the scene around it.

    Args:
        grasps: The grasps that stay clear of the scene, best first.
        removed: How many proposed grasps were dropped for overlapping the scene.
    """

    grasps: GraspCandidateArray
    removed: int


class SceneGraspGenSpec(Spec, Protocol):
    """A grasp provider that can also drop grasps whose gripper would hit the scene."""

    def propose_grasps_in_scene(
        self,
        object_pointcloud: PointCloud2,
        scene_pointcloud: PointCloud2,
        collision_threshold: float,
    ) -> SceneGrasps: ...

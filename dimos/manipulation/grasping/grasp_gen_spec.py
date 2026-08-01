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

from dimos.msgs.geometry_msgs.PoseArray import PoseArray
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.grasping_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.reconstruction_msgs.TSDFGrid import TSDFGrid
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.spec.utils import Spec


class GraspGenSpec(Spec, Protocol):
    def generate_grasps(
        self,
        pointcloud: PointCloud2,
        scene_pointcloud: PointCloud2 | None = None,
    ) -> PoseArray | None: ...


class TSDFGraspGenSpec(Spec, Protocol):
    def generate_grasps_from_tsdf(self, tsdf: TSDFGrid) -> GraspCandidateArray | None: ...
    def generate_grasps_for_target_bounds(
        self,
        target_center: Vector3,
        target_size: Vector3,
        target_frame_id: str,
        target_ts: float,
        cushion_m: float = 0.03,
    ) -> GraspCandidateArray | None: ...

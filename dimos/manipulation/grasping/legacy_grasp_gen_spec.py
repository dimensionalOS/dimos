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

"""Legacy grasp-generation protocol retained for the registry-visible module.

``GraspingModule`` still exposes the pre-existing ``generate_grasps`` API,
while ``GraspGenSpec`` defines the newer candidate-based API.  Keep this
boundary until the registry-visible module is migrated.
"""

from typing import Protocol

from dimos.msgs.geometry_msgs.PoseArray import PoseArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.spec.utils import Spec


class LegacyGraspGenSpec(Spec, Protocol):
    """Compatibility protocol for the pre-existing pose-array API."""

    def generate_grasps(
        self,
        pointcloud: PointCloud2,
        scene_pointcloud: PointCloud2 | None = None,
    ) -> PoseArray | None: ...

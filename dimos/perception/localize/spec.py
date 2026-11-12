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

"""What pick and place and the grasp generators read from localize.

A spec rather than the module class: the GraspGenX runtime imports the grasp
generators in its own environment, which does not carry localize's dependencies.
"""

from typing import Protocol

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.spec.utils import Spec


class LocalizeSpec(Spec, Protocol):
    def get_pointcloud(self, target: str) -> PointCloud2: ...

    def scene_cloud(
        self, exclude: str | None = None, start: float = -5.0, duration: float = 5.0
    ) -> PointCloud2 | None: ...

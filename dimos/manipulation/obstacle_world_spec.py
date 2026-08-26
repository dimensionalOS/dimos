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

"""Planner obstacle mutation protocol for scene-derived geometry."""

from typing import Literal, Protocol

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.spec.utils import Spec


class ObstacleWorldSpec(Spec, Protocol):
    def add_obstacle(
        self,
        name: str,
        pose: Pose,
        shape: Literal["box", "sphere", "cylinder", "mesh"],
        dimensions: list[float] | None = None,
        mesh_path: str | None = None,
    ) -> str: ...

    def update_obstacle(
        self,
        name: str,
        pose: Pose,
        shape: Literal["box", "sphere", "cylinder", "mesh"],
        dimensions: list[float] | None = None,
        mesh_path: str | None = None,
        color: list[float] | None = None,
    ) -> bool: ...

    def remove_obstacle(self, obstacle_id: str) -> bool: ...

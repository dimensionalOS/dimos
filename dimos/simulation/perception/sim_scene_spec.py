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

from dimos.simulation.engines.mujoco_engine import GeomInfo
from dimos.spec.utils import Spec


class SimSceneGeometrySpec(Spec, Protocol):
    """Ground-truth geometry a simulator can hand to privileged perception."""

    def list_body_names(self) -> list[str]: ...
    def get_body_poses(self, names: list[str]) -> dict[str, list[float]]: ...
    def get_body_geoms(self, name: str) -> list[GeomInfo]: ...
    def sample_body_surface(self, name: str, count: int = 512) -> list[list[float]]: ...
    def sample_scene_surface(
        self,
        exclude: list[str] | None = None,
        voxel_size: float = 0.01,
        count: int = 20000,
    ) -> list[list[float]]: ...

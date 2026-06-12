# Copyright 2025-2026 Dimensional Inc.
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

"""Configuration types for manipulation visualization backends."""

from __future__ import annotations

from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field

from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig

ManipulationVisualizationBackend = Literal["meshcat", "viser", "none"]


class NoManipulationVisualizationConfig(BaseModel):
    """Disable manipulation visualization."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    backend: Literal["none"] = "none"

    @property
    def requires_world_visualization(self) -> bool:
        """Whether the planning world must create an embedded visualizer."""
        return False


class MeshcatVisualizationConfig(BaseModel):
    """Use the embedded Meshcat visualizer provided by the planning world."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    backend: Literal["meshcat"] = "meshcat"

    @property
    def requires_world_visualization(self) -> bool:
        """Whether the planning world must create an embedded visualizer."""
        return True


ManipulationVisualizationConfig = Annotated[
    NoManipulationVisualizationConfig | MeshcatVisualizationConfig | ViserVisualizationConfig,
    Field(discriminator="backend"),
]

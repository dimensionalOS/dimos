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

"""Factory functions for manipulation visualization backends."""

from __future__ import annotations

from typing import TYPE_CHECKING

from dimos.manipulation.planning.spec.protocols import VisualizationSpec
from dimos.manipulation.visualization.config import (
    ManipulationVisualizationConfig,
    MeshcatVisualizationConfig,
    NoManipulationVisualizationConfig,
)
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig

if TYPE_CHECKING:
    from dimos.manipulation.manipulation_module import ManipulationModule
    from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor
    from dimos.manipulation.planning.spec.protocols import WorldSpec


def create_manipulation_visualization(
    config: ManipulationVisualizationConfig,
    *,
    world: WorldSpec,
    world_monitor: WorldMonitor,
    manipulation_module: ManipulationModule,
) -> VisualizationSpec | None:
    """Create an optional manipulation visualization backend."""
    if isinstance(config, NoManipulationVisualizationConfig):
        return None

    if isinstance(config, MeshcatVisualizationConfig):
        if isinstance(world, VisualizationSpec):
            return world
        raise ValueError("meshcat visualization requires a world that implements VisualizationSpec")

    if isinstance(config, ViserVisualizationConfig):
        from dimos.manipulation.visualization.viser.visualizer import (
            ViserManipulationVisualizer,
        )

        return ViserManipulationVisualizer(config=config)

    raise AssertionError(f"Unhandled manipulation visualization config: {config!r}")

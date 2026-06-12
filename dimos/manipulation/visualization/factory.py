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

from collections.abc import Mapping
from typing import TYPE_CHECKING, Literal, cast, get_args

from dimos.manipulation.planning.spec.protocols import VisualizationSpec

if TYPE_CHECKING:
    from dimos.manipulation.manipulation_module import ManipulationModule
    from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor

ManipulationVisualizationBackend = Literal["meshcat", "viser", "none"]


def resolve_visualization_backend(
    backend: str | None,
    *,
    enable_viz: bool,
) -> ManipulationVisualizationBackend:
    """Resolve manipulation visualization backend with enable_viz compatibility."""
    if backend is None:
        return "meshcat" if enable_viz else "none"

    if backend not in get_args(ManipulationVisualizationBackend):
        available = list(get_args(ManipulationVisualizationBackend))
        raise ValueError(
            f"Unknown manipulation visualization backend: {backend!r}. Available: {available}"
        )
    return cast("ManipulationVisualizationBackend", backend)


def create_manipulation_visualization(
    backend: ManipulationVisualizationBackend,
    *,
    world_monitor: WorldMonitor,
    manipulation_module: ManipulationModule | None = None,
    options: Mapping[str, object] | None = None,
) -> VisualizationSpec | None:
    """Create an optional manipulation visualization backend."""
    if backend == "none":
        return None

    if backend == "meshcat":
        visualization = world_monitor.visualization
        if visualization is not None:
            return visualization
        raise ValueError("meshcat visualization requires a world that implements VisualizationSpec")

    if backend == "viser":
        from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
        from dimos.manipulation.visualization.viser.visualizer import (
            ViserManipulationVisualizer,
        )

        return ViserManipulationVisualizer(
            world_monitor=world_monitor,
            manipulation_module=manipulation_module,
            config=ViserVisualizationConfig.from_options(options),
        )

    available = list(get_args(ManipulationVisualizationBackend))
    raise ValueError(
        f"Unknown manipulation visualization backend: {backend!r}. Available: {available}"
    )

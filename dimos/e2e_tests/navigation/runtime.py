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

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import os
from types import MappingProxyType
from typing import Any

from dimos.core.global_config import TransportBackend
from dimos.core.stream import Transport
from dimos.e2e_tests.navigation.probe import StreamProbe
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.simulation.scene_controls import NavigationSceneControl


@dataclass(frozen=True)
class NavigationProvider:
    """Process configuration for one navigation scene provider."""

    name: str
    simulator: str
    transport: TransportBackend
    global_args: tuple[str, ...]


PROVIDERS: Mapping[str, NavigationProvider] = MappingProxyType(
    {
        "dimsim": NavigationProvider(
            name="dimsim",
            simulator="dimsim",
            transport="lcm",
            global_args=("--transport", "lcm", "--viewer", "none", "--dimsim-scene", "empty"),
        ),
        "pimsim": NavigationProvider(
            name="pimsim",
            simulator="mujoco",
            transport="zenoh",
            global_args=(
                "--transport",
                "zenoh",
                "--viewer",
                "none",
                "--simulation-provider",
                "pimsim",
                "--scene-package",
                "none",
            ),
        ),
    }
)


def resolve_navigation_provider(
    environ: Mapping[str, str] | None = None,
) -> NavigationProvider:
    """Resolve the provider selected for this pytest process."""
    source = os.environ if environ is None else environ
    name = source.get("DIMOS_E2E_SIMULATOR", "dimsim")
    try:
        return PROVIDERS[name]
    except KeyError as error:
        available = ", ".join(sorted(PROVIDERS))
        raise ValueError(
            f"Unknown DIMOS_E2E_SIMULATOR={name!r}. Available providers: {available}."
        ) from error


@dataclass
class NavigationRun:
    """Live public streams plus the test-only scene-control boundary."""

    provider: NavigationProvider
    scene: NavigationSceneControl
    agent_idle: StreamProbe[bool]
    odom: StreamProbe[PoseStamped]
    global_costmap: StreamProbe[OccupancyGrid]
    human_input: Transport[Any]

    def send_instruction(self, text: str) -> None:
        self.human_input.broadcast(None, text)


__all__ = [
    "PROVIDERS",
    "NavigationProvider",
    "NavigationRun",
    "resolve_navigation_provider",
]

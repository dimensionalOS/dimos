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

from dataclasses import dataclass, field
import importlib.metadata as importlib_metadata
from pathlib import Path
from typing import Any, Protocol, runtime_checkable

from dimos.core.coordination.blueprints import Blueprint
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

ENTRY_POINT_GROUP = "dimos.simulation.providers"


@dataclass(frozen=True)
class SimulationRequest:
    robot_model: str
    model_path: str | Path | None = None
    mesh_dir: str | Path | None = None
    scene_package: str | Path | None = None


@dataclass(frozen=True)
class SimulationBinding:
    backend: Blueprint
    adapter_type: str
    adapter_address: str | Path
    rerun_config: dict[str, Any] = field(default_factory=dict)
    robot_base_pose: PoseStamped = field(default_factory=PoseStamped)


@runtime_checkable
class SimulationProvider(Protocol):
    def build(self, request: SimulationRequest) -> SimulationBinding: ...


def load_simulation_provider(name: str) -> SimulationProvider:
    matches = list(importlib_metadata.entry_points(group=ENTRY_POINT_GROUP, name=name))
    if not matches:
        available = sorted(
            entry_point.name
            for entry_point in importlib_metadata.entry_points(group=ENTRY_POINT_GROUP)
        )
        suffix = f" Available providers: {', '.join(available)}." if available else ""
        raise ValueError(f"Simulation provider {name!r} is not installed.{suffix}")
    if len(matches) > 1:
        raise ValueError(f"Simulation provider {name!r} is registered more than once")
    provider = matches[0].load()
    if not isinstance(provider, SimulationProvider):
        raise TypeError(
            f"Simulation provider {name!r} must implement SimulationProvider, got {provider!r}"
        )
    return provider

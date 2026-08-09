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

from collections.abc import Sequence
from dataclasses import dataclass, field
from enum import StrEnum
import importlib.metadata as importlib_metadata
from pathlib import Path
from typing import Any, Protocol, runtime_checkable

from dimos.control.components import HardwareComponent
from dimos.core.coordination.blueprints import Blueprint
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

ENTRY_POINT_GROUP = "dimos.simulation.providers"


class SimulationFeature(StrEnum):
    """Optional simulator services requested by an ordinary blueprint."""

    EPISODE_CONTROL = "episode_control"
    MANIPULATION_SCENE = "manipulation_scene"
    SENSORS = "sensors"


@dataclass(frozen=True)
class SimulationRequest:
    robot_model: str
    model_path: str | Path | None = None
    mesh_dir: str | Path | None = None
    scene_package: str | Path | None = None
    features: frozenset[SimulationFeature] = frozenset()

    def __post_init__(self) -> None:
        features = frozenset(SimulationFeature(feature) for feature in self.features)
        object.__setattr__(self, "features", features)


@dataclass(frozen=True)
class SimulationBinding:
    backend: Blueprint
    hardware: tuple[HardwareComponent, ...] = ()
    rerun_config: dict[str, Any] = field(default_factory=dict)
    robot_base_pose: PoseStamped = field(default_factory=PoseStamped)

    def __post_init__(self) -> None:
        hardware_ids = tuple(component.hardware_id for component in self.hardware)
        if len(hardware_ids) != len(set(hardware_ids)):
            raise ValueError("simulation hardware IDs must be unique")


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


def resolve_robot(
    *,
    real_hardware: Sequence[HardwareComponent],
    simulation: SimulationRequest,
    default_backend: Blueprint | None = None,
) -> SimulationBinding:
    """Select ordinary hardware or a configured external simulation provider."""
    from dimos.core.global_config import global_config

    provider_name = global_config.simulation_provider
    if not provider_name:
        return SimulationBinding(
            backend=default_backend or Blueprint(()),
            hardware=tuple(real_hardware),
        )

    binding = load_simulation_provider(provider_name).build(simulation)
    if not binding.hardware:
        raise ValueError(
            f"simulation provider {provider_name!r} returned no hardware components "
            f"for {simulation.robot_model!r}"
        )
    return binding


__all__ = [
    "ENTRY_POINT_GROUP",
    "SimulationBinding",
    "SimulationFeature",
    "SimulationProvider",
    "SimulationRequest",
    "load_simulation_provider",
    "resolve_robot",
]

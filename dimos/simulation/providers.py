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

from dataclasses import dataclass
import importlib.metadata as importlib_metadata
from pathlib import Path
from typing import Protocol, runtime_checkable

from dimos.core.coordination.blueprints import Blueprint

ENTRY_POINT_GROUP = "dimos.simulation.providers"


@dataclass(frozen=True)
class SimulationRequest:
    robot_model: str
    model_path: str | Path
    mesh_dir: str | Path
    scene_package: str | Path | None


@dataclass(frozen=True)
class SimulationDeviceEndpoint:
    device_id: str
    protocol: str
    host_address: str
    device_address: str

    def __post_init__(self) -> None:
        if not all(
            value.strip()
            for value in (
                self.device_id,
                self.protocol,
                self.host_address,
                self.device_address,
            )
        ):
            raise ValueError("simulation device endpoint fields must not be empty")


@dataclass(frozen=True)
class SimulationBinding:
    backend: Blueprint
    adapter_type: str
    adapter_address: str | Path
    devices: tuple[SimulationDeviceEndpoint, ...] = ()

    def require_device(self, device_id: str) -> SimulationDeviceEndpoint:
        matches = tuple(device for device in self.devices if device.device_id == device_id)
        if len(matches) != 1:
            raise ValueError(
                f"simulation binding requires exactly one {device_id!r} device, "
                f"found {len(matches)}"
            )
        return matches[0]


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

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
import math
from typing import Any, Protocol, runtime_checkable

ENTRY_POINT_GROUP = "dimos.simulation.scene_controls"


@dataclass(frozen=True)
class PlanarBounds:
    """Axis-aligned bounds in the canonical DimOS world XY plane."""

    min_x: float
    min_y: float
    max_x: float
    max_y: float

    def __post_init__(self) -> None:
        values = (self.min_x, self.min_y, self.max_x, self.max_y)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("planar bounds must be finite")
        if self.min_x > self.max_x or self.min_y > self.max_y:
            raise ValueError("planar bounds minimum must not exceed maximum")

    def distance_to(self, x: float, y: float) -> float:
        """Return planar distance from a point to the filled bounds."""
        dx = max(self.min_x - x, 0.0, x - self.max_x)
        dy = max(self.min_y - y, 0.0, y - self.max_y)
        return math.hypot(dx, dy)


@runtime_checkable
class SceneControl(Protocol):
    """Lifecycle shared by simulator-control adapters."""

    provider_name: str

    def start(self) -> None: ...

    def stop(self) -> None: ...


@runtime_checkable
class NavigationSceneControl(SceneControl, Protocol):
    """Private world controls required by navigation acceptance tests."""

    def set_agent_position(
        self,
        x: float,
        y: float,
        z: float | None = None,
    ) -> None: ...

    def add_wall(self, x1: float, y1: float, x2: float, y2: float) -> None: ...

    def publish_goal(self, x: float, y: float) -> None: ...

    def semantic_object_bounds(self, query: str) -> PlanarBounds: ...


@runtime_checkable
class AgentPoseSceneControl(NavigationSceneControl, Protocol):
    """Optional navigation control that can set position and heading."""

    def set_agent_pose(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
    ) -> None: ...


def load_scene_control(name: str) -> SceneControl:
    """Load one built-in or installed simulator-control adapter."""
    if name == "dimsim":
        from dimos.simulation.dimsim.control import DimSimSceneControl

        return DimSimSceneControl()

    matches = list(importlib_metadata.entry_points(group=ENTRY_POINT_GROUP, name=name))
    if not matches:
        available = sorted(
            {"dimsim"}
            | {
                entry_point.name
                for entry_point in importlib_metadata.entry_points(group=ENTRY_POINT_GROUP)
            }
        )
        raise ValueError(
            f"Scene-control provider {name!r} is not installed. "
            f"Available providers: {', '.join(available)}."
        )
    if len(matches) > 1:
        raise ValueError(f"Scene-control provider {name!r} is registered more than once")
    factory: Any = matches[0].load()
    if not callable(factory):
        raise TypeError(f"Scene-control provider {name!r} entry point must be callable")
    control = factory()
    if not isinstance(control, SceneControl):
        raise TypeError(
            f"Scene-control provider {name!r} must implement SceneControl, got {control!r}"
        )
    if control.provider_name != name:
        raise ValueError(
            f"Scene-control entry point {name!r} returned provider {control.provider_name!r}"
        )
    return control


__all__ = [
    "ENTRY_POINT_GROUP",
    "AgentPoseSceneControl",
    "NavigationSceneControl",
    "PlanarBounds",
    "SceneControl",
    "load_scene_control",
]

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
import math

from dimos.simulation.scene_controls import PlanarBounds

Point2 = tuple[float, float]
Wall = tuple[float, float, float, float]


@dataclass(frozen=True)
class DynamicReplanningScenario:
    """Provider-neutral world and evidence contract for dynamic replanning."""

    scenario_id: str
    start: Point2
    static_walls: tuple[Wall, ...]
    exploration_route: tuple[Point2, ...]
    trigger_point: Point2
    trigger_radius_m: float
    inserted_wall: Wall
    alternate_route_bounds: PlanarBounds
    goal: Point2
    goal_tolerance_m: float
    timeout_s: float

    def __post_init__(self) -> None:
        if not self.scenario_id.strip():
            raise ValueError("scenario ID must not be empty")
        positive = (self.trigger_radius_m, self.goal_tolerance_m, self.timeout_s)
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("scenario distances and timeout must be finite and positive")


DYNAMIC_CORRIDOR = DynamicReplanningScenario(
    scenario_id="two-door-dynamic-corridor",
    start=(3.0, 2.0),
    static_walls=(
        (2.0, -4.0, 12.0, -4.0),
        (2.0, 4.0, 12.0, 4.0),
        (2.0, -4.0, 2.0, 4.0),
        (12.0, -4.0, 12.0, 4.0),
        (7.0, -4.0, 7.0, -2.0),
        (7.0, 0.0, 7.0, 1.0),
        (7.0, 3.0, 7.0, 4.0),
    ),
    exploration_route=((10.0, 2.0), (10.0, -1.0), (3.0, -1.0), (3.0, 2.0)),
    trigger_point=(7.0, 2.0),
    trigger_radius_m=2.5,
    inserted_wall=(7.0, 1.0, 7.0, 3.0),
    alternate_route_bounds=PlanarBounds(6.0, -2.0, 8.0, 0.0),
    goal=(10.5, 1.0),
    goal_tolerance_m=1.0,
    timeout_s=150.0,
)


__all__ = ["DYNAMIC_CORRIDOR", "DynamicReplanningScenario", "Point2", "Wall"]

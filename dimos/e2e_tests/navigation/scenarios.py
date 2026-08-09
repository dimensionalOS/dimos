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


@dataclass(frozen=True)
class SemanticNavigationScenario:
    """Provider-neutral command and private scoring contract for one target."""

    scenario_id: str
    command: str
    memory_query: str
    evaluator_query: str
    task_start: tuple[float, float, float]
    max_target_distance_m: float
    agent_response_timeout_s: float = 150.0
    navigation_timeout_s: float = 180.0

    def __post_init__(self) -> None:
        if not self.scenario_id.strip():
            raise ValueError("scenario ID must not be empty")
        if (
            not self.command.strip()
            or not self.memory_query.strip()
            or not self.evaluator_query.strip()
        ):
            raise ValueError("semantic navigation text must not be empty")
        positive = (
            self.max_target_distance_m,
            self.agent_response_timeout_s,
            self.navigation_timeout_s,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("semantic navigation limits must be finite and positive")
        if not all(math.isfinite(value) for value in self.task_start):
            raise ValueError("semantic navigation task start must be finite")


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

APARTMENT_EXPLORATION_ROUTE: tuple[Point2, ...] = (
    (3.881, 4.803),
    (3.200, 4.500),
    (4.000, 4.500),
    (3.400, 4.500),
    (3.881, 4.803),
    (4.160, 1.615),
    (1.596, 1.505),
    (1.596, 2.400),
    (1.596, 1.505),
    (1.649, 0.137),
    (-3.644, -0.064),
    (-3.759, -2.661),
    (-4.186, -4.830),
    (-3.759, -2.661),
    (-1.070, -3.285),
    (-2.504, -2.452),
    (-2.647, 5.243),
    (-3.663, 3.591),
    (-1.178, 1.974),
    (-2.416, 2.629),
    (-2.581, 0.164),
    (1.834, 0.072),
    (3.010, -3.883),
    (1.756, -3.742),
    (6.336, -4.077),
    (8.264, -5.119),
    (6.258, -0.964),
    (6.453, 5.327),
)
APARTMENT_EXPLORATION_ORIGIN: Point2 = (3.0, 2.0)

APARTMENT_SEMANTIC_SCENARIOS = (
    SemanticNavigationScenario(
        scenario_id="bed",
        command="Go to the bed.",
        memory_query="bed",
        evaluator_query="queen size bed",
        task_start=(-2.504, -2.452, 0.52),
        max_target_distance_m=2.0,
    ),
    SemanticNavigationScenario(
        scenario_id="couch",
        command="Go to the couch.",
        memory_query="couch",
        evaluator_query="sectional",
        task_start=(3.200, 4.500, 0.52),
        max_target_distance_m=2.0,
    ),
    SemanticNavigationScenario(
        scenario_id="kitchen",
        command="Go to the kitchen.",
        memory_query="kitchen",
        evaluator_query="refrigerator",
        task_start=(1.756, -3.742, 0.52),
        max_target_distance_m=3.0,
    ),
    SemanticNavigationScenario(
        scenario_id="television",
        command="Go to the television.",
        memory_query="television",
        evaluator_query="television",
        task_start=(3.200, 4.500, 0.52),
        max_target_distance_m=2.0,
    ),
)


__all__ = [
    "APARTMENT_EXPLORATION_ORIGIN",
    "APARTMENT_EXPLORATION_ROUTE",
    "APARTMENT_SEMANTIC_SCENARIOS",
    "DYNAMIC_CORRIDOR",
    "DynamicReplanningScenario",
    "Point2",
    "SemanticNavigationScenario",
    "Wall",
]

# Copyright 2026 Dimensional Inc.
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

from functools import partial

from dimos.evals.scorers import exact, first_number, numeric, rank_order, ranking
from dimos.evals.suites.lib.habitat_qa import (
    HSSD_DATASET,
    INSTRUCTION,
    environment,
    parsed,
)
from dimos.evals.types import EvalCase, Suite

SCENE_KEY = "hssd_108736851_177263586"
SCENE_NAME = "HSSD scene 9"

_environment = partial(environment, "108736851_177263586", "HSSD_DATASET_CONFIG", HSSD_DATASET)
SUITE: Suite = [
    EvalCase(
        id=f"{SCENE_KEY}_dining_chairs",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many chairs are around the dining table? Return only the count.",
        grade=parsed(first_number, lambda v: exact(8, v)),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_curved_sofa_table_shape",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What shape is the tabletop between the two quarter-circle sofas? A) Circular; B) Square; C) Rectangular; D) Triangular. Return only A, B, C, or D.",
        grade=lambda o: exact("A", o.trajectory.final_answer.strip().upper()),
        tags=frozenset({"visual-attribute", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_side_table_sides",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many sides do the tabletops beside the blue sofa in the living room have? A) 4; B) 5; C) 6; D) 8. Return only A, B, C, or D.",
        grade=lambda o: exact("C", o.trajectory.final_answer.strip().upper()),
        tags=frozenset({"visual-attribute", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_bedrooms",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "How many bedrooms are in the home? Return only the count.",
        grade=parsed(first_number, lambda v: exact(4, v)),
        tags=frozenset({"rooms", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_kitchens",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many kitchen areas are in the home? Return only the count.",
        grade=parsed(first_number, lambda v: exact(2, v)),
        tags=frozenset({"rooms", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_tv_location",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Which room contains the television? A) Living room; B) Office; C) Bedroom; D) Kitchen. Return only A, B, C, or D.",
        grade=lambda o: exact("B", o.trajectory.final_answer.strip().upper()),
        tags=frozenset({"object-location", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_living_area",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate living-room floor area, in square meters? Return only the number.",
        grade=parsed(first_number, lambda v: numeric(123.96, v, tolerance=6, band=25)),
        tags=frozenset({"area", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_largest_bedroom",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate area of the largest bedroom, in square meters? Return only the number.",
        grade=parsed(first_number, lambda v: numeric(37.23, v, tolerance=2.5, band=8)),
        tags=frozenset({"area", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_area_order",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Order these rooms from smallest to largest area. A) Office; B) Living room; C) Dining room. Return all letters once in order, optionally separated by commas.",
        grade=parsed(ranking, lambda v: rank_order("CAB", v)),
        tags=frozenset({"area", "ranking"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_beds",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "How many beds are in the home? Return only the count.",
        grade=parsed(first_number, lambda v: exact(4, v)),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_office_path_order",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Rank these rooms by shortest walking distance to enter them from the office doorway facing the hallway, nearest first, for a robot of radius 0.25 m. A) Larger kitchen; B) Dining room; C) Laundry room. Return all letters once in order, optionally separated by commas.",
        # Source Habitat (10.973,.177897,-.232), static navmesh .25/.60 m.
        # Nearest sampled points inside region polygons: laundry 6.233,
        # dining 21.470, larger kitchen 21.595 m. The last two nearly tie;
        # this is a room-entry convention, not a center-distance ranking.
        grade=parsed(ranking, lambda v: rank_order("CBA", v)),
        tags=frozenset({"distance", "ranking"}),
    ),
]

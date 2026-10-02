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

from dimos.evals.scorers import exact, first_number, numeric, rank_order, ranking, yes_no
from dimos.evals.suites.lib.habitat_qa import (
    HSSD_DATASET,
    INSTRUCTION,
    environment,
    parsed,
)
from dimos.evals.types import EvalCase, Suite

SCENE_KEY = "hssd_106878858_174886965"
SCENE_NAME = "HSSD scene 7"

_environment = partial(environment, "106878858_174886965", "HSSD_DATASET_CONFIG", HSSD_DATASET)
SUITE: Suite = [
    EvalCase(
        id=f"{SCENE_KEY}_bathroom_floor_pattern_match",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Do all the bathrooms have the same floor pattern? Return only yes or no.",
        grade=parsed(yes_no, lambda v: exact("yes", v)),
        tags=frozenset({"visual-attribute", "boolean"}),
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
        id=f"{SCENE_KEY}_bathrooms",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "How many bathrooms are in the home? Return only the count.",
        grade=parsed(first_number, lambda v: exact(2, v)),
        tags=frozenset({"rooms", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_largest_bedroom_area",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate area of the largest bedroom, in square meters? Return only the number.",
        grade=parsed(first_number, lambda v: numeric(46.72, v, tolerance=3, band=10)),
        tags=frozenset({"area", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_dining_perimeter",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate dining-room perimeter, in meters? Return only the number.",
        grade=parsed(first_number, lambda v: numeric(22.54, v, tolerance=1, band=4)),
        tags=frozenset({"perimeter", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_laptop_location",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Which room contains the laptop? A) Bedroom; B) Office; C) Living room; D) Kitchen. Return only A, B, C, or D.",
        grade=lambda o: exact("B", o.trajectory.final_answer.strip().upper()),
        tags=frozenset({"object-location", "single-choice"}),
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
        id=f"{SCENE_KEY}_garage_bedroom_doorways",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the minimum number of doorways from the garage to the largest bedroom? Return only the count.",
        grade=parsed(first_number, lambda v: exact(4, v)),
        tags=frozenset({"connectivity", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_entryway_path_order",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Order these objects from nearest to farthest by collision-free travel distance from the entrance hall opening into the living room, for a robot of radius 0.25 m. A) Office laptop; B) Kitchen refrigerator; C) Garage mower. Return all letters once in order, optionally separated by commas.",
        # Source Habitat (-9.217360,.158400,-4.237486), static navmesh .25/.60 m.
        # .15 m goal grid within 1.5 m of anchors: laptop 3.254, fridge 8.632,
        # mower 11.717 m; the other tested entryway openings preserve this order.
        grade=parsed(ranking, lambda v: rank_order("ABC", v)),
        tags=frozenset({"distance", "ranking"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_garage_car_color",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What color is the car in the garage? A) Blue; B) Red; C) White; D) Black. Return only A, B, C, or D.",
        grade=lambda o: exact("B", o.trajectory.final_answer.strip().upper()),
        tags=frozenset({"visual-attribute", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_exterior_opening",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Is there an open doorway or passage from the home to the outside? Return only yes or no.",
        grade=parsed(yes_no, lambda v: exact("yes", v)),
        tags=frozenset({"object-state", "boolean"}),
    ),
]

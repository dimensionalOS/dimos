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

SCENE_KEY = "hssd_108736884_177263634"
SCENE_NAME = "HSSD scene 10"

_environment = partial(environment, "108736884_177263634", "HSSD_DATASET_CONFIG", HSSD_DATASET)
SUITE: Suite = [
    EvalCase(
        id=f"{SCENE_KEY}_bathroom_plant_exists",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "Is there a plant in any bathroom? Return only yes or no.",
        grade=parsed(yes_no, lambda v: exact("yes", v)),
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_bedrooms",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "How many bedrooms are in the home? Return only the count.",
        grade=parsed(first_number, lambda v: exact(3, v)),
        tags=frozenset({"rooms", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_toilets",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "How many toilets are in the home? Return only the count.",
        grade=parsed(first_number, lambda v: exact(3, v)),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_red_potted_plant_location",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Which room contains the red potted plant? A) Office; B) Bedroom; C) Kitchen; D) Living room. Return only A, B, C, or D.",
        grade=lambda o: exact("D", o.trajectory.final_answer.strip().upper()),
        tags=frozenset({"object-location", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_kitchen_area",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate kitchen floor area, in square meters? Return only the number.",
        grade=parsed(first_number, lambda v: numeric(41.40, v, tolerance=2.5, band=9)),
        tags=frozenset({"area", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_kitchen_counter_windows",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many windows are along the kitchen counter? Return only the count.",
        grade=parsed(first_number, lambda v: exact(3, v)),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_bathtub_shape_match",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Are the bathtubs in the home the same shape? Return only yes or no.",
        grade=parsed(yes_no, lambda v: exact("no", v)),
        tags=frozenset({"visual-attribute", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_fridge_height",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate refrigerator height, in meters? Return only the number.",
        grade=parsed(first_number, lambda v: numeric(1.77, v, tolerance=0.1, band=0.4)),
        tags=frozenset({"dimensions", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_area_order",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Order these rooms from smallest to largest floor area. A) Dining room; B) Kitchen; C) Office. Return all letters once in order, optionally separated by commas.",
        grade=parsed(ranking, lambda v: rank_order("CAB", v)),
        tags=frozenset({"area", "ranking"}),
    ),
]

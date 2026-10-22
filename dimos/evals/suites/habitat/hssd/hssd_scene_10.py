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

"""HSSD 108736884_177263634: synthetic home, 1 floor, ~310 m² (~3,350 sqft) indoor.
3-bed: office, dining, kitchen, living, laundry, entryway, 2 baths plus separate toilet."""

from collections.abc import Callable
import os
from typing import TypeVar

from dimos.evals.environments.habitat import HabitatEnvironment
from dimos.evals.scorers import choice, exact, first_number, numeric, rank_order, ranking, yes_no
from dimos.evals.types import EvalCase, Outcome, Suite
from dimos.utils.data import get_data_dir

T = TypeVar("T")

SCENE_KEY = "hssd_108736884_177263634"
SCENE_NAME = "HSSD scene 10"

INSTRUCTION = (
    "You are answering questions about a live simulated home. You control the robot, "
    "and its sensor recording grows as it observes the environment. Initial observations "
    "do not cover the whole home. Move around to gather the evidence needed to answer "
    "the question. Inspect relevant interior rooms for counts and absence claims. "
    "Indoor areas, including an attached garage, are in scope. Exterior openings may "
    "be observed from indoors; do not leave the home. Use observations rather than "
    "assumptions about a typical home. Return the answer in the requested format."
)


def _parsed(parser: Callable[[str], T], score: Callable[[T], float]) -> Callable[[Outcome], float]:
    """Keep answer parsing separate from scoring; unparseable answers earn zero."""

    def grade(o: Outcome) -> float:
        try:
            value = parser(o.trajectory.final_answer)
        except ValueError:
            return 0.0
        return score(value)

    return grade


_LETTER = choice("ABCD", case_sensitive=True)


def _environment() -> HabitatEnvironment:
    return HabitatEnvironment(
        scene_dataset_config=os.environ.get(
            "HSSD_DATASET_CONFIG",
            str(get_data_dir("hssd-hab/hssd-hab.scene_dataset_config.json")),
        ),
        scene_id="108736884_177263634",
        seed=0,
        blueprint=["habitat-nav", "mcp-server", "observe-skill", "point-nav-skill-container"],
    )


SUITE: Suite = [
    EvalCase(
        id=f"{SCENE_KEY}_bathroom_plant_exists",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "Is there a plant in any bathroom? Return only yes or no.",
        grade=_parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_bedrooms",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "How many bedrooms are in the home? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(3, value)),
        tags=frozenset({"rooms", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_toilets",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "How many toilets are in the home? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(3, value)),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_red_potted_plant_location",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Which room contains the red potted plant? A) Office; B) Bedroom; C) Kitchen; D) Living room. Return only A, B, C, or D.",
        grade=_parsed(_LETTER, lambda value: exact("D", value)),
        tags=frozenset({"object-location", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_kitchen_area",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate kitchen floor area, in square meters? Return only the number.",
        grade=_parsed(first_number, lambda value: numeric(41.40, value, tolerance=2.5, band=9)),
        tags=frozenset({"area", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_kitchen_counter_windows",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many windows are along the kitchen counter? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(3, value)),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_bathtub_shape_match",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Are the bathtubs in the home the same shape? Return only yes or no.",
        grade=_parsed(yes_no, lambda value: exact("no", value)),
        tags=frozenset({"visual-attribute", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_fridge_height",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate refrigerator height, in meters? Return only the number.",
        grade=_parsed(first_number, lambda value: numeric(1.77, value, tolerance=0.1, band=0.4)),
        tags=frozenset({"dimensions", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_area_order",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Order these rooms from smallest to largest floor area. A) Dining room; B) Kitchen; C) Office. Return all letters once in order, optionally separated by commas.",
        grade=_parsed(ranking, lambda value: rank_order("CAB", value)),
        tags=frozenset({"area", "ranking"}),
    ),
]

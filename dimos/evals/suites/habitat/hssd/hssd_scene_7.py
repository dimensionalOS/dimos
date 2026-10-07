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

"""HSSD 106878858_174886965: synthetic home, 1 floor, ~295 m² (~3,200 sqft) indoor.
4-bed: 2 baths, office, living/kitchen/dining, utility, laundry, entryway, garage with red car."""

from collections.abc import Callable
import os
from typing import TypeVar

from dimos.evals.environments.habitat import HabitatEnvironment
from dimos.evals.scorers import choice, exact, first_number, numeric, rank_order, ranking, yes_no
from dimos.evals.types import EvalCase, Outcome, Suite
from dimos.utils.data import get_data_dir

T = TypeVar("T")

SCENE_KEY = "hssd_106878858_174886965"
SCENE_NAME = "HSSD scene 7"

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
        scene_id="106878858_174886965",
        seed=0,
        blueprint=["habitat-nav", "mcp-server", "observe-skill", "point-nav-skill-container"],
    )


SUITE: Suite = [
    EvalCase(
        id=f"{SCENE_KEY}_bathroom_floor_pattern_match",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Do all the bathrooms have the same floor pattern? Return only yes or no.",
        grade=_parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"visual-attribute", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_bedrooms",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "How many bedrooms are in the home? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(4, value)),
        tags=frozenset({"rooms", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_bathrooms",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "How many bathrooms are in the home? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(2, value)),
        tags=frozenset({"rooms", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_largest_bedroom_area",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate area of the largest bedroom, in square meters? Return only the number.",
        grade=_parsed(first_number, lambda value: numeric(46.72, value, tolerance=3, band=10)),
        tags=frozenset({"area", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_dining_perimeter",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate dining-room perimeter, in meters? Return only the number.",
        grade=_parsed(first_number, lambda value: numeric(22.54, value, tolerance=1, band=4)),
        tags=frozenset({"perimeter", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_laptop_location",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Which room contains the laptop? A) Bedroom; B) Office; C) Living room; D) Kitchen. Return only A, B, C, or D.",
        grade=_parsed(_LETTER, lambda value: exact("B", value)),
        tags=frozenset({"object-location", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_beds",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "How many beds are in the home? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(4, value)),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_garage_bedroom_doorways",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the minimum number of doorways from the garage to the largest bedroom? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(4, value)),
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
        grade=_parsed(ranking, lambda value: rank_order("ABC", value)),
        tags=frozenset({"distance", "ranking"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_garage_car_color",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What color is the car in the garage? A) Blue; B) Red; C) White; D) Black. Return only A, B, C, or D.",
        grade=_parsed(_LETTER, lambda value: exact("B", value)),
        tags=frozenset({"visual-attribute", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_exterior_opening",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Is there an open doorway or passage from the home to the outside? Return only yes or no.",
        grade=_parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"object-state", "boolean"}),
    ),
]

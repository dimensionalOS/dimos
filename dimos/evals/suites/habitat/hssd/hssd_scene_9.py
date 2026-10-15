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

"""HSSD 108736851_177263586: synthetic home, 1 floor, ~410 m² (~4,400 sqft) indoor.
4-bed, 3 baths, two kitchens, dining, office with TV, laundry, large living room."""

from collections.abc import Callable
import os
from typing import TypeVar

from dimos.evals.environments.habitat import HabitatEnvironment
from dimos.evals.scorers import choice, exact, first_number, numeric, rank_order, ranking
from dimos.evals.types import EvalCase, Outcome, Suite
from dimos.utils.data import get_data_dir

T = TypeVar("T")

SCENE_KEY = "hssd_108736851_177263586"
SCENE_NAME = "HSSD scene 9"

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
        scene_id="108736851_177263586",
        seed=0,
        blueprint=["habitat-nav", "mcp-server", "observe-skill", "point-nav-skill-container"],
    )


SUITE: Suite = [
    EvalCase(
        id=f"{SCENE_KEY}_dining_chairs",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many chairs are around the dining table? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(8, value)),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_curved_sofa_table_shape",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What shape is the tabletop between the two quarter-circle sofas? A) Circular; B) Square; C) Rectangular; D) Triangular. Return only A, B, C, or D.",
        grade=_parsed(_LETTER, lambda value: exact("A", value)),
        tags=frozenset({"visual-attribute", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_side_table_sides",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many sides do the tabletops beside the blue sofa in the living room have? A) 4; B) 5; C) 6; D) 8. Return only A, B, C, or D.",
        grade=_parsed(_LETTER, lambda value: exact("C", value)),
        tags=frozenset({"visual-attribute", "single-choice"}),
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
        id=f"{SCENE_KEY}_kitchens",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many kitchen areas are in the home? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(2, value)),
        tags=frozenset({"rooms", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_tv_location",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Which room contains the television? A) Living room; B) Office; C) Bedroom; D) Kitchen. Return only A, B, C, or D.",
        grade=_parsed(_LETTER, lambda value: exact("B", value)),
        tags=frozenset({"object-location", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_living_area",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate living-room floor area, in square meters? Return only the number.",
        grade=_parsed(first_number, lambda value: numeric(123.96, value, tolerance=6, band=25)),
        tags=frozenset({"area", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_largest_bedroom",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate area of the largest bedroom, in square meters? Return only the number.",
        grade=_parsed(first_number, lambda value: numeric(37.23, value, tolerance=2.5, band=8)),
        tags=frozenset({"area", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_area_order",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Order these rooms from smallest to largest area. A) Office; B) Living room; C) Dining room. Return all letters once in order, optionally separated by commas.",
        grade=_parsed(ranking, lambda value: rank_order("CAB", value)),
        tags=frozenset({"area", "ranking"}),
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
        id=f"{SCENE_KEY}_office_nearest_room",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Which room has the shortest walking distance to enter from the office doorway facing the hallway, for a robot of radius 0.25 m? A) Larger kitchen; B) Dining room; C) Laundry room. Return only the letter.",
        # Source Habitat (10.973,.177897,-.232), static navmesh .25/.60 m.
        # Nearest sampled points inside region polygons: laundry 6.233,
        # dining 21.470, larger kitchen 21.595 m. Dining and kitchen nearly tie,
        # so only the nearest is asked.
        grade=_parsed(_LETTER, lambda value: exact("C", value)),
        tags=frozenset({"distance", "single-choice"}),
    ),
]

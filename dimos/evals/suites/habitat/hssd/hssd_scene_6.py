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

"""HSSD 106366410_174226806: synthetic home, 1 floor, ~190 m² (~2,050 sqft) indoor.
1-bed with sofa: piano living room, kitchen/dining, laundry, bath, combined gym/office."""

from collections.abc import Callable
import os
from typing import TypeVar

from dimos.evals.environments.habitat import HabitatEnvironment
from dimos.evals.scorers import choice, exact, first_number, numeric, rank_order, ranking, yes_no
from dimos.evals.types import EvalCase, Outcome, Suite
from dimos.utils.data import get_data_dir

T = TypeVar("T")

SCENE_KEY = "hssd_106366410_174226806"
SCENE_NAME = "HSSD scene 6"

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
        scene_id="106366410_174226806",
        seed=0,
        blueprint=["habitat-nav", "mcp-server", "observe-skill", "point-nav-skill-container"],
    )


SUITE: Suite = [
    EvalCase(
        id=f"{SCENE_KEY}_gym_exists",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Is there an exercise area in the home? Return only yes or no.",
        grade=_parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_red_trash_bin_location",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Which room contains the red trash bin? A) Kitchen; B) Combined gym/office; C) Bedroom; D) Living room. Return only A, B, C, or D.",
        grade=_parsed(_LETTER, lambda value: exact("B", value)),
        tags=frozenset({"object-location", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_piano_location",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Which room contains the grand piano? A) Dining room; B) Office; C) Living room; D) Gym. Return only A, B, C, or D.",
        grade=_parsed(_LETTER, lambda value: exact("C", value)),
        tags=frozenset({"object-location", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_refrigerators",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many refrigerator-freezer units are in the kitchen? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(2, value)),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_gym_area",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate floor area of the gym zone, in square meters? Return only the number.",
        grade=_parsed(first_number, lambda value: numeric(23.08, value, tolerance=1.5, band=6)),
        tags=frozenset({"area", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_bedrooms",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "How many bedrooms are in the home? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(1, value)),
        tags=frozenset({"rooms", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_bed_relative_to_sofa",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "For someone seated on the bedroom sofa facing forward, is the bed to their left or right? A) Left; B) Right. Return only A or B.",
        grade=_parsed(_LETTER, lambda value: exact("A", value)),
        tags=frozenset({"spatial-relation", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_refrigerator_height",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate height of a kitchen refrigerator, in meters? Return only the number.",
        grade=_parsed(first_number, lambda value: numeric(1.77, value, tolerance=0.1, band=0.4)),
        tags=frozenset({"dimensions", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_area_order",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Order these areas from smallest to largest floor area. A) Office zone; B) Bedroom; C) Kitchen. Return all letters once in order, optionally separated by commas.",
        grade=_parsed(ranking, lambda value: rank_order("CAB", value)),
        tags=frozenset({"area", "ranking"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_laundry_appliances",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many washing and drying machines are in the laundry room? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(3, value)),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_toilet_room_bathtub",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Does the room containing the toilet also contain a bathtub? Return only yes or no.",
        grade=_parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"spatial-relation", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_bedroom_farthest_object",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Which object is farthest by collision-free travel distance from the bedroom doorway facing the hallway, for a robot of radius 0.25 m? A) Grand piano; B) Dining table; C) Treadmill. Return only the letter.",
        # Source Habitat (2.494610,.150866,-1.863723), static navmesh .25/.60 m.
        # .15 m goal grid within 1.5 m of anchors: table 6.812, treadmill 7.173,
        # piano 16.730 m. Table/treadmill nearly tie, so only the farthest is asked.
        grade=_parsed(_LETTER, lambda value: exact("A", value)),
        tags=frozenset({"distance", "single-choice"}),
    ),
]

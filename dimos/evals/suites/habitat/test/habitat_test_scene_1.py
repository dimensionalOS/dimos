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

"""Habitat test apartment_1.glb: real-apartment scan, 1 floor, ~53 m² (~570 sqft) navigable.
Lounge with L-sofa and wall TV, dining room with sideboard and mirror, connecting corridor."""

from collections.abc import Callable
import os
from typing import TypeVar

from dimos.evals.environments.habitat import HabitatEnvironment
from dimos.evals.scorers import choice, exact, first_number, yes_no
from dimos.evals.types import EvalCase, Outcome, Suite
from dimos.utils.data import get_data_dir

T = TypeVar("T")

SCENE_KEY = "habitat_test_apartment_1"
SCENE_NAME = "Habitat test scene 1"

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
        scene_dataset_config=os.environ.get("HABITAT_TEST_DATASET_CONFIG", "default"),
        scene_id=os.environ.get(
            "HABITAT_TEST_SCENE",
            str(get_data_dir("habitat_test_scenes/apartment_1.glb")),
        ),
        seed=0,
        blueprint=["habitat-nav", "mcp-server", "observe-skill", "point-nav-skill-container"],
    )


SUITE: Suite = [
    EvalCase(
        id=f"{SCENE_KEY}_mirror_shape",
        inputs=INSTRUCTION
        + "\n\nWhat shape is the wall mirror above the dining-room sideboard? A) Rectangular; B) Circular; C) Triangular; D) Hexagonal. Return only the letter.",
        environment=_environment(),
        grade=_parsed(_LETTER, lambda value: exact("B", value)),
        timeout_s=1200,
        tags=frozenset({"visual-attribute", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_window_covering",
        inputs=INSTRUCTION
        + "\n\nWhat type of window covering is used in the living room? A) Horizontal blinds; B) Fabric curtains; C) Exterior shutters; D) No covering. Return only the letter.",
        environment=_environment(),
        grade=_parsed(_LETTER, lambda value: exact("A", value)),
        timeout_s=1200,
        tags=frozenset({"visual-attribute", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_dining_door_state",
        inputs=INSTRUCTION
        + "\n\nIs the dining-room door open or closed? A) Closed; B) Open. Return only the letter.",
        environment=_environment(),
        grade=_parsed(_LETTER, lambda value: exact("B", value)),
        timeout_s=1200,
        tags=frozenset({"object-state", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_tv_location",
        inputs=INSTRUCTION
        + "\n\nWhich room contains the wall-mounted television? A) Dining room; B) Bedroom; C) Living room; D) Bathroom. Return only the letter.",
        environment=_environment(),
        grade=_parsed(_LETTER, lambda value: exact("C", value)),
        timeout_s=1200,
        tags=frozenset({"object-location", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_chess_decoration_count",
        inputs=INSTRUCTION
        + "\n\nHow many oversized chess-piece decorations are on the console beneath the television? Return only the count.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: exact(2, value)),
        timeout_s=1200,
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_serving_stand_tiers",
        inputs=INSTRUCTION
        + "\n\nHow many tiers does the serving stand on the dining table have? Return only the count.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: exact(2, value)),
        timeout_s=1200,
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_coffee_table_between",
        inputs=INSTRUCTION
        + "\n\nIs there a coffee table between the sectional sofa and the television? Return only yes or no.",
        environment=_environment(),
        grade=_parsed(yes_no, lambda value: exact("yes", value)),
        timeout_s=1200,
        tags=frozenset({"spatial-relation", "boolean"}),
    ),
]

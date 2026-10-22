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

"""HM3D 00861-GLAQ4DNUx5U: annotated real-home scan, 2 levels, ~126 m² (~1,350 sqft) navigable.
Bedrooms (one with a mural), bathrooms, kitchen/living, utility with laundry; some scan holes."""

from collections.abc import Callable
import os
from typing import TypeVar

from dimos.evals.environments.habitat import HabitatEnvironment
from dimos.evals.scorers import choice, exact, first_number, yes_no
from dimos.evals.types import EvalCase, Outcome, Suite
from dimos.utils.data import get_data_dir

T = TypeVar("T")

SCENE_KEY = "hm3d_GLAQ4DNUx5U"
SCENE_NAME = "HM3D scene 2"

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
            "HM3D_ANNOTATED_DATASET_CONFIG",
            str(
                get_data_dir(
                    "hm3d-0.2/hm3d/example/hm3d_annotated_example_basis.scene_dataset_config.json"
                )
            ),
        ),
        scene_id="00861-GLAQ4DNUx5U",
        seed=0,
        blueprint=["habitat-nav", "mcp-server", "observe-skill", "point-nav-skill-container"],
    )


SUITE: Suite = [
    EvalCase(
        id=f"{SCENE_KEY}_mural_location",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Which room has a large colorful graffiti-style wall mural? A) Kitchen; B) Bathroom; C) Bedroom; D) Utility room. Return only the letter.",
        grade=_parsed(_LETTER, lambda value: exact("C", value)),
        tags=frozenset({"visual-attribute", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_every_desk_chair",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "Does every desk have a desk chair? Return only yes or no.",
        grade=_parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"spatial-relation", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_beds",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many beds are in the scanned home? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(4, value)),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_televisions",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many televisions are in the scanned home? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(3, value)),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_toilets",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many toilets are in the scanned home? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(4, value)),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_washing_machines",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many washing machines are in the utility room? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(2, value)),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_exercise_bike_exists",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Is there an exercise bike in the home? Return only yes or no.",
        grade=_parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_exercise_bike_location",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Which type of room contains the exercise bike? A) Kitchen; B) Bedroom; C) Bathroom; D) Garage. Return only the letter.",
        grade=_parsed(_LETTER, lambda value: exact("B", value)),
        tags=frozenset({"object-location", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_fridge_location",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Which type of room contains the refrigerator? A) Living room; B) Bedroom; C) Utility/laundry room; D) Bathroom. Return only the letter.",
        grade=_parsed(_LETTER, lambda value: exact("C", value)),
        tags=frozenset({"object-location", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_ironing_board_exists",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Is there an ironing board in the utility room? Return only yes or no.",
        grade=_parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_vacuum_cleaners",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many vacuum cleaners are in the scanned home? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(2, value)),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_bedroom_tvs",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many bedrooms contain a television? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(2, value)),
        tags=frozenset({"spatial-relation", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_every_bedroom_tv",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Does every bedroom have a television? Return only yes or no.",
        grade=_parsed(yes_no, lambda value: exact("no", value)),
        tags=frozenset({"spatial-relation", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_ovens",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "How many ovens are in the kitchen? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(2, value)),
        tags=frozenset({"object-count", "count"}),
    ),
]

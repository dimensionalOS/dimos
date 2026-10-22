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

"""ReplicaCAD v3_sc1_staging_00: FRL apartment shell, 1 floor, ~85 m² (~900 sqft).
Sparsely staged: sofa, TV stand, one bicycle, beanbags, chairs, plants.
Articulated furniture (fridge, counter, cupboards, door) does not load: physics is off."""

from collections.abc import Callable
import os
from typing import TypeVar

from dimos.evals.environments.habitat import HabitatEnvironment
from dimos.evals.scorers import choice, exact, first_number, numeric, rank_order, ranking, yes_no
from dimos.evals.types import EvalCase, Outcome, Suite
from dimos.utils.data import get_data_dir

T = TypeVar("T")

SCENE_KEY = "replicacad_v3_sc1_staging_00"
SCENE_NAME = "ReplicaCAD scene 3"

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
            "REPLICACAD_DATASET_CONFIG",
            str(get_data_dir("replica_cad_dataset/replicaCAD.scene_dataset_config.json")),
        ),
        scene_id="v3_sc1_staging_00",
        seed=0,
        blueprint=["habitat-nav", "mcp-server", "observe-skill", "point-nav-skill-container"],
    )


SUITE: Suite = [
    EvalCase(
        id=f"{SCENE_KEY}_bicycles",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "How many bicycles are in the scene? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(1, value)),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_beanbags",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many beanbag seats are in the scene? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(2, value)),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_chairs",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many chairs are in the scene, excluding beanbags and the sofa? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(2, value)),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_plants",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many indoor potted plants are in the scene? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(2, value)),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_sofa_exists",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "Is there a sofa in the scene? Return only yes or no.",
        grade=_parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_beanbag_exists",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Are there any beanbag seats in the scene? Return only yes or no.",
        grade=_parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_sofa_width",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate sofa width, in meters? Return only the number.",
        grade=_parsed(first_number, lambda value: numeric(2.14, value, tolerance=0.1, band=0.4)),
        tags=frozenset({"dimensions", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_bicycle_height",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate height of the bicycle, in meters? Return only the number.",
        grade=_parsed(first_number, lambda value: numeric(0.98, value, tolerance=0.06, band=0.25)),
        tags=frozenset({"dimensions", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_sofa_stand_distance",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate horizontal straight-line distance between the centers of the sofa and TV stand, in meters? Return only the number.",
        grade=_parsed(first_number, lambda value: numeric(4.77, value, tolerance=0.25, band=1)),
        tags=frozenset({"distance", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_sofa_bike_distance",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate horizontal straight-line distance between the centers of the sofa and bicycle, in meters? Return only the number.",
        grade=_parsed(first_number, lambda value: numeric(6.67, value, tolerance=0.3, band=1.2)),
        tags=frozenset({"distance", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_nearer_object",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Which object's center is closer to the sofa's center in a horizontal straight line? A) TV stand; B) Bicycle. Return only the letter.",
        grade=_parsed(_LETTER, lambda value: exact("A", value)),
        tags=frozenset({"distance", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_height_order",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Order these objects from shortest to tallest. A) TV stand; B) Bicycle; C) Sofa. Return all three letters once in order, optionally separated by commas.",
        grade=_parsed(ranking, lambda value: rank_order("ACB", value)),
        tags=frozenset({"dimensions", "ranking"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_books_exists",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "Are there any books in the scene? Return only yes or no.",
        grade=_parsed(yes_no, lambda value: exact("no", value)),
        tags=frozenset({"existence", "boolean"}),
    ),
]

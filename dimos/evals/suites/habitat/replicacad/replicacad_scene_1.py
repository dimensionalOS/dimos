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

"""ReplicaCAD apt_1: synthetic FRL apartment, 1 floor, ~85 m² (~900 sqft).
Fully furnished: sofa far from TV, bicycles, beanbags, many small objects.
Articulated furniture (fridge, counter, cupboards, door) does not load: physics is off."""

from collections.abc import Callable
import os
from typing import TypeVar

from dimos.evals.environments.habitat import HabitatEnvironment
from dimos.evals.scorers import choice, exact, first_number, numeric, rank_order, ranking, yes_no
from dimos.evals.types import EvalCase, Outcome, Suite
from dimos.utils.data import get_data_dir

T = TypeVar("T")

SCENE_KEY = "replicacad_apt_1"
SCENE_NAME = "ReplicaCAD scene 1"

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
        scene_id="apt_1",
        seed=0,
        blueprint=["habitat-nav", "mcp-server", "observe-skill", "point-nav-skill-container"],
    )


SUITE: Suite = [
    EvalCase(
        id=f"{SCENE_KEY}_bicycles",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "How many bicycles are in the scene? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(2, value)),
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
        id=f"{SCENE_KEY}_stools",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "How many stools are in the scene? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(2, value)),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_chairs",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many chairs are in the scene, excluding stools and beanbags? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(6, value)),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_plants",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many indoor potted plants are in the scene? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(3, value)),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_remotes",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many remote controls are in the scene? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(2, value)),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_umbrella",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "Is there an umbrella in the scene? Return only yes or no.",
        grade=_parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_books",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many individual books are in the scene? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(21, value)),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_sofa_width",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate width of the sofa, in meters? Return only the number.",
        grade=_parsed(first_number, lambda value: numeric(2.14, value, tolerance=0.10, band=0.40)),
        tags=frozenset({"dimensions", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_stand_height",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate height of the TV stand, in meters? Return only the number.",
        grade=_parsed(first_number, lambda value: numeric(0.60, value, tolerance=0.05, band=0.20)),
        tags=frozenset({"dimensions", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_sofa_stand_distance",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate horizontal straight-line distance between the centers of the sofa and TV stand, in meters? Return only the number.",
        grade=_parsed(first_number, lambda value: numeric(6.04, value, tolerance=0.30, band=1.2)),
        tags=frozenset({"distance", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_nearest_bicycle",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate horizontal straight-line distance from the center of the sofa to the center of the nearest bicycle, in meters? Return only the number.",
        grade=_parsed(first_number, lambda value: numeric(1.15, value, tolerance=0.15, band=0.60)),
        tags=frozenset({"distance", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_height_order",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Order these objects from shortest to tallest. A) Sofa; B) TV stand; C) A bicycle. Return all three letters once in order, optionally separated by commas.",
        grade=_parsed(ranking, lambda value: rank_order("BAC", value)),
        tags=frozenset({"dimensions", "ranking"}),
    ),
]

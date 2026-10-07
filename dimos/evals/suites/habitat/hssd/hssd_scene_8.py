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

"""HSSD 107734110_175999914: synthetic home, 1 floor, ~130 m² (~1,400 sqft) indoor.
Sparse 1-bed: digital-piano living room, kitchen, office with sofa bed, utility, bath, closets."""

from collections.abc import Callable
import os
from typing import TypeVar

from dimos.evals.environments.habitat import HabitatEnvironment
from dimos.evals.scorers import choice, exact, first_number, numeric, yes_no
from dimos.evals.types import EvalCase, Outcome, Suite
from dimos.utils.data import get_data_dir

T = TypeVar("T")

SCENE_KEY = "hssd_107734110_175999914"
SCENE_NAME = "HSSD scene 8"

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
        scene_id="107734110_175999914",
        seed=0,
        blueprint=["habitat-nav", "mcp-server", "observe-skill", "point-nav-skill-container"],
    )


SUITE: Suite = [
    EvalCase(
        id=f"{SCENE_KEY}_bedrooms",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "How many bedrooms are in the home? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(1, value)),
        tags=frozenset({"rooms", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_closets",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many separate closet spaces are in the home? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(2, value)),
        tags=frozenset({"rooms", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_piano_location",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Which room contains the piano? A) Bedroom; B) Office; C) Kitchen; D) Living room. Return only A, B, C, or D.",
        grade=_parsed(_LETTER, lambda value: exact("D", value)),
        tags=frozenset({"object-location", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_computer_location",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Which room contains the desktop computer? A) Office; B) Living room; C) Kitchen; D) Bedroom. Return only A, B, C, or D.",
        grade=_parsed(_LETTER, lambda value: exact("A", value)),
        tags=frozenset({"object-location", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_sofa_office",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "Is there a sofa in the office? Return only yes or no.",
        grade=_parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_televisions",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many televisions are in the home? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(3, value)),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_living_area",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate living-room area, in square meters? Return only the number.",
        grade=_parsed(first_number, lambda value: numeric(51.53, value, tolerance=3, band=10)),
        tags=frozenset({"area", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_piano_width",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate width of the digital piano, in meters? Return only the number.",
        grade=_parsed(first_number, lambda value: numeric(1.33, value, tolerance=0.08, band=0.3)),
        tags=frozenset({"dimensions", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_office_doorway_radius",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the largest circular robot radius that fits through the office doorway in 2D, based on the structural opening width, in meters? Return only the number.",
        # Stage slice Y=1 m at X=-4.79/-4.85/-4.90: Z gap [-3.997643,-3.017643].
        # Width .98 m / 2 is structural clearance only, not a furnished-route guarantee.
        grade=_parsed(first_number, lambda value: numeric(0.49, value, tolerance=0.025, band=0.10)),
        tags=frozenset({"clearance", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_piano_computer_distance",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How far is the piano from the office computer in a horizontal straight line, in meters? Return only the number.",
        # Horizontal visual-AABB centers including node transforms/instance scale: 11.635427 m.
        grade=_parsed(first_number, lambda value: numeric(11.64, value, tolerance=0.5, band=2)),
        tags=frozenset({"distance", "numeric"}),
    ),
]

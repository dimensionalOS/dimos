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

"""HSSD 103997970_171031287: synthetic home, 1 floor, ~76 m² (~820 sqft) indoor.
Three rooms: open living/kitchen/dining with round table, bedroom, bathroom; plants throughout."""

from collections.abc import Callable
import os
from typing import TypeVar

from dimos.evals.environments.habitat import HabitatEnvironment
from dimos.evals.scorers import choice, exact, first_number, numeric, yes_no
from dimos.evals.types import EvalCase, Outcome, Suite
from dimos.utils.data import get_data_dir

T = TypeVar("T")

SCENE_KEY = "hssd_103997970_171031287"
SCENE_NAME = "HSSD scene 4"

INSTRUCTION = (
    "You are answering questions about a live simulated home. You control the robot, "
    "and its sensor recording grows as it observes the environment. Initial observations "
    "do not cover the whole home. Move around to gather the evidence needed to answer "
    "the question. Inspect relevant interior rooms for counts and absence claims. "
    "Only indoor areas are in scope. Use observations rather than assumptions about "
    "a typical home. When you have enough evidence, return the answer in the requested format."
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
        scene_id="103997970_171031287",
        seed=0,
        blueprint=["habitat-nav", "mcp-server", "observe-skill", "point-nav-skill-container"],
    )


SUITE: Suite = [
    EvalCase(
        id="hssd_103997970_171031287_bedrooms",
        inputs=INSTRUCTION + "\n\nHow many bedrooms are in the home? Return only the count.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: exact(1, value)),
        timeout_s=1200,
        tags=frozenset({"rooms", "count"}),
    ),
    EvalCase(
        id="hssd_103997970_171031287_dining_exists",
        inputs=INSTRUCTION
        + "\n\nIs there a separately enclosed dining room? Return only yes or no.",
        environment=_environment(),
        grade=_parsed(yes_no, lambda value: exact("no", value)),
        timeout_s=1200,
        tags=frozenset({"rooms", "boolean"}),
    ),
    EvalCase(
        id="hssd_103997970_171031287_largest_room",
        inputs=INSTRUCTION
        + "\n\nWhich room is largest by floor area? A) Bedroom; B) Bathroom; C) Open-plan living/kitchen/dining room. Return only A, B, or C.",
        environment=_environment(),
        grade=_parsed(_LETTER, lambda value: exact("C", value)),
        timeout_s=1200,
        tags=frozenset({"rooms", "area", "single-choice"}),
    ),
    EvalCase(
        id="hssd_103997970_171031287_smallest_room",
        inputs=INSTRUCTION
        + "\n\nWhich room is smallest by floor area? A) Open-plan living/kitchen/dining room; B) Bathroom; C) Bedroom. Return only A, B, or C.",
        environment=_environment(),
        grade=_parsed(_LETTER, lambda value: exact("B", value)),
        timeout_s=1200,
        tags=frozenset({"rooms", "area", "single-choice"}),
    ),
    EvalCase(
        id="hssd_103997970_171031287_kitchen_area",
        inputs=INSTRUCTION
        + "\n\nWhat is the approximate floor area of the kitchen area, in square meters? Return only the number.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: numeric(5.36, value, tolerance=0.4, band=1.5)),
        timeout_s=1200,
        tags=frozenset({"area", "numeric"}),
    ),
    EvalCase(
        id="hssd_103997970_171031287_bedroom_perimeter",
        inputs=INSTRUCTION
        + "\n\nWhat is the approximate bedroom perimeter, in meters? Return only the number.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: numeric(18.49, value, tolerance=0.8, band=3)),
        timeout_s=1200,
        tags=frozenset({"perimeter", "numeric"}),
    ),
    EvalCase(
        id="hssd_103997970_171031287_bathtub_exists",
        inputs=INSTRUCTION + "\n\nDoes the bathroom contain a bathtub? Return only yes or no.",
        environment=_environment(),
        grade=_parsed(yes_no, lambda value: exact("yes", value)),
        timeout_s=1200,
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id="hssd_103997970_171031287_room_count",
        inputs=INSTRUCTION + "\n\nHow many rooms are in the home? Return only the count.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: exact(3, value)),
        timeout_s=1200,
        tags=frozenset({"rooms", "count"}),
    ),
    EvalCase(
        id="hssd_103997970_171031287_laptop_exists",
        inputs=INSTRUCTION + "\n\nIs there a laptop anywhere in the home? Return only yes or no.",
        environment=_environment(),
        grade=_parsed(yes_no, lambda value: exact("no", value)),
        timeout_s=1200,
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id="hssd_103997970_171031287_dining_table_diameter",
        inputs=INSTRUCTION
        + "\n\nWhat is the approximate diameter of the round dining table, in meters? Return only the number.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: numeric(1.60, value, tolerance=0.1, band=0.35)),
        timeout_s=1200,
        tags=frozenset({"dimensions", "numeric"}),
    ),
    EvalCase(
        id="hssd_103997970_171031287_tv_location",
        inputs=INSTRUCTION
        + "\n\nWhich area contains the television? A) Bedroom; B) Bathroom; C) Living area; D) Kitchen area. Return only A, B, C, or D.",
        environment=_environment(),
        grade=_parsed(_LETTER, lambda value: exact("C", value)),
        timeout_s=1200,
        tags=frozenset({"object-location", "single-choice"}),
    ),
    EvalCase(
        id="hssd_103997970_171031287_every_room_plants",
        inputs=INSTRUCTION
        + "\n\nAre there plants in every room in the home? Return only yes or no.",
        environment=_environment(),
        grade=_parsed(yes_no, lambda value: exact("yes", value)),
        timeout_s=1200,
        tags=frozenset({"spatial-relation", "boolean"}),
    ),
]

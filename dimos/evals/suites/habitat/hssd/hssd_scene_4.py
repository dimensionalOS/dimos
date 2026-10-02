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

import os

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.evals.environments.habitat import HabitatEnvironment
from dimos.evals.scorers import exact, first_number, numeric, yes_no
from dimos.evals.suites.lib.habitat_qa import parsed as _parsed
from dimos.evals.types import EvalCase, Suite

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


def _environment() -> HabitatEnvironment:
    return HabitatEnvironment(
        scene_dataset_config=os.environ.get(
            "HSSD_DATASET_CONFIG",
            str(
                DIMOS_PROJECT_ROOT
                / "target/habitat/data/hssd-hab/hssd-hab.scene_dataset_config.json"
            ),
        ),
        scene_id="103997970_171031287",
        seed=0,
        blueprint=["habitat-nav", "mcp-server", "observe-skill"],
    )


SUITE: Suite = [
    EvalCase(
        id="hssd_103997970_171031287_bedrooms",
        inputs=INSTRUCTION + "\n\nHow many bedrooms are in the home? Return only the count.",
        environment=_environment(),
        grade=_parsed(first_number, lambda v: exact(1, v)),
        timeout_s=1200,
        tags=frozenset({"rooms", "count"}),
    ),
    EvalCase(
        id="hssd_103997970_171031287_dining_exists",
        inputs=INSTRUCTION
        + "\n\nIs there a separately enclosed dining room? Return only yes or no.",
        environment=_environment(),
        grade=_parsed(yes_no, lambda v: exact("no", v)),
        timeout_s=1200,
        tags=frozenset({"rooms", "boolean"}),
    ),
    EvalCase(
        id="hssd_103997970_171031287_largest_room",
        inputs=INSTRUCTION
        + "\n\nWhich room is largest by floor area? A) Bedroom; B) Bathroom; C) Open-plan living/kitchen/dining room. Return only A, B, or C.",
        environment=_environment(),
        grade=lambda o: exact("C", o.trajectory.final_answer.strip().upper()),
        timeout_s=1200,
        tags=frozenset({"rooms", "area", "single-choice"}),
    ),
    EvalCase(
        id="hssd_103997970_171031287_smallest_room",
        inputs=INSTRUCTION
        + "\n\nWhich room is smallest by floor area? A) Open-plan living/kitchen/dining room; B) Bathroom; C) Bedroom. Return only A, B, or C.",
        environment=_environment(),
        grade=lambda o: exact("B", o.trajectory.final_answer.strip().upper()),
        timeout_s=1200,
        tags=frozenset({"rooms", "area", "single-choice"}),
    ),
    EvalCase(
        id="hssd_103997970_171031287_kitchen_area",
        inputs=INSTRUCTION
        + "\n\nWhat is the approximate floor area of the kitchen area, in square meters? Return only the number.",
        environment=_environment(),
        grade=_parsed(first_number, lambda v: numeric(5.36, v, tolerance=0.4, band=1.5)),
        timeout_s=1200,
        tags=frozenset({"area", "numeric"}),
    ),
    EvalCase(
        id="hssd_103997970_171031287_bedroom_perimeter",
        inputs=INSTRUCTION
        + "\n\nWhat is the approximate bedroom perimeter, in meters? Return only the number.",
        environment=_environment(),
        grade=_parsed(first_number, lambda v: numeric(18.49, v, tolerance=0.8, band=3)),
        timeout_s=1200,
        tags=frozenset({"perimeter", "numeric"}),
    ),
    EvalCase(
        id="hssd_103997970_171031287_bathtub_exists",
        inputs=INSTRUCTION + "\n\nDoes the bathroom contain a bathtub? Return only yes or no.",
        environment=_environment(),
        grade=_parsed(yes_no, lambda v: exact("yes", v)),
        timeout_s=1200,
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id="hssd_103997970_171031287_room_count",
        inputs=INSTRUCTION + "\n\nHow many rooms are in the home? Return only the count.",
        environment=_environment(),
        grade=_parsed(first_number, lambda v: exact(3, v)),
        timeout_s=1200,
        tags=frozenset({"rooms", "count"}),
    ),
    EvalCase(
        id="hssd_103997970_171031287_laptop_exists",
        inputs=INSTRUCTION + "\n\nIs there a laptop anywhere in the home? Return only yes or no.",
        environment=_environment(),
        grade=_parsed(yes_no, lambda v: exact("no", v)),
        timeout_s=1200,
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id="hssd_103997970_171031287_dining_table_diameter",
        inputs=INSTRUCTION
        + "\n\nWhat is the approximate diameter of the round dining table, in meters? Return only the number.",
        environment=_environment(),
        grade=_parsed(first_number, lambda v: numeric(1.60, v, tolerance=0.1, band=0.35)),
        timeout_s=1200,
        tags=frozenset({"dimensions", "numeric"}),
    ),
    EvalCase(
        id="hssd_103997970_171031287_tv_location",
        inputs=INSTRUCTION
        + "\n\nWhich area contains the television? A) Bedroom; B) Bathroom; C) Living area; D) Kitchen area. Return only A, B, C, or D.",
        environment=_environment(),
        grade=lambda o: exact("C", o.trajectory.final_answer.strip().upper()),
        timeout_s=1200,
        tags=frozenset({"object-location", "single-choice"}),
    ),
    EvalCase(
        id="hssd_103997970_171031287_every_room_plants",
        inputs=INSTRUCTION
        + "\n\nAre there plants in every room in the home? Return only yes or no.",
        environment=_environment(),
        grade=_parsed(yes_no, lambda v: exact("yes", v)),
        timeout_s=1200,
        tags=frozenset({"spatial-relation", "boolean"}),
    ),
]

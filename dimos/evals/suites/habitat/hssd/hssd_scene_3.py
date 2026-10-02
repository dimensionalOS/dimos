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
from dimos.evals.scorers import exact, first_number, numeric, rank_order, ranking, yes_no
from dimos.evals.suites.lib.habitat_qa import parsed as _parsed
from dimos.evals.types import EvalCase, Suite

SCENE_KEY = "hssd_103997424_171030444"
SCENE_NAME = "HSSD scene 3"

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
        scene_id="103997424_171030444",
        seed=0,
        blueprint=["habitat-nav", "mcp-server", "observe-skill"],
    )


SUITE: Suite = [
    EvalCase(
        id="hssd_103997424_171030444_bedrooms",
        inputs=INSTRUCTION + "\n\nHow many bedrooms are in the home? Return only the count.",
        environment=_environment(),
        grade=_parsed(first_number, lambda v: exact(1, v)),
        timeout_s=1200,
        tags=frozenset({"rooms", "count"}),
    ),
    EvalCase(
        id="hssd_103997424_171030444_office_exists",
        inputs=INSTRUCTION + "\n\nIs there an office in the home? Return only yes or no.",
        environment=_environment(),
        grade=_parsed(yes_no, lambda v: exact("yes", v)),
        timeout_s=1200,
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id="hssd_103997424_171030444_computer_location",
        inputs=INSTRUCTION
        + "\n\nWhich room contains the desktop computer? A) Kitchen; B) Bedroom; C) Office; D) Bathroom. Return only A, B, C, or D.",
        environment=_environment(),
        grade=lambda o: exact("C", o.trajectory.final_answer.strip().upper()),
        timeout_s=1200,
        tags=frozenset({"object-location", "single-choice"}),
    ),
    EvalCase(
        id="hssd_103997424_171030444_sofa_in_kitchen",
        inputs=INSTRUCTION + "\n\nIs there a sofa in the kitchen area? Return only yes or no.",
        environment=_environment(),
        grade=_parsed(yes_no, lambda v: exact("yes", v)),
        timeout_s=1200,
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id="hssd_103997424_171030444_bathtub_exists",
        inputs=INSTRUCTION + "\n\nIs there a bathtub in the bathroom? Return only yes or no.",
        environment=_environment(),
        grade=_parsed(yes_no, lambda v: exact("yes", v)),
        timeout_s=1200,
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id="hssd_103997424_171030444_largest_room",
        inputs=INSTRUCTION
        + "\n\nWhich is larger by floor area, the living room or the kitchen? A) Living room; B) Kitchen. Return only A or B.",
        environment=_environment(),
        grade=lambda o: exact("A", o.trajectory.final_answer.strip().upper()),
        timeout_s=1200,
        tags=frozenset({"rooms", "area", "single-choice"}),
    ),
    EvalCase(
        id="hssd_103997424_171030444_kitchen_area",
        inputs=INSTRUCTION
        + "\n\nWhat is the approximate kitchen floor area, in square meters? Return only the number.",
        environment=_environment(),
        grade=_parsed(first_number, lambda v: numeric(30.93, v, tolerance=2, band=7)),
        timeout_s=1200,
        tags=frozenset({"area", "numeric"}),
    ),
    EvalCase(
        id="hssd_103997424_171030444_dining_perimeter",
        inputs=INSTRUCTION
        + "\n\nWhat is the approximate perimeter of the dining room, in meters? Return only the number.",
        environment=_environment(),
        grade=_parsed(first_number, lambda v: numeric(12.31, v, tolerance=0.6, band=2.5)),
        timeout_s=1200,
        tags=frozenset({"perimeter", "numeric"}),
    ),
    EvalCase(
        id="hssd_103997424_171030444_fridge_height",
        inputs=INSTRUCTION
        + "\n\nWhat is the approximate refrigerator height, in meters? Return only the number.",
        environment=_environment(),
        grade=_parsed(first_number, lambda v: numeric(1.68, v, tolerance=0.1, band=0.4)),
        timeout_s=1200,
        tags=frozenset({"dimensions", "numeric"}),
    ),
    EvalCase(
        id="hssd_103997424_171030444_area_order",
        inputs=INSTRUCTION
        + "\n\nOrder these rooms from smallest to largest floor area. A) Dining room; B) Office; C) Bedroom. Return all three letters once in order, optionally separated by commas.",
        environment=_environment(),
        grade=_parsed(ranking, lambda v: rank_order("BAC", v)),
        timeout_s=1200,
        tags=frozenset({"area", "ranking"}),
    ),
    EvalCase(
        id="hssd_103997424_171030444_dining_table_diagonal",
        inputs=INSTRUCTION
        + "\n\nWhat is the approximate diagonal length of the rectangular dining tabletop, in meters? Return only the number.",
        environment=_environment(),
        # Visual tabletop extents 2.131282 × 1.168271 m; horizontal diagonal 2.430477 m.
        grade=_parsed(first_number, lambda v: numeric(2.43, v, tolerance=0.12, band=0.45)),
        timeout_s=1200,
        tags=frozenset({"dimensions", "numeric"}),
    ),
    EvalCase(
        id="hssd_103997424_171030444_computer_bed_wall",
        inputs=INSTRUCTION
        + "\n\nAre the office computer and bed on opposite sides of the same wall? Return only yes or no.",
        environment=_environment(),
        grade=_parsed(yes_no, lambda v: exact("yes", v)),
        timeout_s=1200,
        tags=frozenset({"spatial-relation", "boolean"}),
    ),
    EvalCase(
        id="hssd_103997424_171030444_adjacent_red_objects",
        inputs=INSTRUCTION
        + "\n\nWhich pair consists of two red objects beside each other? A) Sofa and table; B) Bed and table; C) Chair and table; D) Sofa and refrigerator. Return only A, B, C, or D.",
        environment=_environment(),
        grade=lambda o: exact("D", o.trajectory.final_answer.strip().upper()),
        timeout_s=1200,
        tags=frozenset({"visual-attribute", "spatial-relation", "single-choice"}),
    ),
]

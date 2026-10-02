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

SCENE_KEY = "hssd_102344193"
SCENE_NAME = "HSSD scene 1"

INSTRUCTION = (
    "You are answering questions about a live simulated home. "
    "You control the robot, and its sensor recording grows as it observes the environment. "
    "Initial observations do not cover the whole home. "
    "Move around to gather the evidence needed to answer the question. "
    "Inspect the relevant interior rooms for counts and absence claims. "
    "Only indoor areas are in scope; do not attempt to enter outdoor areas. "
    "Use observations rather than assumptions about a typical home. "
    "When you have enough evidence, return the answer in the requested format."
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
        scene_id="102344193",
        seed=0,
        # Interior living-room point; do not sample an outdoor spawn.
        start_position_ros_override=(3.0, 5.5, 0.124386),
        blueprint=["habitat-nav", "mcp-server", "observe-skill"],
    )


SUITE: Suite = [
    EvalCase(
        id="hssd_102344193_bedrooms",
        inputs=INSTRUCTION + "\n\nHow many bedrooms are in the home? Return only the count.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: exact(1, value)),
        timeout_s=1200,
        tags=frozenset({"rooms", "count"}),
    ),
    EvalCase(
        id="hssd_102344193_bathroom_count",
        inputs=INSTRUCTION + "\n\nHow many bathrooms are in the home? Return only the count.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: exact(1, value)),
        timeout_s=1200,
        tags=frozenset({"rooms", "count"}),
    ),
    EvalCase(
        id="hssd_102344193_largest_room",
        inputs=INSTRUCTION
        + "\n\nWhich room has the largest floor area? A) Bedroom; B) Living room; C) Kitchen; D) Bathroom. Return only A, B, C, or D.",
        environment=_environment(),
        grade=lambda o: exact("B", o.trajectory.final_answer.strip().upper()),
        timeout_s=1200,
        tags=frozenset({"rooms", "area", "single-choice"}),
    ),
    EvalCase(
        id="hssd_102344193_living_area",
        inputs=INSTRUCTION
        + "\n\nWhat is the approximate floor area of the living room, in square meters? Return only the number.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: numeric(47.23, value, tolerance=2.5, band=10)),
        timeout_s=1200,
        tags=frozenset({"rooms", "area", "numeric"}),
    ),
    EvalCase(
        id="hssd_102344193_bedroom_perimeter",
        inputs=INSTRUCTION
        + "\n\nWhat is the approximate perimeter of the bedroom, in meters? Return only the number.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: numeric(15.55, value, tolerance=0.75, band=3)),
        timeout_s=1200,
        tags=frozenset({"perimeter", "numeric"}),
    ),
    EvalCase(
        id="hssd_102344193_laptop_location",
        inputs=INSTRUCTION
        + "\n\nWhich room contains the laptop? A) Living room; B) Kitchen; C) Bedroom; D) Bathroom. Return only A, B, C, or D.",
        environment=_environment(),
        grade=lambda o: exact("C", o.trajectory.final_answer.strip().upper()),
        timeout_s=1200,
        tags=frozenset({"object-location", "single-choice"}),
    ),
    EvalCase(
        id="hssd_102344193_laundry_exists",
        inputs=INSTRUCTION
        + "\n\nIs there a washer-dryer in the laundry area? Return only yes or no.",
        environment=_environment(),
        grade=_parsed(yes_no, lambda value: exact("yes", value)),
        timeout_s=1200,
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id="hssd_102344193_fridge_height",
        inputs=INSTRUCTION
        + "\n\nWhat is the approximate height of the refrigerator, in meters? Return only the number.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: numeric(1.68, value, tolerance=0.10, band=0.40)),
        timeout_s=1200,
        tags=frozenset({"dimensions", "numeric"}),
    ),
    EvalCase(
        id="hssd_102344193_room_area_order",
        inputs=INSTRUCTION
        + "\n\nWhat is the order of these rooms from smallest to largest floor area? A) Bathroom; B) Bedroom; C) Kitchen. Return all three letters once in order, optionally separated by commas.",
        environment=_environment(),
        grade=_parsed(ranking, lambda value: rank_order("ACB", value)),
        timeout_s=1200,
        tags=frozenset({"area", "ranking"}),
    ),
    EvalCase(
        id="hssd_102344193_fridge_state",
        inputs=INSTRUCTION
        + "\n\nIs the refrigerator open or closed? A) Open; B) Closed. Return only A or B.",
        environment=_environment(),
        grade=lambda o: exact("B", o.trajectory.final_answer.strip().upper()),
        timeout_s=1200,
        tags=frozenset({"object-state", "single-choice"}),
    ),
    EvalCase(
        id="hssd_102344193_laptop_tv_distance",
        inputs=INSTRUCTION
        + "\n\nHow far is the laptop from the television in a straight line, in meters? Return only the number.",
        environment=_environment(),
        # Horizontal transformed visual-AABB centers; measured 11.459718 m.
        grade=_parsed(first_number, lambda value: numeric(11.46, value, tolerance=0.5, band=2)),
        timeout_s=1200,
        tags=frozenset({"distance", "numeric"}),
    ),
]

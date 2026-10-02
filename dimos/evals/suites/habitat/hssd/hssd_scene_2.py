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

SCENE_KEY = "hssd_102344403"
SCENE_NAME = "HSSD scene 2"

INSTRUCTION = (
    "You are answering questions about a live simulated home. You control the robot, "
    "and its sensor recording grows as it observes the environment. Initial observations "
    "do not cover the whole home. Move around to gather the evidence needed to answer "
    "the question. Inspect relevant interior rooms for counts and absence claims. "
    "Only indoor areas, including the garage, are in scope. "
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
        scene_id="102344403",
        seed=0,
        start_position_ros_override=(3.713, 6.3, 0.159347),
        blueprint=["habitat-nav", "mcp-server", "observe-skill"],
    )


SUITE: Suite = [
    EvalCase(
        id="hssd_102344403_bedrooms",
        inputs=INSTRUCTION + "\n\nHow many bedrooms are in the home? Return only the count.",
        environment=_environment(),
        grade=_parsed(first_number, lambda v: exact(3, v)),
        timeout_s=1200,
        tags=frozenset({"rooms", "count"}),
    ),
    EvalCase(
        id="hssd_102344403_game_room_exists",
        inputs=INSTRUCTION + "\n\nIs there a recreation or games room? Return only yes or no.",
        environment=_environment(),
        grade=_parsed(yes_no, lambda v: exact("yes", v)),
        timeout_s=1200,
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id="hssd_102344403_piano_location",
        inputs=INSTRUCTION
        + "\n\nWhich room contains the grand piano? A) Living room; B) Office; C) Recreation room; D) Gym. Return only A, B, C, or D.",
        environment=_environment(),
        grade=lambda o: exact("C", o.trajectory.final_answer.strip().upper()),
        timeout_s=1200,
        tags=frozenset({"object-location", "single-choice"}),
    ),
    EvalCase(
        id="hssd_102344403_treadmill_location",
        inputs=INSTRUCTION
        + "\n\nWhich room contains the treadmill? A) Gym; B) Garage; C) Office; D) Lounge. Return only A, B, C, or D.",
        environment=_environment(),
        grade=lambda o: exact("A", o.trajectory.final_answer.strip().upper()),
        timeout_s=1200,
        tags=frozenset({"object-location", "single-choice"}),
    ),
    EvalCase(
        id="hssd_102344403_kitchen_fridges",
        inputs=INSTRUCTION
        + "\n\nHow many refrigerator units are in the kitchen? Return only the count.",
        environment=_environment(),
        grade=_parsed(first_number, lambda v: exact(2, v)),
        timeout_s=1200,
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id="hssd_102344403_living_area",
        inputs=INSTRUCTION
        + "\n\nWhat is the approximate living-room floor area, in square meters? Return only the number.",
        environment=_environment(),
        grade=_parsed(first_number, lambda v: numeric(80.97, v, tolerance=4, band=16)),
        timeout_s=1200,
        tags=frozenset({"area", "numeric"}),
    ),
    EvalCase(
        id="hssd_102344403_area_order",
        inputs=INSTRUCTION
        + "\n\nOrder these spaces from smallest to largest floor area. A) Recreation room; B) Lounge; C) Garage. Return all three letters once in order, optionally separated by commas.",
        environment=_environment(),
        grade=_parsed(ranking, lambda v: rank_order("BAC", v)),
        timeout_s=1200,
        tags=frozenset({"area", "ranking"}),
    ),
    EvalCase(
        id="hssd_102344403_fridge_height",
        inputs=INSTRUCTION
        + "\n\nWhat is the approximate height of a kitchen refrigerator unit, in meters? Return only the number.",
        environment=_environment(),
        grade=_parsed(first_number, lambda v: numeric(2.54, v, tolerance=0.15, band=0.5)),
        timeout_s=1200,
        tags=frozenset({"dimensions", "numeric"}),
    ),
    EvalCase(
        id="hssd_102344403_garage_cars",
        inputs=INSTRUCTION + "\n\nHow many cars are in the garage? Return only the count.",
        environment=_environment(),
        grade=_parsed(first_number, lambda v: exact(3, v)),
        timeout_s=1200,
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id="hssd_102344403_every_bedroom_tv",
        inputs=INSTRUCTION + "\n\nDoes every bedroom have a television? Return only yes or no.",
        environment=_environment(),
        grade=_parsed(yes_no, lambda v: exact("no", v)),
        timeout_s=1200,
        tags=frozenset({"spatial-relation", "boolean"}),
    ),
    EvalCase(
        id="hssd_102344403_arcade_exists",
        inputs=INSTRUCTION
        + "\n\nIs there an arcade machine in the recreation room? Return only yes or no.",
        environment=_environment(),
        grade=_parsed(yes_no, lambda v: exact("no", v)),
        timeout_s=1200,
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id="hssd_102344403_lounge_path_order",
        inputs=INSTRUCTION
        + "\n\nWhat is the order of these objects from nearest to farthest by collision-free travel distance from the lounge entrance facing the living room, for a robot of radius 0.25 m? A) Grand piano; B) Treadmill; C) Nearest kitchen refrigerator. Return all three letters once in order, optionally separated by commas.",
        environment=_environment(),
        # Static-object navmesh, radius .25/height .60 m; .10 m goal grid
        # within 1 m of each anchor. From the configured lounge start:
        # piano 4.575 m, nearest fridge 5.608 m, treadmill 22.549 m.
        grade=_parsed(ranking, lambda v: rank_order("ACB", v)),
        timeout_s=1200,
        tags=frozenset({"distance", "ranking"}),
    ),
    EvalCase(
        id="hssd_102344403_dumbbells",
        inputs=INSTRUCTION + "\n\nHow many dumbbells are in the gym? Return only the count.",
        environment=_environment(),
        grade=_parsed(first_number, lambda v: exact(6, v)),
        timeout_s=1200,
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id="hssd_102344403_smallest_car_color",
        inputs=INSTRUCTION
        + "\n\nWhat color is the smallest car in the garage? A) Blue; B) White; C) Red; D) Black. Return only A, B, C, or D.",
        environment=_environment(),
        grade=lambda o: exact("C", o.trajectory.final_answer.strip().upper()),
        timeout_s=1200,
        tags=frozenset({"visual-attribute", "single-choice"}),
    ),
]

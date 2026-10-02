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

from functools import partial

from dimos.evals.scorers import exact, first_number, numeric, rank_order, ranking, yes_no
from dimos.evals.suites.lib.habitat_qa import (
    HSSD_DATASET,
    INSTRUCTION,
    environment,
    parsed,
)
from dimos.evals.types import EvalCase, Suite

SCENE_KEY = "hssd_106366410_174226806"
SCENE_NAME = "HSSD scene 6"

_environment = partial(environment, "106366410_174226806", "HSSD_DATASET_CONFIG", HSSD_DATASET)
SUITE: Suite = [
    EvalCase(
        id=f"{SCENE_KEY}_gym_exists",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Is there an exercise area in the home? Return only yes or no.",
        grade=parsed(yes_no, lambda v: exact("yes", v)),
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_red_trash_bin_location",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Which room contains the red trash bin? A) Kitchen; B) Combined gym/office; C) Bedroom; D) Living room. Return only A, B, C, or D.",
        grade=lambda o: exact("B", o.trajectory.final_answer.strip().upper()),
        tags=frozenset({"object-location", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_piano_location",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Which room contains the grand piano? A) Dining room; B) Office; C) Living room; D) Gym. Return only A, B, C, or D.",
        grade=lambda o: exact("C", o.trajectory.final_answer.strip().upper()),
        tags=frozenset({"object-location", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_refrigerators",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many refrigerator-freezer units are in the kitchen? Return only the count.",
        grade=parsed(first_number, lambda v: exact(2, v)),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_gym_area",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate floor area of the gym zone, in square meters? Return only the number.",
        grade=parsed(first_number, lambda v: numeric(23.08, v, tolerance=1.5, band=6)),
        tags=frozenset({"area", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_bedrooms",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "How many bedrooms are in the home? Return only the count.",
        grade=parsed(first_number, lambda v: exact(1, v)),
        tags=frozenset({"rooms", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_bed_relative_to_sofa",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "For someone seated on the bedroom sofa facing forward, is the bed to their left or right? A) Left; B) Right. Return only A or B.",
        grade=lambda o: exact("A", o.trajectory.final_answer.strip().upper()),
        tags=frozenset({"spatial-relation", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_refrigerator_height",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate height of a kitchen refrigerator, in meters? Return only the number.",
        grade=parsed(first_number, lambda v: numeric(1.77, v, tolerance=0.1, band=0.4)),
        tags=frozenset({"dimensions", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_area_order",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Order these areas from smallest to largest floor area. A) Office zone; B) Bedroom; C) Kitchen. Return all letters once in order, optionally separated by commas.",
        grade=parsed(ranking, lambda v: rank_order("CAB", v)),
        tags=frozenset({"area", "ranking"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_laundry_appliances",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many washing and drying machines are in the laundry room? Return only the count.",
        grade=parsed(first_number, lambda v: exact(3, v)),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_toilet_room_bathtub",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Does the room containing the toilet also contain a bathtub? Return only yes or no.",
        grade=parsed(yes_no, lambda v: exact("yes", v)),
        tags=frozenset({"spatial-relation", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_bedroom_path_order",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Order these objects from nearest to farthest by collision-free travel distance from the bedroom doorway facing the hallway, for a robot of radius 0.25 m. A) Grand piano; B) Dining table; C) Treadmill. Return all letters once in order, optionally separated by commas.",
        # Source Habitat (2.494610,.150866,-1.863723), static navmesh .25/.60 m.
        # .15 m goal grid within 1.5 m of anchors: table 6.812, treadmill 7.173,
        # piano 16.730 m. Table/treadmill separation is approach-sensitive.
        grade=parsed(ranking, lambda v: rank_order("BCA", v)),
        tags=frozenset({"distance", "ranking"}),
    ),
]

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

from dimos.evals.suites.lib.habitat_qa import (
    INSTRUCTION,
    REPLICACAD_DATASET,
    boolean,
    choice,
    count,
    environment,
    measurement,
    order,
)
from dimos.evals.types import EvalCase, Suite

SCENE_KEY = "replicacad_v3_sc2_staging_00"
SCENE_NAME = "ReplicaCAD scene 4"

_environment = partial(
    environment, "v3_sc2_staging_00", "REPLICACAD_DATASET_CONFIG", REPLICACAD_DATASET
)
SUITE: Suite = [
    EvalCase(
        id=f"{SCENE_KEY}_bicycles",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "How many bicycles are in the scene? Return only the count.",
        grade=count(2),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_chairs",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many chairs are in the scene, excluding the sofa? Return only the count.",
        grade=count(2),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_plants",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many indoor potted plants are in the scene? Return only the count.",
        grade=count(2),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_sofa_exists",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "Is there a sofa in the scene? Return only yes or no.",
        grade=boolean("yes"),
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_fridge_exists",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Is there a refrigerator in the scene? Return only yes or no.",
        grade=boolean("yes"),
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_sofa_width",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate sofa width, in meters? Return only the number.",
        grade=measurement(2.14, 0.1, 0.4),
        tags=frozenset({"dimensions", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_tvstand_height",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate height of the TV stand, in meters? Return only the number.",
        grade=measurement(0.60, 0.05, 0.2),
        tags=frozenset({"dimensions", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_sofa_stand_distance",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate horizontal straight-line distance between the centers of the sofa and TV stand, in meters? Return only the number.",
        grade=measurement(4.83, 0.25, 1),
        tags=frozenset({"distance", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_nearest_bike",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate horizontal straight-line distance from the center of the sofa to the center of the nearest bicycle, in meters? Return only the number.",
        grade=measurement(5.61, 0.3, 1.2),
        tags=frozenset({"distance", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_farthest_bike",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate horizontal straight-line distance from the center of the sofa to the center of the farther bicycle, in meters? Return only the number.",
        grade=measurement(7.17, 0.35, 1.4),
        tags=frozenset({"distance", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_nearer_object",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Which object's center is closer to the sofa's center in a horizontal straight line? A) Nearest bicycle; B) TV stand. Return only the letter.",
        grade=choice("B"),
        tags=frozenset({"distance", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_height_order",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Order these objects from shortest to tallest. A) A bicycle; B) TV stand; C) Sofa. Return all three letters once in order, optionally separated by commas.",
        grade=order("BCA"),
        tags=frozenset({"dimensions", "ranking"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_beanbag_exists",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Is there a beanbag seat anywhere in the scene? Return only yes or no.",
        grade=boolean("no"),
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_books",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "How many books are in the scene? Return only the count.",
        grade=count(0),
        tags=frozenset({"object-count", "count"}),
    ),
]

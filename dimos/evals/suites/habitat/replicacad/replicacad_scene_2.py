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
    count,
    environment,
    measurement,
    order,
)
from dimos.evals.types import EvalCase, Suite

SCENE_KEY = "replicacad_apt_5"
SCENE_NAME = "ReplicaCAD scene 2"

_environment = partial(environment, "apt_5", "REPLICACAD_DATASET_CONFIG", REPLICACAD_DATASET)
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
        id=f"{SCENE_KEY}_beanbags",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many beanbag seats are in the scene? Return only the count.",
        grade=count(2),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_stools",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "How many stools are in the scene? Return only the count.",
        grade=count(1),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_chairs",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many chairs are in the scene, excluding stools and beanbags? Return only the count.",
        grade=count(6),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_plants",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many indoor potted plants are in the scene? Return only the count.",
        grade=count(3),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_bowls",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "How many bowls are in the scene? Return only the count.",
        grade=count(4),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_books",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many individual books are in the scene? Return only the count.",
        grade=count(19),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_umbrella",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "Is there an umbrella in the scene? Return only yes or no.",
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
        grade=measurement(2.14, 0.10, 0.40),
        tags=frozenset({"dimensions", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_stand_height",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate TV-stand height, in meters? Return only the number.",
        grade=measurement(0.60, 0.05, 0.20),
        tags=frozenset({"dimensions", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_sofa_stand_distance",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate horizontal straight-line distance between the centers of the sofa and TV stand, in meters? Return only the number.",
        grade=measurement(2.61, 0.20, 0.8),
        tags=frozenset({"distance", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_nearest_bicycle",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate horizontal straight-line distance from the center of the sofa to the center of the nearest bicycle, in meters? Return only the number.",
        grade=measurement(2.77, 0.20, 0.8),
        tags=frozenset({"distance", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_height_order",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Order these objects from shortest to tallest. A) A bicycle; B) Sofa; C) TV stand. Return all three letters once in order, optionally separated by commas.",
        grade=order("CBA"),
        tags=frozenset({"dimensions", "ranking"}),
    ),
]

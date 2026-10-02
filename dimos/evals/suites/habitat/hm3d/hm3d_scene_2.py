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
    HM3D_ANNOTATED_DATASET,
    INSTRUCTION,
    boolean,
    choice,
    count,
    environment,
)
from dimos.evals.types import EvalCase, Suite

SCENE_KEY = "hm3d_GLAQ4DNUx5U"
SCENE_NAME = "HM3D scene 2"

_environment = partial(
    environment, "00861-GLAQ4DNUx5U", "HM3D_ANNOTATED_DATASET_CONFIG", HM3D_ANNOTATED_DATASET
)
SUITE: Suite = [
    EvalCase(
        id=f"{SCENE_KEY}_mural_location",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Which room has a large colorful graffiti-style wall mural? A) Kitchen; B) Bathroom; C) Bedroom; D) Utility room. Return only the letter.",
        grade=choice("C"),
        tags=frozenset({"visual-attribute", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_every_desk_chair",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "Does every desk have a desk chair? Return only yes or no.",
        grade=boolean("yes"),
        tags=frozenset({"spatial-relation", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_beds",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many beds are in the scanned home? Return only the count.",
        grade=count(4),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_televisions",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many televisions are in the scanned home? Return only the count.",
        grade=count(3),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_toilets",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many toilets are in the scanned home? Return only the count.",
        grade=count(4),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_washing_machines",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many washing machines are in the utility room? Return only the count.",
        grade=count(2),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_exercise_bike_exists",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Is there an exercise bike in the home? Return only yes or no.",
        grade=boolean("yes"),
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_exercise_bike_location",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Which type of room contains the exercise bike? A) Kitchen; B) Bedroom; C) Bathroom; D) Garage. Return only the letter.",
        grade=choice("B"),
        tags=frozenset({"object-location", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_fridge_location",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Which type of room contains the refrigerator? A) Living room; B) Bedroom; C) Utility/laundry room; D) Bathroom. Return only the letter.",
        grade=choice("C"),
        tags=frozenset({"object-location", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_ironing_board_exists",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Is there an ironing board in the utility room? Return only yes or no.",
        grade=boolean("yes"),
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_vacuum_cleaners",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many vacuum cleaners are in the scanned home? Return only the count.",
        grade=count(2),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_bedroom_tvs",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many bedrooms contain a television? Return only the count.",
        grade=count(2),
        tags=frozenset({"spatial-relation", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_every_bedroom_tv",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Does every bedroom have a television? Return only yes or no.",
        grade=boolean("no"),
        tags=frozenset({"spatial-relation", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_ovens",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "How many ovens are in the kitchen? Return only the count.",
        grade=count(2),
        tags=frozenset({"object-count", "count"}),
    ),
]

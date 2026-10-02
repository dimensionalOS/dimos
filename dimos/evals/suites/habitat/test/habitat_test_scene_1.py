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
    TEST_APARTMENT,
    boolean,
    choice,
    count,
    environment,
)
from dimos.evals.types import EvalCase, Suite

SCENE_KEY = "habitat_test_apartment_1"
SCENE_NAME = "Habitat test scene 1"

_environment = partial(
    environment,
    TEST_APARTMENT,
    "HABITAT_TEST_DATASET_CONFIG",
    "default",
    scene_env="HABITAT_TEST_SCENE",
)
SUITE: Suite = [
    EvalCase(
        id=f"{SCENE_KEY}_mirror_shape",
        inputs=INSTRUCTION
        + "\n\nWhat shape is the wall mirror above the dining-room sideboard? A) Rectangular; B) Circular; C) Triangular; D) Hexagonal. Return only the letter.",
        environment=_environment(),
        grade=choice("B"),
        timeout_s=1200,
        tags=frozenset({"visual-attribute", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_window_covering",
        inputs=INSTRUCTION
        + "\n\nWhat type of window covering is used in the living room? A) Horizontal blinds; B) Fabric curtains; C) Exterior shutters; D) No covering. Return only the letter.",
        environment=_environment(),
        grade=choice("A"),
        timeout_s=1200,
        tags=frozenset({"visual-attribute", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_dining_door_state",
        inputs=INSTRUCTION
        + "\n\nIs the dining-room door open or closed? A) Closed; B) Open. Return only the letter.",
        environment=_environment(),
        grade=choice("B"),
        timeout_s=1200,
        tags=frozenset({"object-state", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_tv_location",
        inputs=INSTRUCTION
        + "\n\nWhich room contains the wall-mounted television? A) Dining room; B) Bedroom; C) Living room; D) Bathroom. Return only the letter.",
        environment=_environment(),
        grade=choice("C"),
        timeout_s=1200,
        tags=frozenset({"object-location", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_chess_decoration_count",
        inputs=INSTRUCTION
        + "\n\nHow many oversized chess-piece decorations are on the console beneath the television? Return only the count.",
        environment=_environment(),
        grade=count(2),
        timeout_s=1200,
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_serving_stand_tiers",
        inputs=INSTRUCTION
        + "\n\nHow many tiers does the serving stand on the dining table have? Return only the count.",
        environment=_environment(),
        grade=count(2),
        timeout_s=1200,
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_coffee_table_between",
        inputs=INSTRUCTION
        + "\n\nIs there a coffee table between the sectional sofa and the television? Return only yes or no.",
        environment=_environment(),
        grade=boolean("yes"),
        timeout_s=1200,
        tags=frozenset({"spatial-relation", "boolean"}),
    ),
]

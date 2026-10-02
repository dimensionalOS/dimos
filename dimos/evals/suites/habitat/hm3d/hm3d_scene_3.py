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
    HM3D_DATASET,
    INSTRUCTION,
    boolean,
    choice,
    count,
    environment,
)
from dimos.evals.types import EvalCase, Suite

SCENE_KEY = "hm3d_NBg5UqG3di3"
SCENE_NAME = "HM3D scene 3"

_environment = partial(environment, "00770-NBg5UqG3di3", "HM3D_DATASET_CONFIG", HM3D_DATASET)
SUITE: Suite = [
    EvalCase(
        id=f"{SCENE_KEY}_corridor_panel_color",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What color are the wall panels in the corridor with the gilded vaulted ceiling? A) Green; B) White; C) Red; D) Blue. Return only the letter.",
        grade=choice("C"),
        tags=frozenset({"visual-attribute", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_floor_pattern",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the dominant pattern of the wood floor in the blue-patterned room? A) Checkerboard; B) Herringbone; C) Plain parallel strips; D) Hexagons. Return only the letter.",
        grade=choice("B"),
        tags=frozenset({"visual-attribute", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_fireplace_exists",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Is there a fireplace in the room with blue patterned upper walls and wooden lower panels? Return only yes or no.",
        grade=boolean("yes"),
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_fireplace_location",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Which room contains the fireplace beneath the exposed wooden ceiling? A) Red-and-gold corridor; B) Blue-patterned room with wooden lower walls; C) White corridor; D) Pale-blue decorative room. Return only the letter.",
        grade=choice("B"),
        tags=frozenset({"object-location", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_radiator_below_window",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Are there radiators beneath the windows in the blue-patterned room? Return only yes or no.",
        grade=boolean("yes"),
        tags=frozenset({"spatial-relation", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_blue_room_windows",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many windows are on the long exterior wall of the blue-patterned room? Return only the count.",
        grade=count(3),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_hallway_extinguisher",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Is there a fire extinguisher in the white corridor? Return only yes or no.",
        grade=boolean("yes"),
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_arched_passage",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Is there an arched passage in the pale-blue decorative room? Return only yes or no.",
        grade=boolean("yes"),
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_open_white_door",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Is the white door leading from the red-and-gold corridor into a white room open or closed? A) Open; B) Closed. Return only the letter.",
        grade=choice("A"),
        tags=frozenset({"object-state", "single-choice"}),
    ),
]

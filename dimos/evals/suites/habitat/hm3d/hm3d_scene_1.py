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
    measurement,
    order,
)
from dimos.evals.types import EvalCase, Suite

SCENE_KEY = "hm3d_CFVBbU9Rsyb"
SCENE_NAME = "HM3D scene 1"

_environment = partial(environment, "00337-CFVBbU9Rsyb", "HM3D_DATASET_CONFIG", HM3D_DATASET)
SUITE: Suite = [
    EvalCase(
        id=f"{SCENE_KEY}_sofa_color",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What color is the sofa beneath the framed trousers display in the middle-level living area? A) Blue; B) Green; C) Red; D) White. Return only the letter.",
        grade=choice("C"),
        tags=frozenset({"visual-attribute", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_kitchen_cabinet_color",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What color are the lower kitchen cabinets beside the dining table with rectangular placemats? A) Blue-gray; B) Red; C) Black; D) Yellow. Return only the letter.",
        grade=choice("A"),
        tags=frozenset({"visual-attribute", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_washing_machine_exists",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Is there a washing machine in the scanned environment? Return only yes or no.",
        grade=boolean("yes"),
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_washing_machine_location",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Which type of room contains the washing machine beneath the long worktop? A) Bedroom; B) Utility room; C) Living room; D) Bathroom. Return only the letter.",
        grade=choice("B"),
        tags=frozenset({"object-location", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_utility_high_chair_count",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many child high chairs stand in front of the utility-room worktop? Return only the count.",
        grade=count(2),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_fire_extinguisher_exists",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Is there a wall-mounted fire extinguisher beside a stair landing? Return only yes or no.",
        grade=boolean("yes"),
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_red_sofa_cushion_count",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many dark throw cushions are on the red sofa below the framed trousers display? Return only the count.",
        grade=count(2),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_below_trousers_display",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is directly below the framed trousers display in the red-sofa living area? A) Bed; B) Dining table; C) Washing machine; D) Sofa. Return only the letter.",
        grade=choice("D"),
        tags=frozenset({"object-location", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_bed_below_skylight_exists",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Is there a bed beneath a skylight in a room with a sloped wooden ceiling? Return only yes or no.",
        grade=boolean("yes"),
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_bunk_beds_exists",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Are there bunk beds in the upper-level sleeping area? Return only yes or no.",
        grade=boolean("yes"),
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_blue_armchair_bedroom_wardrobe_state",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Is the wardrobe in the bedroom with the blue armchair and balcony doors open or closed? A) Open; B) Closed. Return only the letter.",
        grade=choice("B"),
        tags=frozenset({"object-state", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_location_elevation_order",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Order these locations from lowest to highest floor elevation. A) Upper-level bunk-bed area; B) Utility room with the washing machine; C) Living area with the red sofa below the framed trousers display. Return all three letters once in order, optionally separated by commas.",
        grade=order("BCA"),
        tags=frozenset({"elevation", "ranking"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_bunk_utility_height_difference",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Approximately how much higher is the floor of the bunk-bed area than the floor of the utility room, in meters? Return only the number.",
        grade=measurement(5.60, 0.20, 1.0),
        tags=frozenset({"elevation", "numeric"}),
    ),
]
